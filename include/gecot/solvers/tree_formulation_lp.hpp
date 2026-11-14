#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <flat_map>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/utility/column_manager.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The tree formulation relaxation, by column generation.
//
// Buying the options X within the budget, c(X) <= B, a case k scores the sum
// over its targets t of
//     v_t(X) = q_t(X) Σ_s q_s(X) P_st(X),
// the target quality with its gains times the source qualities with their
// gains, each times the probability of its most reliable path to t, and the
// criterion φ combines the cases' scores. The relaxation lets each target t
// of case k buy its own set of options S, a tree of value v_tS, as long as it
// is paid for:
//
//     max  φ(z)
//     s.t. z_k <= Σ_{t∈k} Σ_S v_tS y_tS    π_k   contribution
//          Σ_S y_tS <= 1                   σ_t   uniqueness
//          Σ_{S∋o} y_tS <= x_o             λ_to  purchase
//          Σ_o c_o x_o <= B                μ     budget
//          x, y >= 0
//
// Buying X, the trees y_tX = 1 are feasible. The master starts with each
// target's tree without options, so that the first duals value every target,
// and the pricing of a target finds its best reduced cost
//     rc_tS = π_k v_tS - σ_t - Σ_{o∈S} λ_to
// among its trees within the budget, which the LP above does not require:
// this tightens the relaxation. The pricing can miss improving trees, so the
// relaxation value is not a bound.
struct tree_formulation_lp : public mip_solver_base {
    using variable_t = mippp::model_variable<int, double>;
    using constraint_t = mippp::model_constraint<int>;

    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict

    // The pricing of a target t of case k, a MILP on the graph contracted for
    // t, with X_o = 1 if the tree buys o, Φ_a the flow on arc a and F the
    // flow that reaches t:
    //     max  π_k v - Σ_o λ_to X_o,    v = q_t F + Σ_o Δq_to F_o
    //     s.t. F_o <= F,  F_o <= M_t X_o       the target quality gains of t
    //          [u = t] F + Σ_{a∈δ⁺(u)} Φ_a
    //              <= Σ_{a∈δ⁻(u)} p_a Φ_a + q_u + Σ_o Δq_uo X_o   ∀ vertex u
    //          Φ_a <= M_u X_o           ∀ arc a = (u, w) improved by o
    //          Σ_o c_o X_o <= B
    // whose value v at the optimum is v_tS, for S = {o : X_o = 1}.
    template <instance_c I, typename MILP>
    struct pricing_model {
        case_id_t case_id;
        melon::vertex_t<instance_graph_t<I>> target;
        MILP model;
        std::flat_map<option_t, variable_t> X;  // the options that can serve t
        mippp::runtime_linear_expression<variable_t, double> v;
        // a pricing depends only on these duals: those of an unproductive
        // pricing would find no tree again
        std::vector<double> unproductive_duals;

        pricing_model(const I & instance, const double budget,
                      const auto & instance_case, const auto & strong_arcs,
                      const auto & useless_arcs, const auto original_t,
                      const bool pruning)
            : case_id(instance_case.id()), target(original_t) {
            using namespace mippp::operators;
            // the pricing models are solved in parallel
            if constexpr(mippp::has_thread_limit<MILP>)
                model.set_thread_limit(1);
            _set_tolerances(model, 1e-10, 1e-8);
            model.set_maximization();

            const auto [graph, source_quality_map, vertex_options_map,
                        arc_no_map, probability_map, arc_option_map, t] =
                compute_contracted_generalized_flow_graph(
                    instance_case, strong_arcs, useless_arcs, original_t,
                    pruning);

            std::vector<option_t> options;
            options.append_range(std::views::join(std::views::transform(
                vertex_options_map, [](const auto & vertex_options) {
                    return std::views::values(vertex_options);
                })));
            options.append_range(std::views::transform(
                std::views::filter(
                    arc_option_map,
                    [](const auto & o) { return o.has_value(); }),
                [](const auto & o) { return o.value(); }));
            std::ranges::sort(options);
            const auto [first, last] = std::ranges::unique(options);
            options.erase(first, last);
            X.insert_range(
                std::views::transform(options, [this](const option_t o) {
                    return std::make_pair(o, model.add_binary_variable());
                }));
            model.add_constraint(xsum(X, [&](const auto & e) {
                                     const auto & [o, X_o] = e;
                                     return instance.option_cost(o) * X_o;
                                 }) <= budget);

            const auto M = compute_big_M_map(
                graph, source_quality_map, vertex_options_map, probability_map,
                std::views::filter(
                    melon::vertices(graph),
                    [&graph, &arc_option_map, t](const auto & u) {
                        return u == t ||
                               std::ranges::any_of(
                                   melon::out_arcs(graph, u),
                                   [&arc_option_map](const auto & a) {
                                       return arc_option_map[a].has_value();
                                   });
                    }));

            const auto F = model.add_variable();
            v += instance_case.target_quality_map()[original_t] * F;
            for(const auto & [sqg, tqg, o] :
                instance_case.vertex_options_map()[original_t]) {
                const auto F_o = model.add_variable();
                model.add_constraint(F_o <= F);
                model.add_constraint(F_o <= M[t] * X.at(o));
                v += tqg * F_o;
            }

            const auto Phi = model.add_variables(
                graph.num_arcs(),
                [&arc_no_map](const melon::arc_t<decltype(graph)> & a) {
                    return arc_no_map[a];
                });
            auto inflow = [&](const auto & u) {
                return xsum(graph.in_arcs(u),
                            [&](const auto & a) {
                                return probability_map[a] * Phi(a);
                            }) +
                       source_quality_map[u] +
                       xsum(vertex_options_map[u], [this](const auto & e) {
                           const auto & [sqg, o] = e;
                           return sqg * X.at(o);
                       });
            };
            for(const auto & u : melon::vertices(graph)) {
                if(u == t)
                    model.add_constraint(F + xsum(graph.out_arcs(u), Phi) <=
                                         inflow(u));
                else
                    model.add_constraint(xsum(graph.out_arcs(u), Phi) <=
                                         inflow(u));
            }
            for(const auto & a : melon::arcs(graph))
                if(arc_option_map[a].has_value())
                    model.add_constraint(Phi(a) <=
                                         X.at(arc_option_map[a].value()) *
                                             M[melon::arc_source(graph, a)]);
        }
    };

    // set_lp_algorithm only compiles on the models that list the algorithm
    template <typename LP, typename A>
    static void _set_lp_algorithm(LP & model, const A algorithm) {
        if constexpr(mippp::has_lp_algorithm<LP, A>)
            model.set_lp_algorithm(algorithm);
    }

    template <typename LP, typename MILP, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        namespace colgen = mippp::colgen;
        using namespace mippp;
        using namespace mippp::operators;
        check_plain_budget(instance, "tree_lp");
        _check_criterion(instance);
        spdlog::stopwatch sw;
        auto time_up = [&] {
            return time_limit && sw.elapsed().count() > time_limit.value();
        };

        // The master: x, the budget row, and for each case k, z_k with its
        // contribution row, then the rows of its targets, whose pricing
        // models are built along.
        LP model;
        if constexpr(has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();
        const auto x = model.add_variables(
            instance.options().size(), [](const option_t & o) { return o; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * x(o);
                             }) <= budget);
        std::vector<variable_t> z;
        std::vector<constraint_t> contribution_rows;
        std::vector<pricing_model<I, MILP>> targets;
        std::vector<constraint_t> uniqueness_rows;
        std::vector<std::flat_map<option_t, constraint_t>> purchase_rows;
        std::vector<preprocessing_row<>> preprocessing;
        for(const auto & instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget_t{{budget}});
            preprocessing.emplace_back(
                instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map));
            z.push_back(model.add_variable());
            contribution_rows.push_back(model.add_constraint(z.back() <= 0));
            // the vertices of positive quality or quality gain as a target
            const auto first_target = std::ssize(targets);
            for(const auto & t :
                std::views::filter(melon::vertices(instance_case.graph()),
                                   [&](const auto & t) {
                                       return _is_target(instance_case, t);
                                   })) {
                uniqueness_rows.push_back(model.add_constraint(
                    empty_linear_expression<variable_t, double> <= 1));
                targets.emplace_back(instance, budget, instance_case,
                                     strong_arcs_map[t], useless_arcs_map[t], t,
                                     !no_pruning);
            }
            for(const auto & p : std::views::drop(targets, first_target)) {
                auto & rows = purchase_rows.emplace_back();
                for(const option_t o : p.X.keys())
                    rows.emplace(
                        o, model.add_constraint(
                               empty_linear_expression<variable_t, double> <=
                               x(o)));
            }
        }
        _log_preprocessing(preprocessing);
        model.set_objective(std::visit(
            formula_variable_visitor{model,
                                     [&](const case_id_t & k) { return z[k]; }},
            instance.criterion()));

        auto log_master = [&, option_name_max_length = std::ranges::max(
                                  std::ranges::views::transform(
                                      instance.options(), [&](auto && o) {
                                          return instance.option_name(o).size();
                                      }))]() {
            const auto model_solution = model.get_solution();
            spdlog::trace("tree_decomposition relaxation has value: {}",
                          model.get_solution_value());
            for(auto && option : instance.options()) {
                const auto & option_name = instance.option_name(option);
                spdlog::trace("    {:<{}} {}", option_name,
                              option_name_max_length,
                              model_solution[x(option)]);
            }
        };

        // a tree y_tS, the key of the column manager: the index t of its
        // target and its options S, sorted; its value v_tS is the data
        using tree = std::pair<std::size_t, std::vector<option_t>>;
        colgen::column_manager<
            LP, tree, colgen::property_list<colgen::reduced_cost>,
            colgen::property_list<colgen::reduced_cost, colgen::age>, double>
            columns;
        auto add_tree = [&](LP &, const auto & entry) {
            const auto & [t, S] = entry.seed;
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_rows[targets[t].case_id], -entry.data)),
                std::views::single(std::make_pair(uniqueness_rows[t], 1.0)),
                std::views::transform(S, [&](const option_t o) {
                    return std::make_pair(purchase_rows[t].at(o), 1.0);
                })));
        };
        // the tree of each target under the options S, with those it can use
        auto add_start_trees = [&](const instance_solution_t<I> & S) {
            for(const auto & instance_case : instance.cases()) {
                const auto & graph = instance_case.graph();
                auto source_quality_map = instance_case.source_quality_map();
                auto target_quality_map = instance_case.target_quality_map();
                auto probability_map = instance_case.arc_probability_map();
                for(const auto & u : melon::vertices(graph))
                    for(const auto & [sqg, tqg, o] :
                        instance_case.vertex_options_map()[u]) {
                        if(!S[o]) continue;
                        source_quality_map[u] += sqg;
                        target_quality_map[u] += tqg;
                    }
                for(const auto & a : melon::arcs(graph))
                    for(const auto & [improved_prob, o] :
                        instance_case.arc_options_map()[a])
                        if(S[o])
                            probability_map[a] =
                                std::max(probability_map[a], improved_prob);
                for(std::size_t t = 0; t < targets.size(); ++t) {
                    if(targets[t].case_id != instance_case.id()) continue;
                    const auto u = targets[t].target;
                    columns.emplace_master_column(
                        model,
                        tree{t, std::ranges::to<std::vector>(std::views::filter(
                                    targets[t].X.keys(),
                                    [&](const option_t o) { return S[o]; }))},
                        add_tree,
                        target_quality_map[u] *
                            pc_num_vertex_in_flow(graph, source_quality_map,
                                                  probability_map, u));
                }
            }
        };
        add_start_trees(instance.create_option_map(false));
        if(mip_start) {
            if(compute_solution_cost(instance, _mip_start_solution(instance)) >
               budget + feasibility_tol)
                spdlog::warn("MIP start exceeds the budget: ignored");
            else
                add_start_trees(_mip_start_solution(instance));
        }

        // in the master for more than evict_age rounds and unattractive, so
        // nonbasic
        const colgen::conjunction<colgen::above<colgen::age>,
                                  colgen::negative<colgen::reduced_cost>>
            unattractive{colgen::above<colgen::age>{evict_age},
                         colgen::negative<colgen::reduced_cost>{1e-6}};
        if constexpr(!has_remove_variable<LP>)
            if(evict_age)
                spdlog::warn(
                    "This solver cannot remove columns: --evict-age ignored");
        // The barrier without crossover for the master, on the solvers that
        // have one: its central duals halve the rounds, its 1e-8 primal-dual
        // gap is below the pricing test's 1e-7. Not with eviction: the central
        // duals move at every round and the evicted columns come back, in a
        // cycle.
        using barrier_t = lp_algorithm::interior_point_without_crossover;
        const bool barrier = has_lp_algorithm<LP, barrier_t> && !evict_age;
        if(barrier) _set_lp_algorithm(model, barrier_t{});

        // Column generation.
        solve_report report{"converged"};
        std::size_t iterations = 1;  // LP solves
        std::size_t num_evicted = 0;
        for(;; ++iterations) {
            model.solve();
            const auto dual_solution = model.get_dual_solution();
            log_master();
            report_progress({.relaxation = model.get_solution_value(),
                             .round = iterations});

            auto pi = [&](const std::size_t t) {
                return dual_solution[contribution_rows[targets[t].case_id]];
            };
            auto sigma = [&](const std::size_t t) {
                return dual_solution[uniqueness_rows[t]];
            };
            auto lambda = [&](const std::size_t t, const option_t o) {
                return dual_solution[purchase_rows[t].at(o)];
            };
            auto reduced_cost = [&](const tree & y, const double v) {
                const auto & [t, S] = y;
                double rc = pi(t) * v - sigma(t);
                for(const option_t o : S) rc -= lambda(t, o);
                return rc;
            };
            // the pricing test, at the master's dual tolerance: rc > margin
            auto margin = [&](const std::size_t t) {
                return 1e-7 * sigma(t) + 1e-6;
            };

            // the best tree of target t, if rc_tS > margin
            auto price = [&](const std::size_t t)
                -> std::optional<std::pair<tree, double>> {
                pricing_model<I, MILP> & p = targets[t];
                std::vector<double> duals{pi(t), sigma(t)};
                for(const option_t o : p.X.keys())
                    duals.push_back(lambda(t, o));
                if(duals == p.unproductive_duals) return std::nullopt;
                p.model.set_objective(pi(t) * p.v -
                                      xsum(p.X, [&](const auto & e) {
                                          const auto & [o, X_o] = e;
                                          return lambda(t, o) * X_o;
                                      }));
                p.model.solve();
                if(p.model.get_solution_value() - sigma(t) <= margin(t)) {
                    p.unproductive_duals = std::move(duals);
                    return std::nullopt;
                }
                const auto solution = p.model.get_solution();
                return std::pair{
                    tree{t, std::ranges::to<std::vector>(
                                std::views::keys(std::views::filter(
                                    p.X,
                                    [&](const auto & e) {
                                        return solution[e.second] > 0.5;
                                    })))},
                    evaluate(p.v, solution)};
            };

            std::atomic<bool> cut_short = false;  // by a stop
            tbb::concurrent_vector<std::pair<tree, double>> priced_trees;
            tbb::parallel_for(std::size_t{0}, targets.size(),
                              [&](const std::size_t t) {
                                  if(stop_requested || time_up()) {
                                      cut_short = true;
                                      return;
                                  }
                                  if(auto y = price(t))
                                      priced_trees.push_back(std::move(*y));
                              });

            // the improving trees enter the master. A tree priced again stays
            // where it is: the pricing's gap can find it improving while its
            // copy in the master is not, and it would come back every round.
            columns.emplace_pool_columns(priced_trees | std::views::as_rvalue);
            columns.update_columns([&](const auto & entry) {
                return colgen::priced{reduced_cost(entry.seed, entry.data)};
            });
            const std::size_t num_activated = columns.activate_columns(
                model,
                [&](const auto & entry) {
                    return entry.state.template get<colgen::reduced_cost>() >
                           margin(entry.seed.first);
                },
                add_tree);
            if(!num_activated && !cut_short) {
                report_progress({.end = true});
                spdlog::debug("Column generation stopped: converged");
                break;
            }
            if(stop_requested || time_up()) {
                report.status =
                    stop_requested ? "interrupted" : "time limit reached";
                report_progress({.end = true});
                spdlog::warn("Column generation stopped: {}", report.status);
                // Gurobi, CPLEX and COPT give no solution for a master
                // changed since its last solve, HiGHS a zero value
                if(num_activated) model.solve();
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns.evict_columns(model, unattractive);
        }
        // the solution at a vertex, as the simplex gives it
        if(barrier) {
            _set_lp_algorithm(model, lp_algorithm::automatic{});
            model.solve();
        }

        auto solution = instance.create_option_map(false);
        const auto model_solution = model.get_solution();
        for(const auto & o : instance.options())
            solution[o] = model_solution[x(o)];
        log_master();

        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted}};
        report.score_stats = {{"relaxation_value", model.get_solution_value()}};
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
