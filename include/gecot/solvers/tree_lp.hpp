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
#include "gecot/flow_rows.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/column_generation.hpp"
#include "gecot/solvers/flow_preprocessing.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The tree relaxation of docs/formulations/relaxations.md, by column
// generation. Its pricing can miss improving columns, so the value of its
// master is not a bound.
struct tree_lp : public flow_preprocessing {
    using variable_t = mippp::model_variable<int, double>;
    using constraint_t = mippp::model_constraint<int>;

    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict

    // The pricing of a target t of case k: the flow formulation of
    // docs/formulations/flow.md for t alone on its graph G_t, with X_i = 1 if
    // the column buys i, F the flow that reaches t and F_i that of a target
    // quality gain, under the budget, and the objective
    //     max  γ_k v - Σ_i λ_ti X_i,    v = q_t F + Σ_i q^i_t F_i
    // whose value v at the optimum is v_t(S), for S = {i : X_i = 1}.
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
            _set_tolerances(model, mip_gap, 1e-8);
            model.set_maximization();

            const contracted_flow_graph G = compute_contracted_flow_graph(
                instance, instance_case, budget_t{{budget}}, original_t,
                strong_arcs, useless_arcs, big_M_kind::plain, 1e-8, pruning);
            const auto & graph = G.graph;
            const auto & t = G.t;

            std::vector<option_t> options;
            options.append_range(std::views::join(std::views::transform(
                G.source_options_map, [](const auto & vertex_options) {
                    return std::views::values(vertex_options);
                })));
            options.append_range(std::views::transform(
                std::views::filter(
                    G.improvement_option_map,
                    [](const auto & option) { return option.has_value(); }),
                [](const auto & option) { return option.value(); }));
            std::ranges::sort(options);
            const auto [first, last] = std::ranges::unique(options);
            options.erase(first, last);
            X.insert_range(
                std::views::transform(options, [this](const option_t i) {
                    return std::make_pair(i, model.add_binary_variable());
                }));
            model.add_constraint(xsum(X, [&](const auto & e) {
                                     const auto & [i, X_i] = e;
                                     return instance.option_cost(i) * X_i;
                                 }) <= budget);

            const auto F = model.add_variable();
            v += instance_case.target_quality_map()[original_t] * F;
            for(const auto & [source_gain, target_gain, i] :
                instance_case.vertex_options_map()[original_t]) {
                const auto F_i = model.add_variable();
                model.add_constraint(F_i <= F);
                model.add_constraint(F_i <= G.target_big_M(i) * X.at(i));
                v += target_gain * F_i;
            }

            const auto Phi = model.add_variables(
                graph.num_arcs(), [&G](const contracted_flow_graph::arc & a) {
                    return G.arc_no_map[a];
                });
            add_flow_rows(
                model, graph, t, F, Phi,
                [this](const option_t i) { return X.at(i); }, G.probability_map,
                G.source_quality_map, G.source_options_map);
            for(const auto & a : melon::arcs(graph))
                if(G.improvement_option_map[a].has_value())
                    model.add_constraint(
                        Phi(a) <= X.at(G.improvement_option_map[a].value()) *
                                      G.improvement_big_M(a));
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
        LP model;
        if constexpr(has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        _warn_without_numeric_focus<LP>("LP");
        _set_numeric_focus(model);
        _warn_without_numeric_focus<MILP>("pricing MIP");
        model.set_maximization();

        // The pricing models: one per vertex of positive quality or quality
        // gain as a target, case by case
        std::vector<pricing_model<I, MILP>> targets;
        std::vector<preprocessing_row<>> preprocessing;
        for(const auto & instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget_t{{budget}});
            preprocessing.emplace_back(
                instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map));
            for(const auto & t :
                std::views::filter(melon::vertices(instance_case.graph()),
                                   [&](const auto & t) {
                                       return _is_target(instance_case, t);
                                   })) {
                targets.emplace_back(instance, budget, instance_case,
                                     strong_arcs_map[t], useless_arcs_map[t], t,
                                     pruning);
                _set_numeric_focus(targets.back().model);
            }
        }
        _log_preprocessing(preprocessing);

        // The master: x, the budget row, and for each case k: z_k with its
        // contribution row, the uniqueness row of each of its targets, then
        // their purchase rows
        const auto x = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && i) {
                                 return instance.option_cost(i) * x(i);
                             }) <= budget);
        std::vector<variable_t> z;
        std::vector<constraint_t> contribution_rows, uniqueness_rows;
        std::vector<std::flat_map<option_t, constraint_t>> purchase_rows;
        for(const auto & instance_case : instance.cases()) {
            z.push_back(model.add_variable());
            contribution_rows.push_back(model.add_constraint(z.back() <= 0));
            auto case_targets = std::views::filter(
                targets, [k = instance_case.id()](const auto & p) {
                    return p.case_id == k;
                });
            for([[maybe_unused]] const auto & p : case_targets)
                uniqueness_rows.push_back(model.add_constraint(
                    empty_linear_expression<variable_t, double> <= 1));
            for(const auto & p : case_targets) {
                auto & rows = purchase_rows.emplace_back();
                for(const option_t i : p.X.keys())
                    rows.emplace(
                        i, model.add_constraint(
                               empty_linear_expression<variable_t, double> <=
                               x(i)));
            }
        }
        model.set_objective(std::visit(
            formula_variable_visitor{model,
                                     [&](const case_id_t & k) { return z[k]; }},
            instance.criterion()));

        // a master changed since its last solve gives no solution to read
        auto log_master = [&] {
            if(!spdlog::should_log(spdlog::level::trace)) return;
            const auto model_solution = model.get_solution();
            solvers::log_master(
                instance, "tree_lp", model.get_solution_value(),
                [&](const option_t i) { return model_solution[x(i)]; });
        };

        // a tree y_tS, the key of the column manager: the index t of its
        // target and its options S, sorted; its value v_tS is the data
        using tree = std::pair<std::size_t, std::vector<option_t>>;
        column_manager<LP, tree> columns;
        auto add_tree = [&](LP &, const auto & entry) {
            const auto & [t, S] = entry.seed;
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_rows[targets[t].case_id], -entry.data)),
                std::views::single(std::make_pair(uniqueness_rows[t], 1.0)),
                std::views::transform(S, [&](const option_t i) {
                    return std::make_pair(purchase_rows[t].at(i), 1.0);
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
                    for(const auto & [sqg, tqg, i] :
                        instance_case.vertex_options_map()[u]) {
                        if(!S[i]) continue;
                        source_quality_map[u] += sqg;
                        target_quality_map[u] += tqg;
                    }
                for(const auto & a : melon::arcs(graph))
                    for(const auto & [improved_probability, i] :
                        instance_case.arc_options_map()[a])
                        if(S[i])
                            probability_map[a] = std::max(probability_map[a],
                                                          improved_probability);
                for(std::size_t t = 0; t < targets.size(); ++t) {
                    if(targets[t].case_id != instance_case.id()) continue;
                    const auto u = targets[t].target;
                    columns.emplace_master_column(
                        model,
                        tree{t, std::ranges::to<std::vector>(std::views::filter(
                                    targets[t].X.keys(),
                                    [&](const option_t i) { return S[i]; }))},
                        add_tree,
                        target_quality_map[u] *
                            pc_num_vertex_in_flow(graph, source_quality_map,
                                                  probability_map, u));
                }
            }
        };
        add_start_trees(instance.create_option_map(false));
        if(mip_start) {
            const auto start = _mip_start_solution(instance);
            if(const auto defect =
                   _start_defect(instance, start, budget_t{{budget}});
               !defect.empty())
                spdlog::warn("MIP start {}: ignored", defect);
            else
                add_start_trees(start);
        }

        const auto unattractive = unattractive_columns<LP>(evict_age);
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

            auto gamma = [&](const std::size_t t) {
                return dual_solution[contribution_rows[targets[t].case_id]];
            };
            auto sigma = [&](const std::size_t t) {
                return dual_solution[uniqueness_rows[t]];
            };
            auto lambda = [&](const std::size_t t, const option_t i) {
                return dual_solution[purchase_rows[t].at(i)];
            };
            auto reduced_cost = [&](const tree & y, const double v) {
                const auto & [t, S] = y;
                double rc = gamma(t) * v - sigma(t);
                for(const option_t i : S) rc -= lambda(t, i);
                return rc;
            };
            auto margin = [&](const std::size_t t) {
                return pricing_margin(sigma(t));
            };

            // the best tree of target t, if its reduced cost is above margin
            auto price = [&](const std::size_t t)
                -> std::optional<std::pair<tree, double>> {
                pricing_model<I, MILP> & p = targets[t];
                std::vector<double> duals{gamma(t), sigma(t)};
                for(const option_t i : p.X.keys())
                    duals.push_back(lambda(t, i));
                if(duals == p.unproductive_duals) return std::nullopt;
                p.model.set_objective(gamma(t) * p.v -
                                      xsum(p.X, [&](const auto & e) {
                                          const auto & [i, X_i] = e;
                                          return lambda(t, i) * X_i;
                                      }));
                p.model.solve();
                if(p.model.get_solution_value() - sigma(t) <= margin(t)) {
                    p.unproductive_duals = std::move(duals);
                    return std::nullopt;
                }
                const auto solution = p.model.get_solution();
                return std::pair{
                    tree{t, std::ranges::to<std::vector>(std::views::keys(
                                std::views::filter(p.X,
                                                   [&](const auto & e) {
                                                       return _is_bought(
                                                           solution[e.second]);
                                                   })))},
                    evaluate(p.v, solution)};
            };

            std::atomic<bool> cut_short = false;  // by a stop
            tbb::concurrent_vector<std::pair<tree, double>> priced_trees;
            tbb::parallel_for(std::size_t{0}, targets.size(),
                              [&](const std::size_t t) {
                                  if(stop_requested || _time_up(sw)) {
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
                    return reduced_cost_of(entry) > margin(entry.seed.first);
                },
                add_tree);
            if(!num_activated && !cut_short) {
                report_progress({.end = true});
                spdlog::debug("Column generation stopped: converged");
                break;
            }
            if(stop_requested || _time_up(sw)) {
                report.status =
                    stop_requested ? "interrupted" : "time limit reached";
                // Gurobi, CPLEX and COPT give no solution for a master
                // changed since its last solve, HiGHS a zero value
                if(num_activated) {
                    model.solve();
                    report_progress({.relaxation = model.get_solution_value(),
                                     .round = ++iterations});
                }
                report_progress({.end = true});
                spdlog::warn("Column generation stopped: {}", report.status);
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
        for(const auto & i : instance.options())
            solution[i] = model_solution[x(i)];
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
