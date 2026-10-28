#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <flat_map>
#include <optional>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "mippp/utility/column_manager.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

namespace gecot {
namespace solvers {

// Path formulation relaxation: the contribution of each (source, target) pair
// is a column, a set of purchased options with the value
// q_s(S) * q_t(S) * P_st(S), the qualities with their gains and the most
// reliable path probability under S. The columns of a pair are computed once:
// option costs and duals are nonnegative, so a higher probability with a
// subset of the options is better whatever the duals. Each round, the pricing
// subproblem of a pair keeps the column of best reduced cost.
struct path_formulation_rr : public mip_solver_base {
    using variable_t = mippp::model_variable<int, double>;
    using constraint_t = mippp::model_constraint<int>;

    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict

    struct column {
        double value;
        std::vector<option_t> options;  // sorted
    };

    // the non-dominated (probability, options) labels of the paths from s,
    // within the budget, at every vertex
    template <instance_c I, case_c C>
    static auto _path_labels(const I & instance, const C & instance_case,
                             const auto s, const double budget) {
        struct label {
            double prob;
            std::vector<option_t> options;  // sorted
            double cost;
            bool alive = true;
        };
        const auto & graph = instance_case.graph();
        const auto & probability_map = instance_case.arc_probability_map();
        const auto & arc_options_map = instance_case.arc_options_map();
        auto labels_map = melon::create_vertex_map<std::vector<label>>(graph);
        std::priority_queue<
            std::tuple<double, std::remove_cv_t<decltype(s)>, std::size_t>>
            heap;
        auto relax = [&](const auto v, label && l) {
            auto & labels = labels_map[v];
            for(const auto & e : labels)
                if(e.alive && e.prob >= l.prob &&
                   std::ranges::includes(l.options, e.options))
                    return;
            for(auto & e : labels)
                if(e.alive && l.prob >= e.prob &&
                   std::ranges::includes(e.options, l.options))
                    e.alive = false;
            heap.emplace(l.prob, v, labels.size());
            labels.push_back(std::move(l));
        };
        relax(s, {1.0, {}, 0.0});
        while(!heap.empty()) {
            const auto [prob, u, i] = heap.top();
            heap.pop();
            if(!labels_map[u][i].alive) continue;
            const label l = labels_map[u][i];
            for(const auto & a : melon::out_arcs(graph, u)) {
                const auto v = melon::arc_target(graph, a);
                if(probability_map[a] > 0)
                    relax(v, {l.prob * probability_map[a], l.options, l.cost});
                for(const auto & [improved_prob, o] : arc_options_map[a]) {
                    if(std::ranges::binary_search(l.options, o)) {
                        relax(v, {l.prob * improved_prob, l.options, l.cost});
                        continue;
                    }
                    const double cost = l.cost + instance.option_cost(o);
                    if(cost > budget) continue;
                    auto options = l.options;
                    options.insert(std::ranges::upper_bound(options, o), o);
                    relax(v,
                          {l.prob * improved_prob, std::move(options), cost});
                }
            }
        }
        return labels_map;
    }

    // the non-dominated columns of the pair (s, t): its path labels with each
    // set of the quality options of s and t, within the budget
    template <instance_c I, case_c C>
    static std::vector<column> _pair_columns(const I & instance,
                                             const C & instance_case,
                                             const auto s, const auto t,
                                             const auto & labels,
                                             const double budget) {
        const auto & vertex_options_map = instance_case.vertex_options_map();
        std::vector<option_t> endpoint_options;
        for(const auto & [sqg, tqg, o] : vertex_options_map[s])
            if(sqg > 0) endpoint_options.push_back(o);
        for(const auto & [sqg, tqg, o] : vertex_options_map[t])
            if(tqg > 0) endpoint_options.push_back(o);
        std::ranges::sort(endpoint_options);
        endpoint_options.erase(std::ranges::unique(endpoint_options).begin(),
                               endpoint_options.end());
        if(endpoint_options.size() > 20)
            throw std::invalid_argument(
                "path_rr: too many quality options at a vertex");

        std::vector<column> columns;
        for(const auto & l : labels) {
            if(!l.alive) continue;
            for(std::size_t mask = 0;
                mask < (std::size_t{1} << endpoint_options.size()); ++mask) {
                auto options = l.options;
                for(std::size_t i = 0; i < endpoint_options.size(); ++i)
                    if(mask >> i & 1) options.push_back(endpoint_options[i]);
                std::ranges::sort(options);
                options.erase(std::ranges::unique(options).begin(),
                              options.end());
                double cost = 0, qs = instance_case.source_quality_map()[s],
                       qt = instance_case.target_quality_map()[t];
                for(const option_t o : options) cost += instance.option_cost(o);
                if(cost > budget) continue;
                for(const auto & [sqg, tqg, o] : vertex_options_map[s])
                    if(std::ranges::binary_search(options, o)) qs += sqg;
                for(const auto & [sqg, tqg, o] : vertex_options_map[t])
                    if(std::ranges::binary_search(options, o)) qt += tqg;
                column c{qs * qt * l.prob, std::move(options)};
                if(c.value <= 0 ||
                   std::ranges::any_of(columns, [&c](const column & e) {
                       return e.value >= c.value &&
                              std::ranges::includes(c.options, e.options);
                   }))
                    continue;
                std::erase_if(columns, [&c](const column & e) {
                    return c.value >= e.value &&
                           std::ranges::includes(e.options, c.options);
                });
                columns.push_back(std::move(c));
            }
        }
        return columns;
    }

    struct pair_data {
        case_id_t case_id;
        std::vector<column> columns;
        constraint_t uniqueness_constraint;
        std::flat_map<option_t, constraint_t> purchase_constraints;
    };

    struct column_seed {
        std::size_t pair;
        std::size_t column;
        bool operator==(const column_seed &) const = default;
    };
    struct column_seed_hash {
        std::size_t operator()(const column_seed & s) const noexcept {
            return s.pair * 0x9e3779b97f4a7c15ULL + s.column;
        }
    };

    template <typename LP, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        namespace colgen = mippp::colgen;
        spdlog::stopwatch sw;
        auto solution = instance.create_option_map(false);

        using namespace mippp;
        using namespace mippp::operators;
        LP model;
        if constexpr(mippp::has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();

        const auto X_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        // the pairs that options can change, the others summed per case
        std::vector<pair_data> pairs;
        std::vector<variable_t> contribution_vars;
        std::vector<constraint_t> contribution_constraints;
        for(auto && instance_case : instance.cases()) {
            const auto & graph = instance_case.graph();
            auto endpoint = [&](const auto v, const auto & quality_map,
                                const bool as_source) {
                return quality_map[v] > 0 ||
                       std::ranges::any_of(
                           instance_case.vertex_options_map()[v],
                           [as_source](const auto & e) {
                               const auto & [sqg, tqg, o] = e;
                               return (as_source ? sqg : tqg) > 0;
                           });
            };
            std::vector<melon::vertex_t<case_graph_t<decltype(instance_case)>>>
                sources, targets;
            for(const auto & v : melon::vertices(graph)) {
                if(endpoint(v, instance_case.source_quality_map(), true))
                    sources.push_back(v);
                if(endpoint(v, instance_case.target_quality_map(), false))
                    targets.push_back(v);
            }
            // per source, the constant pairs' values and the other pairs'
            // columns, kept in order so the model is the same from run to run
            std::vector<std::pair<double, std::vector<std::vector<column>>>>
                source_pairs(sources.size());
            tbb::parallel_for(std::size_t{0}, sources.size(), [&](auto i) {
                const auto labels_map =
                    _path_labels(instance, instance_case, sources[i], budget);
                for(const auto t : targets) {
                    auto columns =
                        _pair_columns(instance, instance_case, sources[i], t,
                                      labels_map[t], budget);
                    if(columns.size() == 1 && columns[0].options.empty())
                        source_pairs[i].first += columns[0].value;
                    else if(!columns.empty())
                        source_pairs[i].second.push_back(std::move(columns));
                }
            });
            double constant = 0;
            for(auto & [value, source_columns] : source_pairs)
                constant += value;
            for(auto & columns :
                std::views::join(std::views::values(source_pairs))) {
                pair_data & p = pairs.emplace_back(
                    instance_case.id(), std::move(columns),
                    model.add_constraint(
                        empty_linear_expression<variable_t, double> <= 1));
                for(const auto & c : p.columns)
                    for(const option_t o : c.options)
                        if(!p.purchase_constraints.contains(o))
                            p.purchase_constraints.emplace(
                                o, model.add_constraint(
                                       empty_linear_expression<variable_t,
                                                               double> <=
                                       X_vars(o)));
            }
            contribution_vars.push_back(model.add_variable());
            contribution_constraints.push_back(
                model.add_constraint(contribution_vars.back() <= constant));
        }

        model.set_objective(std::visit(
            formula_variable_visitor{
                model,
                [&](const case_id_t & i) { return contribution_vars[i]; }},
            instance.criterion()));

        auto log_lambda = [&, option_name_max_length = std::ranges::max(
                                  std::ranges::views::transform(
                                      instance.options(), [&](auto && o) {
                                          return instance.option_name(o).size();
                                      }))]() {
            const auto model_solution = model.get_solution();
            spdlog::info("path formulation relaxation has value: {}",
                         model.get_solution_value());
            for(auto && option : instance.options()) {
                const auto & option_name = instance.option_name(option);
                spdlog::info("    {:<{}} {}", option_name,
                             option_name_max_length,
                             model_solution[X_vars(option)]);
            }
        };

        colgen::column_manager<
            LP, column_seed, colgen::property_list<colgen::reduced_cost>,
            colgen::property_list<colgen::reduced_cost, colgen::age>,
            column_seed_hash>
            columns;
        auto add_column = [&](LP &, const column_seed & seed) {
            const pair_data & p = pairs[seed.pair];
            const column & c = p.columns[seed.column];
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_constraints[p.case_id], -c.value)),
                std::views::single(
                    std::make_pair(p.uniqueness_constraint, 1.0)),
                std::views::transform(c.options, [&p](const option_t o) {
                    return std::make_pair(p.purchase_constraints.at(o), 1.0);
                })));
        };
        // the master starts with the columns that buy nothing or, with
        // --mip-start, the best columns within the start's options
        std::optional<instance_solution_t<I>> start;
        if(mip_start &&
           compute_solution_cost(instance, _mip_start_solution(instance)) >
               budget + feasibility_tol)
            spdlog::warn("MIP start exceeds the budget: ignored");
        else if(mip_start)
            start = _mip_start_solution(instance);
        for(std::size_t i = 0; i < pairs.size(); ++i) {
            std::optional<std::size_t> best;
            for(std::size_t j = 0; j < pairs[i].columns.size(); ++j) {
                const column & c = pairs[i].columns[j];
                if(std::ranges::all_of(c.options,
                                       [&](const option_t o) {
                                           return start && (*start)[o];
                                       }) &&
                   (!best || c.value > pairs[i].columns[*best].value))
                    best = j;
            }
            if(best) {
                const column_seed seed{i, *best};
                columns.emplace_master_column(seed, add_column(model, seed));
            }
        }

        auto reduced_cost = [&](const column_seed & seed,
                                const auto & dual_solution) {
            const pair_data & p = pairs[seed.pair];
            const column & c = p.columns[seed.column];
            double rc =
                dual_solution[contribution_constraints[p.case_id]] * c.value -
                dual_solution[p.uniqueness_constraint];
            for(const option_t o : c.options)
                rc -= dual_solution[p.purchase_constraints.at(o)];
            return rc;
        };
        // in the master for more than evict_age rounds and unattractive, so
        // nonbasic
        const colgen::all<colgen::conjunction<
            colgen::above<colgen::age>, colgen::negative<colgen::reduced_cost>>>
            unattractive{colgen::above<colgen::age>{evict_age},
                         colgen::negative<colgen::reduced_cost>{1e-6}};
        if constexpr(!mippp::has_remove_variable<LP>)
            if(evict_age)
                spdlog::warn(
                    "This solver cannot remove columns: --evict-age ignored");

        solve_report report{"converged"};
        std::size_t iterations = 1;  // LP solves
        std::size_t num_evicted = 0;
        auto time_up = [&] {
            return time_limit && sw.elapsed().count() > time_limit.value();
        };
        for(;; ++iterations) {
            model.solve();
            const auto dual_solution = model.get_dual_solution();

            log_lambda();

            // the pricing test, at the master's dual tolerance
            auto improving_rc = [&](const column_seed & seed, const double rc) {
                return rc > 1e-7 * dual_solution[pairs[seed.pair]
                                                     .uniqueness_constraint] +
                                1e-6;
            };
            tbb::concurrent_vector<column_seed> priced_columns;
            tbb::parallel_for(std::size_t{0}, pairs.size(), [&](auto i) {
                std::optional<std::pair<double, column_seed>> best;
                for(std::size_t j = 0; j < pairs[i].columns.size(); ++j) {
                    const column_seed seed{i, j};
                    const double rc = reduced_cost(seed, dual_solution);
                    if(!best || rc > best->first) best.emplace(rc, seed);
                }
                if(best && improving_rc(best->second, best->first))
                    priced_columns.push_back(best->second);
            });
            columns.emplace_pool_columns(priced_columns);
            columns.update_columns([&](const column_seed & seed) {
                return colgen::priced{reduced_cost(seed, dual_solution)};
            });
            const colgen::all improving{[&](const auto & entry) {
                const auto & [seed, state] = entry;
                return improving_rc(seed,
                                    state.template get<colgen::reduced_cost>());
            }};
            const std::size_t num_activated =
                columns.manage_columns(model, improving, add_column)
                    .num_activated;
            if(!num_activated) {
                spdlog::info("Column generation stopped: converged");
                break;
            }
            if(time_up()) {
                spdlog::warn("Column generation stopped: time limit reached");
                report.status = "time limit reached";
                // as in tree_rr: the solvers lose the changed master's solution
                model.solve();
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(mippp::has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns
                                       .manage_columns(model, colgen::none{},
                                                       unattractive, add_column)
                                       .num_evicted;
        }

        const auto model_solution = model.get_solution();
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        log_lambda();

        std::size_t num_columns = 0;
        for(const pair_data & p : pairs) num_columns += p.columns.size();
        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pairs", pairs.size()},
                        {"columns", num_columns},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted},
                        {"relaxation_value", model.get_solution_value()}};
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
