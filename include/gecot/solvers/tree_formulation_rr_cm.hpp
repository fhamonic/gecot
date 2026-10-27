#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include "mippp/utility/column_manager.hpp"

#include "gecot/solvers/tree_formulation_rr.hpp"

namespace gecot {
namespace solvers {

// tree_rr with the master's trees kept by mippp's column manager: the priced
// trees enter a pool, the improving ones the master and, with evict_age, the
// unattractive ones go back to the pool
struct tree_formulation_rr_cm : public tree_formulation_rr {
    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict

    template <instance_c I>
    struct tree_seed {
        case_id_t case_id;
        melon::vertex_t<instance_graph_t<I>> target;
        std::vector<option_t> options;  // sorted, each with a purchase row
        double contribution;            // not part of the key

        bool operator==(const tree_seed & other) const {
            return case_id == other.case_id && target == other.target &&
                   options == other.options;
        }
    };
    struct tree_seed_hash {
        std::size_t operator()(const auto & seed) const noexcept {
            std::size_t h = seed.case_id;
            auto combine = [&h](std::size_t v) {
                h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            };
            combine(seed.target);
            for(const option_t o : seed.options) combine(o);
            return h;
        }
    };

    // try_generate_column, leaving the tree to the column manager
    template <instance_c I>
    static std::optional<tree_seed<I>> _price(const case_id_t id,
                                              auto & sub_model_data,
                                              const auto & master_data,
                                              const auto & dual_solution) {
        using namespace mippp;
        using namespace mippp::operators;
        auto duals = sub_model_data.pricing_duals(master_data, dual_solution);
        if(duals == sub_model_data.unproductive_duals) return std::nullopt;
        const auto t = sub_model_data.target;
        auto & sub_model = sub_model_data.sub_model;
        sub_model.set_objective(
            dual_solution[master_data.contribution_constraint] *
                sub_model_data.default_objective -
            xsum(sub_model_data.sub_X_vars_map, [&](const auto & e) {
                const auto & [option, X_var] = e;
                return dual_solution[master_data.purchase_constraints_map.at(
                           std::make_pair(t, option))] *
                       X_var;
            }));
        sub_model.solve();
        if((1 + 1e-7) *
                   dual_solution[master_data.uniqueness_constraint_map.at(t)] +
               1e-6 >=
           sub_model.get_solution_value()) {
            sub_model_data.unproductive_duals = std::move(duals);
            return std::nullopt;
        }
        const auto solution = sub_model.get_solution();
        return tree_seed<I>{
            id, t,
            std::ranges::to<std::vector>(std::views::keys(std::views::filter(
                sub_model_data.sub_X_vars_map,
                [&](const auto & e) { return solution[e.second] > 0.5; }))),
            evaluate(sub_model_data.default_objective, solution)};
    }

    // positive when the tree improves the master
    static double _reduced_cost(const auto & seed,
                                const auto & master_model_cases,
                                const auto & dual_solution) {
        const auto & master_data = master_model_cases[seed.case_id];
        double rc = dual_solution[master_data.contribution_constraint] *
                        seed.contribution -
                    dual_solution[master_data.uniqueness_constraint_map.at(
                        seed.target)];
        for(const option_t o : seed.options)
            rc -= dual_solution[master_data.purchase_constraints_map.at(
                std::make_pair(seed.target, o))];
        return rc;
    }

    template <typename LP, typename MILP, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        namespace colgen = mippp::colgen;
        spdlog::stopwatch sw;
        auto solution = instance.create_option_map(false);

        using namespace mippp;
        using namespace mippp::operators;
        std::mutex master_model_mutex;
        LP model;
        if constexpr(mippp::has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();

        const auto X_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        std::vector<master_model_case_data<I, LP, MILP>> master_model_cases;

        for(auto && instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            assert(master_model_cases.size() == instance_case.id());
            master_model_cases.emplace_back(master_model_mutex, model, X_vars,
                                            instance, budget, instance_case,
                                            strong_arcs_map, useless_arcs_map);
        }

        model.set_objective(std::visit(
            formula_variable_visitor{
                model,
                [&](const case_id_t & i) {
                    return master_model_cases[i].contribution_variable;
                }},
            instance.criterion()));

        auto log_lambda = [&, option_name_max_length = std::ranges::max(
                                  std::ranges::views::transform(
                                      instance.options(), [&](auto && o) {
                                          return instance.option_name(o).size();
                                      }))]() {
            const auto model_solution = model.get_solution();
            spdlog::info("tree_decomposition relaxation has value: {}",
                         model.get_solution_value());
            for(auto && option : instance.options()) {
                const auto & option_name = instance.option_name(option);
                spdlog::info("    {:<{}} {}", option_name,
                             option_name_max_length,
                             model_solution[X_vars(option)]);
            }
        };

        using seed_t = tree_seed<I>;
        colgen::column_manager<
            LP, seed_t, colgen::property_list<colgen::reduced_cost>,
            colgen::property_list<colgen::reduced_cost, colgen::age>,
            tree_seed_hash>
            columns;
        auto add_tree = [&](LP &, const seed_t & seed) {
            return master_model_cases[seed.case_id].add_tree(
                seed.target, seed.contribution, seed.options);
        };
        _add_mip_start_trees(
            instance, budget, master_model_cases,
            [&](const case_id_t id, const auto & t, const double contribution,
                const auto & options) {
                seed_t seed{id, t,
                            std::ranges::to<std::vector>(std::views::filter(
                                options,
                                [&](const option_t o) {
                                    return master_model_cases[id]
                                        .purchase_constraints_map.contains(
                                            std::make_pair(t, o));
                                })),
                            contribution};
                std::ranges::sort(seed.options);
                const auto var = add_tree(model, seed);
                columns.emplace_master_column(std::move(seed), var);
            });
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

            // a round cut short by the time limit may have priced no column
            std::atomic<bool> cut_short = false;
            tbb::concurrent_vector<seed_t> priced_trees;
            tbb::parallel_for(
                std::size_t{0}, master_model_cases.size(),
                [&](const case_id_t id) {
                    auto & master_data = master_model_cases[id];
                    tbb::parallel_for_each(
                        master_data.sub_models, [&](auto & sub_model) {
                            if(time_up()) {
                                cut_short = true;
                                return;
                            }
                            if(auto tree = _price<I>(id, sub_model, master_data,
                                                     dual_solution))
                                priced_trees.push_back(std::move(*tree));
                        });
                });
            columns.emplace_pool_columns(priced_trees | std::views::as_rvalue);
            columns.update_columns([&](const seed_t & seed) {
                return colgen::priced{
                    _reduced_cost(seed, master_model_cases, dual_solution)};
            });
            // the pricing test, at the master's dual tolerance
            const colgen::all improving{[&](const auto & entry) {
                const auto & [seed, state] = entry;
                return state.template get<colgen::reduced_cost>() >
                       1e-7 * dual_solution[master_model_cases[seed.case_id]
                                                .uniqueness_constraint_map.at(
                                                    seed.target)] +
                           1e-6;
            }};
            const std::size_t num_activated =
                columns.manage_columns(model, improving, add_tree)
                    .num_activated;
            if(!num_activated && !cut_short) {
                spdlog::info("Column generation stopped: converged");
                break;
            }
            if(time_up()) {
                spdlog::warn("Column generation stopped: time limit reached");
                report.status = "time limit reached";
                // as in tree_rr: the solvers lose the changed master's solution
                if(num_activated) model.solve();
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(mippp::has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns
                                       .manage_columns(model, colgen::none{},
                                                       unattractive, add_tree)
                                       .num_evicted;
        }

        const auto model_solution = model.get_solution();
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        log_lambda();

        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted},
                        // not a bound: the pricing can miss improving trees
                        {"relaxation_value", model.get_solution_value()}};
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
