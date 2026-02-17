#pragma once

#include <limits>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

#include "gecot/solvers/benders_base.hpp"
#include "gecot/solvers/greedy_incremental.hpp"
#include "gecot/solvers/greedy_decremental.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct flow_benders : public benders_base {
    template <case_c C>
    auto _compute_dual_flow(const contracted_graph_data<C> & data,
                            const auto & solution) const {
        double opt = 0.0;
        auto rho_values = melon::create_vertex_map<double>(data.graph, 0.0);

        auto get_quality = [&](auto && u) {
            double quality = data.source_quality_map[u];
            for(auto && [source_quality_gain, option] :
                data.source_options_map[u]) {
                if(!solution[option]) continue;
                quality += source_quality_gain;
            }
            return quality;
        };

        const auto & reversed_graph = melon::views::reverse(data.graph);
        for(const auto & [u, prob] : melon::dijkstra(
                detail::pc_num_dijkstra_traits<decltype(reversed_graph),
                                               double>{},
                reversed_graph,
                [&](auto && a) {
                    double prob = data.probability_map[a];
                    for(auto && [improved_prob, option] :
                        data.arc_options_map[a]) {
                        if(!solution[option]) continue;
                        prob = std::max(prob, improved_prob);
                    }
                    return prob;
                },
                data.t)) {
            opt += get_quality(u) * prob;
            rho_values[u] = prob;
        }

        return std::make_tuple(opt, rho_values);
    }

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        using namespace mippp;
        using namespace mippp::operators;

        api_type api;
        model_type model(api);

        model.set_optimality_tolerance(1e-10);
        model.set_feasibility_tolerance(feasibility_tol);

        const auto C_vars = model.add_variables(
            instance.cases().size(), [](const case_id_t & id) { return id; });

        model.set_maximization();
        model.set_objective(std::visit(formula_variable_visitor{model, C_vars},
                                       instance.criterion()));

        const auto X_vars = model.add_binary_variables(
            instance.options().size(), [](const option_t & i) { return i; });

        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        auto cases_contracted_data =
            instance.template create_case_map<std::vector<
                std::pair<model_type::variable,
                          contracted_graph_data<instance_case_t<I>>>>>();

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto & original_graph = instance_case.graph();
            const auto & original_target_quality_map =
                instance_case.target_quality_map();
            const auto & original_vertex_options_map =
                instance_case.vertex_options_map();
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            auto C_constr = model.add_constraint(C_vars(case_id) <= 0);

            spdlog::stopwatch prep_sw;
            spdlog::trace(
                "Computing contractions and big-Ms of the '{}' graph:",
                instance_case.name());
            {
                progress_bar<spdlog::level::trace, 64> pb(
                    original_graph.num_vertices());
                for(const auto & original_t : melon::vertices(original_graph)) {
                    const double original_t_target_quality =
                        original_target_quality_map[original_t];
                    if(original_t_target_quality == 0 &&
                       original_vertex_options_map[original_t].empty()) {
                        pb.tick();
                        continue;
                    }
                    const auto & [F_var, data] =
                        cases_contracted_data[case_id].emplace_back(
                            model.add_column({std::make_pair(
                                C_constr, -original_t_target_quality)}),
                            compute_contracted_graph_data(
                                instance, instance_case, budget,
                                strong_arcs_map[original_t],
                                useless_arcs_map[original_t], original_t));

                    for(const auto & [sqg, target_quality_gain, option] :
                        original_vertex_options_map[original_t]) {
                        const auto F_prime_t_var = model.add_column(
                            {std::make_pair(C_constr, -target_quality_gain)});
                        model.add_constraint(F_prime_t_var <= F_var);
                        model.add_constraint(F_prime_t_var <=
                                             data.big_M_map[data.t] *
                                                 X_vars(option));
                        // model.add_indicator_constraint(X_vars(option), false,
                        //                                F_prime_t_var <= 0);
                    }
                    pb.tick();
                }
            }
            if(spdlog::get_level() == spdlog::level::trace) {
                std::size_t num_vertices, num_arcs, num_graphs;
                num_vertices = num_arcs = 0u;
                num_graphs = cases_contracted_data[instance_case.id()].size();
                for(const auto & [var, data] :
                    cases_contracted_data[instance_case.id()]) {
                    num_vertices += data.graph.num_vertices();
                    num_arcs += data.graph.num_arcs();
                }
                spdlog::trace("  {:>10.2f} vertices on average",
                              static_cast<double>(num_vertices) /
                                  static_cast<double>(num_graphs));
                spdlog::trace("  {:>10.2f} arcs on average",
                              static_cast<double>(num_arcs) /
                                  static_cast<double>(num_graphs));
                spdlog::trace(
                    "          (took {} ms)",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        prep_sw.elapsed())
                        .count());
            }
        }

        auto get_cut_expression =
            [&](const contracted_graph_data<instance_case_t<I>> & data,
                const auto & current_solution, const auto & rho_values) {
                return xsum(data.graph.vertices(),
                            [&](auto && u) {
                                return (data.source_quality_map[u] +
                                        xsum(data.source_options_map[u],
                                             [&](auto && p) {
                                                 auto && [source_quality_gain,
                                                          option] = p;
                                                 return source_quality_gain *
                                                        X_vars(option);
                                             })) *
                                       rho_values[u];
                            }) +
                       xsum(data.graph.arcs(), [&](auto && a) {
                           return xsum(data.arc_options_map[a], [&,
                                                                 a](auto && p) {
                               auto && [improved_prob, option] = p;
                               auto u = data.graph.arc_source(a);
                               auto v = data.graph.arc_target(a);
                               auto cai =
                                   current_solution[option]
                                       ? 0.0
                                       : std::max(0.0, improved_prob *
                                                               rho_values[v] -
                                                           rho_values[u]);
                               return data.big_M_map[u] * X_vars(option) * cai;
                           });
                       });
            };

        for(auto && instance_case : instance.cases()) {
            for(auto && [var, data] :
                cases_contracted_data[instance_case.id()]) {
                auto && [opt, rho_values] = _compute_dual_flow(data, solution);
                model.add_constraint(
                    var <= get_cut_expression(data, solution, rho_values));
            }
        }

        model.set_candidate_solution_callback(
            [&](model_type::candidate_solution_callback_handle & handle) {
                const auto master_solution = handle.get_solution();
                auto sol = melon::views::map([&](option_t i) {
                    return master_solution[X_vars(i)] > 0.5;
                });
                for(auto && instance_case : instance.cases()) {
                    for(const auto & [var, data] :
                        cases_contracted_data[instance_case.id()]) {
                        auto && [opt, rho_values] =
                            _compute_dual_flow(data, sol);
                        if(master_solution[var] - opt <= feasibility_tol * opt)
                            continue;
                        handle.add_lazy_constraint(
                            var <= get_cut_expression(data, sol, rho_values));
                    }
                }
            });

        {
            const auto start_solution = GreedyIncremental{}.solve(instance, budget);
            model.set_mip_start(
                std::views::transform(instance.options(), [&](auto o) {
                    return std::make_pair(X_vars(o), start_solution[o]);
                }));

            for(auto && instance_case : instance.cases()) {
                for(auto && [var, data] :
                    cases_contracted_data[instance_case.id()]) {
                    auto && [opt, rho_values] =
                        _compute_dual_flow(data, start_solution);
                    model.add_constraint(
                        var <=
                        get_cut_expression(data, start_solution, rho_values));
                }
            }
        }

        {
            const auto start_solution = GreedyDecremental{}.solve(instance, budget);
            model.set_mip_start(
                std::views::transform(instance.options(), [&](auto o) {
                    return std::make_pair(X_vars(o), start_solution[o]);
                }));

            for(auto && instance_case : instance.cases()) {
                for(auto && [var, data] :
                    cases_contracted_data[instance_case.id()]) {
                    auto && [opt, rho_values] =
                        _compute_dual_flow(data, start_solution);
                    model.add_constraint(
                        var <=
                        get_cut_expression(data, start_solution, rho_values));
                }
            }
        }
        model.solve();
        const auto master_solution = model.get_solution();
        for(const auto & i : instance.options())
            solution[i] = (master_solution[X_vars(i)] > 0.5);
        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic
