#pragma once

#include <limits>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_generalized_flow_graph.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct mip {
    double feasibility_tol = 0.0;
    bool print_model = false;

    template <typename M, typename V>
    struct formula_variable_visitor {
        std::reference_wrapper<M> model;
        std::reference_wrapper<const V> C_vars;

        formula_variable_visitor(M & t_model, const V & t_C_vars)
            : model{t_model}, C_vars{t_C_vars} {}

        auto operator()(const criterion_constant & c) {
            return model.get().add_variable(
                {.lower_bound = c, .upper_bound = c});
        }
        auto operator()(const criterion_var & v) { return C_vars.get()(v); }
        auto operator()(const criterion_sum & f) {
            using namespace mippp::operators;
            auto var = model.get().add_variable();
            std::vector<typename M::variable> vars;
            for(auto && e : f.values) vars.emplace_back(std::visit(*this, e));
            model.get().add_constraint(var == xsum(vars));
            return var;
        }
        auto operator()(const criterion_product & f) {
            using namespace mippp::operators;
            if(!std::holds_alternative<criterion_constant>(f.values[0]) &&
               f.values.size() != 2)
                throw std::invalid_argument(
                    "mip doesn't support products of variables in the "
                    "criterion "
                    "!");

            auto var = model.get().add_variable();
            model.get().add_constraint(
                var == std::get<criterion_constant>(f.values[0]) *
                           std::visit(*this, f.values[1]));
            return var;
        }
        auto operator()(const criterion_min & f) {
            using namespace mippp::operators;
            auto var = model.get().add_variable();
            for(auto && e : f.values) {
                model.get().add_constraint(var <= std::visit(*this, e));
            }
            return var;
        }
    };

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        using namespace mippp;
        using namespace mippp::operators;
        gurobi_api api;
        gurobi_milp model(api);
        model.set_optimality_tolerance(1e-10);
        model.set_feasibility_tolerance(feasibility_tol);

        const auto C_vars = model.add_variables(
            instance.cases().size(), [](const case_id_t & id) { return id; }
            // ,[](const case_id_t & id) { return "C_" + std::to_string(id); }
        );

        model.set_maximization();
        model.set_objective(std::visit(formula_variable_visitor{model, C_vars},
                                       instance.criterion()));

        const auto X_vars = model.add_binary_variables(
            instance.options().size(), [](const option_t & i) { return i; }
            // ,[](const option_t & i) { return "Y_" + std::to_string(i); }
        );

        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto [graph, source_quality_map, target_quality_map,
                        vertex_options_map, probability_map, arc_option_map] =
                compute_generalized_flow_graph(instance_case);
            const auto big_M_map = compute_big_M_map(
                graph, source_quality_map,
                melon::views::map([&vertex_options_map](auto && u) {
                    return std::views::transform(
                        vertex_options_map[u], [](auto && e) {
                            auto && [source_quality_gain, tqg, option] = e;
                            return std::make_pair(source_quality_gain, option);
                        });
                }),
                probability_map, melon::vertices(graph));

            const auto F_vars = model.add_variables(
                graph.num_vertices(),
                [](const melon::vertex_t<instance_graph_t<I>> & v) { return v; }
                // , [case_id](const melon::vertex_t<instance_graph_t<I>> & v) {
                //     return "F_" + std::to_string(v) + "(" +
                //            std::to_string(case_id) + ")";
                // }
            );
            std::vector<std::pair<model_variable<int, double>, double>>
                F_prime_additional_terms;

            for(const auto & t : melon::vertices(graph)) {
                for(const auto & [sqm, target_quality_gain, option] :
                    vertex_options_map[t]) {
                    const auto F_prime_t_var =
                        model.add_variable(/*"F'_" + std::to_string(t) + "_" +
                                           std::to_string(option) + "(" +
                                           std::to_string(case_id) + ")"*/);
                    model.add_constraint(F_prime_t_var <= F_vars(t));
                    model.add_constraint(F_prime_t_var <=
                                         big_M_map[t] * X_vars(option));
                    F_prime_additional_terms.emplace_back(F_prime_t_var,
                                                          target_quality_gain);
                }
                const auto Phi_t_vars = model.add_variables(
                    graph.num_arcs(),
                    [](const melon::arc_t<instance_graph_t<I>> & a) {
                        return a;
                    }
                    // ,[t, case_id](const melon::arc_t<instance_graph_t<I>> &
                    // a) {
                    //     return "P_" + std::to_string(t) + "_" +
                    //            std::to_string(a) + "(" +
                    //            std::to_string(case_id) + ")";
                    // }
                );
                for(const auto & u : melon::vertices(graph)) {
                    if(u == t) continue;
                    model.add_constraint(
                        xsum(graph.out_arcs(u), Phi_t_vars) <=
                        xsum(graph.in_arcs(u),
                             [&](auto && a) {
                                 return probability_map[a] * Phi_t_vars(a);
                             }) +
                            source_quality_map[u] +
                            xsum(vertex_options_map[u], [&X_vars](auto && e) {
                                auto && [source_quality_gain, tqg, option] = e;
                                return source_quality_gain * X_vars(option);
                            }));
                }
                model.add_constraint(
                    F_vars(t) + xsum(graph.out_arcs(t), Phi_t_vars) <=
                    xsum(graph.in_arcs(t),
                         [&](auto && a) {
                             return probability_map[a] * Phi_t_vars(a);
                         }) +
                        source_quality_map[t] +
                        xsum(vertex_options_map[t], [&X_vars](auto && e) {
                            auto && [source_quality_gain, tqg, option] = e;
                            return source_quality_gain * X_vars(option);
                        }));

                for(const auto & a : melon::arcs(graph)) {
                    if(!arc_option_map[a].has_value()) continue;
                    model.add_constraint(
                        Phi_t_vars(a) <=
                        big_M_map[melon::arc_source(graph, a)] *
                            X_vars(arc_option_map[a].value()));
                }
            }
            model.add_constraint(
                C_vars(instance_case.id()) <=
                xsum(melon::vertices(graph), [&](auto && v) {
                    return target_quality_map[v] * F_vars(v);
                }) + xsum(F_prime_additional_terms, [](auto && p) {
                    return p.first * p.second;
                }));
        }

        spdlog::trace("mip model has:");
        spdlog::trace("  {:>10} variables", model.num_variables());
        spdlog::trace("  {:>10} constraints", model.num_constraints());
        spdlog::trace("  {:>10} entries", model.num_entries());

        model.solve();
        const auto model_solution = model.get_solution();

        spdlog::trace("mip solution found with value: {}",
                      model.get_solution_value());
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic
