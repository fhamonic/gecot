#ifndef GECOT_SOLVERS_MIP_HPP
#define GECOT_SOLVERS_MIP_HPP

#include <stdexcept>

#include <spdlog/spdlog.h>

#include "mippp/mip_model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_generalized_flow_graph.hpp"
#include "gecot/utils/mip_helper.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct MIP {
    bool parallel = false;
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
            auto var = model.get().add_variable();
            std::vector<typename M::var> vars;
            for(auto && e : f.values) vars.emplace_back(std::visit(*this, e));

            model.get().add_constraint(
                var <= mippp::linear_expression(
                           ranges::views::transform(
                               vars, [](auto && v) { return v.id(); }),
                           ranges::views::transform(
                               vars, [](auto && v) { return 1.0; }),
                           0.0));
            return var;
        }
        auto operator()(const criterion_product & f) {
            if(!std::holds_alternative<criterion_constant>(f.values[0]) &&
               f.values.size() != 2)
                throw std::invalid_argument(
                    "MIP doesn't support products of variables in the "
                    "criterion "
                    "!");

            auto var = model.get().add_variable();
            model.get().add_constraint(
                var <= std::get<criterion_constant>(f.values[0]) *
                           std::visit(*this, f.values[1]));
            return var;
        }
        auto operator()(const criterion_min & f) {
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
        using mip = mip_model<cli_solver_model_traits>;
        mip model;

        const auto C_vars = model.add_variables(
            instance.cases().size(), [](const case_id_t & id) { return id; },
            [](const case_id_t & id) { return "C_" + std::to_string(id); });

        model.add_to_objective(std::visit(
            formula_variable_visitor{model, C_vars}, instance.criterion()));

        const auto X_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; },
            [](const option_t & i) { return "Y_" + std::to_string(i); },
            {.upper_bound = 1, .type = mip::var_category::binary});

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](const auto & o) {
                return instance.option_cost(o);
            }) <= budget);

        int num_blocks = 0;

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto [graph, quality_map, vertex_options_map, probability_map,
                        arc_option_map] =
                compute_generalized_flow_graph(instance_case);
            const auto big_M_map = compute_big_M_map(
                graph, quality_map, vertex_options_map, probability_map,
                melon::vertices(graph), parallel);

            const auto F_vars = model.add_variables(
                graph.num_vertices(),
                [](const melon::vertex_t<instance_graph_t<I>> & v) {
                    return v;
                },
                [case_id](const melon::vertex_t<instance_graph_t<I>> & v) {
                    return "F_" + std::to_string(v) + "(" +
                           std::to_string(case_id) + ")";
                });
            std::vector<std::pair<variable<int, double>, double>>
                F_prime_additional_terms;

            for(const auto & t : melon::vertices(graph)) {
                for(const auto & [quality_gain, option] :
                    vertex_options_map[t]) {
                    const auto F_prime_t_var =
                        model.add_variable("F'_" + std::to_string(t) + "_" +
                                           std::to_string(option) + "(" +
                                           std::to_string(case_id) + ")");
                    model.add_constraint(F_prime_t_var <= F_vars(t));
                    model.add_constraint(F_prime_t_var <=
                                         big_M_map[t].value() * X_vars(option));
                    F_prime_additional_terms.emplace_back(F_prime_t_var,
                                                          quality_gain);
                }
                const auto Phi_t_vars = model.add_variables(
                    graph.num_arcs(),
                    [](const melon::arc_t<instance_graph_t<I>> & a) {
                        return a;
                    },
                    [t, case_id](const melon::arc_t<instance_graph_t<I>> & a) {
                        return "P_" + std::to_string(t) + "_" +
                               std::to_string(a) + "(" +
                               std::to_string(case_id) + ")";
                    });
                for(const auto & u : melon::vertices(graph)) {
                    if(u == t) continue;
                    model.add_constraint(
                        xsum(graph.out_arcs(u), Phi_t_vars) <=
                        xsum(graph.in_arcs(u), Phi_t_vars, probability_map) +
                            quality_map[u] +
                            xsum(
                                vertex_options_map[u],
                                [&X_vars](const auto & p) {
                                    return X_vars(p.second);
                                },
                                [](const auto & p) { return p.first; }));
                }
                model.add_constraint(
                    F_vars(t) + xsum(graph.out_arcs(t), Phi_t_vars) <=
                    xsum(graph.in_arcs(t), Phi_t_vars, probability_map) +
                        quality_map[t] +
                        xsum(
                            vertex_options_map[t],
                            [&X_vars](const auto & p) {
                                return X_vars(p.second);
                            },
                            [](const auto & p) { return p.first; }));

                for(const auto & a : melon::arcs(graph)) {
                    if(!arc_option_map[a].has_value()) continue;
                    model.add_constraint(
                        Phi_t_vars(a) <=
                        big_M_map[melon::arc_source(graph, a)].value() *
                            X_vars(arc_option_map[a].value()));
                }
            }
            model.add_constraint(
                C_vars(instance_case.id()) <=
                xsum(melon::vertices(graph), F_vars, quality_map) +
                    xsum(
                        F_prime_additional_terms,
                        [](const auto & p) { return p.first; },
                        [](const auto & p) { return p.second; }));
        }

        spdlog::trace("MIP model has:");
        spdlog::trace("  {:>10} variables", model.num_variables());
        spdlog::trace("  {:>10} constraints", model.num_constraints());
        spdlog::trace("  {:>10} entries", model.num_entries());

        auto solver = mip_helper::build_solver(model);
        solver->set_loglevel(spdlog::get_level() == spdlog::level::trace ? 1
                                                                         : 0);
        solver->set_timeout(3600);
        solver->set_mip_gap(1e-10);
        auto ret_code = solver->optimize();
        if(ret_code != 0)
            throw std::runtime_error(solver->name() + " failed with code " +
                                     std::to_string(ret_code) +
                                     ", see logs at " +
                                     solver->logs_path().string());
        const auto solver_solution = solver->get_solution();

        spdlog::trace("MIP solution found with value: {}",
                      solver->get_objective_value());
        for(const auto & i : instance.options()) {
            solution[i] =
                solver_solution[static_cast<std::size_t>(X_vars(i).id())];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_MIP_HPP
