#ifndef GECOT_SOLVERS_PREPROCESSED_MIP_HPP
#define GECOT_SOLVERS_PREPROCESSED_MIP_HPP

#include <type_traits>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include <spdlog/spdlog.h>

#include "mippp/mip_model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/utils/mip_helper.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct preprocessed_MIP {
    bool parallel = false;
    bool print_model = false;
    double probability_resolution;
    int nb_mus;

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
            model.get().add_constraint(
                var <= mippp::xsum(f.values, [this](const auto & e) {
                    return std::visit(*this, e);
                }));
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
            for(auto && e : f.values)
                model.get().add_constraint(var <= std::visit(*this, e));
            return var;
        }
    };

    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const double budget) const {
        if(nb_mus > 1) {
            return compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget, parallel,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution, nb_mus);
        } else {
            return compute_strong_and_useless_arcs(
                instance_case, parallel,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution);
        }
    }

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

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto & original_graph = instance_case.graph();
            const auto & original_quality_map =
                instance_case.vertex_quality_map();
            const auto & original_vertex_options_map =
                instance_case.vertex_options_map();

            const auto F_vars = model.add_variables(
                original_graph.nb_vertices(),
                [](const melon::vertex_t<instance_graph_t<I>> & v) {
                    return v;
                },
                [case_id](const melon::vertex_t<instance_graph_t<I>> & v) {
                    return "F_" + std::to_string(v) + "(" +
                           std::to_string(case_id) + ")";
                });
            std::vector<std::pair<variable<int, double>, double>>
                F_prime_additional_terms;

            spdlog::stopwatch prep_sw;
            spdlog::trace("Preprocessing the '{}' graph:",
                          instance_case.name());

            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            if(spdlog::get_level() == spdlog::level::trace) {
                int nb_strong, nb_useless, nb_sinks;
                nb_strong = nb_useless = nb_sinks = 0;
                for(auto && v : melon::vertices(original_graph)) {
                    if(original_quality_map[v] == 0 &&
                       instance_case.vertex_options_map()[v].empty())
                        continue;
                    nb_strong += strong_arcs_map[v].size();
                    nb_useless += useless_arcs_map[v].size();
                    ++nb_sinks;
                }
                spdlog::trace("  {:>8} strong arcs on average",
                              nb_strong / nb_sinks);
                spdlog::trace("  {:>8} useless arcs on average",
                              nb_useless / nb_sinks);
                spdlog::trace(
                    "           (took {} ms)",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        prep_sw.elapsed())
                        .count());
            }

            for(const auto & original_t : melon::vertices(original_graph)) {
                if(original_quality_map[original_t] == 0 &&
                   instance_case.vertex_options_map()[original_t].empty())
                    continue;

                const auto [graph, quality_map, vertex_options_map, arc_no_map,
                            probability_map, arc_option_map, t] =
                    compute_contracted_generalized_flow_graph(
                        instance_case, strong_arcs_map[original_t],
                        useless_arcs_map[original_t], original_t);
                using Graph = std::decay_t<decltype(graph)>;
                const auto big_M_map = compute_big_M_map(
                    graph, quality_map, vertex_options_map, probability_map,
                    std::views::filter(
                        melon::vertices(graph),
                        [&graph, &arc_option_map, t](auto && u) {
                            return u == t ||
                                   std::ranges::any_of(
                                       melon::out_arcs(graph, u),
                                       [&arc_option_map](auto && a) {
                                           return arc_option_map[a].has_value();
                                       });
                        }),
                    parallel);

                for(const auto & [quality_gain, option] :
                    original_vertex_options_map[original_t]) {
                    const auto F_prime_t_var =
                        model.add_variable("F'_" + std::to_string(original_t) +
                                           "_" + std::to_string(option) + "(" +
                                           std::to_string(case_id) + ")");
                    model.add_constraint(F_prime_t_var <= F_vars(original_t));
                    model.add_constraint(F_prime_t_var <=
                                         big_M_map[t].value() * X_vars(option));
                    F_prime_additional_terms.emplace_back(F_prime_t_var,
                                                          quality_gain);
                }
                const auto Phi_t_vars = model.add_variables(
                    graph.nb_arcs(),
                    [&arc_no_map](const melon::arc_t<Graph> & a) {
                        return arc_no_map[a];
                    },
                    [original_t, case_id](const melon::arc_t<Graph> & a) {
                        return "P_" + std::to_string(original_t) + "_" +
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
                    F_vars(original_t) + xsum(graph.out_arcs(t), Phi_t_vars) <=
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
                        X_vars(arc_option_map[a].value()) *
                            big_M_map[melon::arc_source(graph, a)].value());
                }
            }
            model.add_constraint(
                C_vars(instance_case.id()) <=
                xsum(melon::vertices(original_graph), F_vars,
                     original_quality_map) +
                    xsum(
                        F_prime_additional_terms,
                        [](const auto & p) { return p.first; },
                        [](const auto & p) { return p.second; }));
        }

        spdlog::trace("MIP model has:");
        spdlog::trace("  {:>10} variables", model.nb_variables());
        spdlog::trace("  {:>10} constraints", model.nb_constraints());
        spdlog::trace("  {:>10} entries", model.nb_entries());

        if(print_model) std::cout << model << std::endl;

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

#endif  // GECOT_SOLVERS_PREPROCESSED_MIP_HPP