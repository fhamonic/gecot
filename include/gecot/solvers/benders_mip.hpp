#ifndef GECOT_SOLVERS_BENDERS_MIP_HPP
#define GECOT_SOLVERS_BENDERS_MIP_HPP

#include <limits>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "mippp/detail/std_ranges_to_range_v3.hpp"
#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct benders_MIP {
    double feasibility_tol = 0.0;
    bool print_model = false;
    double probability_resolution;
    int num_mus;

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
                    "MIP doesn't support products of variables in the "
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

    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const double budget) const {
        if(num_mus >= 1) {
            return compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution, num_mus);
        } else {
            return compute_strong_and_useless_arcs(
                instance_case,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution);
        }
    }

    template <case_c C>
    auto _compute_dual_flow(const contracted_graph_data<C> & data,
                            const auto & solution) const {
        double opt = 0.0;
        auto rho_values = melon::create_vertex_map<double>(data.graph, 0.0);

        auto get_quality = [&](auto && u) {
            double quality = data.quality_map[u];
            for(auto && [quality_gain, option] : data.vertex_options_map[u]) {
                if(!solution[option]) continue;
                quality += quality_gain;
            }
            return quality;
        };
        double t_quality =
            data.instance_case.get().vertex_quality_map()[data.original_t];
        for(auto && [quality_gain, option] :
            data.instance_case.get().vertex_options_map()[data.original_t]) {
            if(!solution[option]) continue;
            t_quality += quality_gain;
        }

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
            rho_values[u] = t_quality * prob;
        }
        opt *= t_quality;

        return std::make_tuple(opt, rho_values);
    }

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
                std::pair<gurobi_milp::variable,
                          contracted_graph_data<instance_case_t<I>>>>>();

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto & original_graph = instance_case.graph();
            const auto & original_quality_map =
                instance_case.vertex_quality_map();
            const auto & original_vertex_options_map =
                instance_case.vertex_options_map();
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            spdlog::stopwatch prep_sw;
            spdlog::trace("Computing contractions and big-Ms of the '{}' graph:",
                          instance_case.name());
            {
                progress_bar<spdlog::level::trace, 64> pb(
                    original_graph.num_vertices());
                for(const auto & original_t : melon::vertices(original_graph)) {
                    if(original_quality_map[original_t] == 0 &&
                       original_vertex_options_map[original_t].empty()) {
                        pb.tick();
                        continue;
                    }
                    cases_contracted_data[case_id].emplace_back(
                        model.add_variable(),
                        compute_contracted_graph_data(
                            instance_case, strong_arcs_map[original_t],
                            useless_arcs_map[original_t], original_t));
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
            model.add_constraint(
                C_vars(case_id) <=
                xsum(std::views::keys(cases_contracted_data[case_id])));
        }

        auto get_cut_expression =
            [&](const contracted_graph_data<instance_case_t<I>> & data,
                const auto & current_solution, const auto & rho_values) {
                return xsum(data.graph.vertices(),
                            [&](auto && u) {
                                return (data.quality_map[u] +
                                        xsum(data.vertex_options_map[u],
                                             [&](auto && p) {
                                                 auto && [quality_gain,
                                                          option] = p;
                                                 return quality_gain *
                                                        X_vars(option);
                                             })) *
                                       rho_values[u];
                            }) +
                       xsum(data.graph.arcs(),
                            [&](auto && a) {
                                return xsum(
                                    data.arc_options_map[a], [&, a](auto && p) {
                                        auto && [improved_prob, option] = p;
                                        auto u = data.graph.arc_source(a);
                                        auto v = data.graph.arc_target(a);
                                        auto cai =
                                            current_solution[option]
                                                ? 0.0
                                                : std::max(
                                                      0.0,
                                                      improved_prob *
                                                              rho_values[v] -
                                                          rho_values[u]);
                                        return data.big_M_map[u] *
                                               X_vars(option) * cai;
                                    });
                            }) +
                       xsum(data.vertex_options_map[data.t], [&](auto && p) {
                           auto && [quality_gain, option] = p;
                           auto ri =
                               current_solution[option] ? 0.0 : quality_gain;
                           return data.big_M_map[data.t] * X_vars(option) * ri;
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

        model.set_solution_callback([&](gurobi_milp::callback_handle & handle) {
            const auto master_solution = handle.get_solution();
            auto sol = melon::views::map(
                [&](option_t i) { return master_solution[X_vars(i)] > 0.5; });
            for(auto && instance_case : instance.cases()) {
                for(const auto & [var, data] :
                    cases_contracted_data[instance_case.id()]) {
                    auto && [opt, rho_values] = _compute_dual_flow(data, sol);
                    if(master_solution[var] - opt <= feasibility_tol * opt)
                        continue;
                    handle.add_lazy_constraint(
                        var <= get_cut_expression(data, sol, rho_values));
                }
            }
        });
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

#endif  // GECOT_SOLVERS_BENDERS_MIP_HPP
