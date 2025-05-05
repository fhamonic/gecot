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

    auto _compute_dual_flow(
        const auto & instance, const auto & graph, const auto & quality_map,
        const auto & vertex_options_map, const auto & probability_map,
        const auto & arc_options_map, const auto & t, const auto & original_t,
        const auto & original_quality_map,
        const auto & original_vertex_options_map, const auto & solution) const {
        double opt = 0.0;
        auto rho_values = melon::create_vertex_map<double>(graph, 0.0);

        auto get_quality = [&](auto && u) {
            double quality = quality_map[u];
            for(auto && [quality_gain, option] : vertex_options_map[u]) {
                if(!solution[option]) continue;
                quality += quality_gain;
            }
            return quality;
        };
        double t_quality = original_quality_map[original_t];
        for(auto && [quality_gain, option] :
            original_vertex_options_map[original_t]) {
            if(!solution[option]) continue;
            t_quality += quality_gain;
        }

        const auto & reversed_graph = melon::views::reverse(graph);
        auto algo = melon::dijkstra(
            detail::pc_num_dijkstra_traits<decltype(reversed_graph), double>{},
            reversed_graph, [&](auto && a) {
                double prob = probability_map[a];
                for(auto && [improved_prob, option] : arc_options_map[a]) {
                    if(!solution[option]) continue;
                    prob = std::max(prob, improved_prob);
                }
                return prob;
            });

        algo.reset();
        algo.add_source(t);
        for(const auto & [u, prob] : algo) {
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
        //*
        using contracted_graph_t = melon::mutable_digraph;
        using contracted_data_t = std::tuple<
            contracted_graph_t, melon::vertex_map_t<contracted_graph_t, double>,
            melon::vertex_map_t<contracted_graph_t,
                                std::vector<std::pair<double, option_t>>>,
            melon::arc_map_t<contracted_graph_t, double>,
            melon::arc_map_t<contracted_graph_t,
                             std::vector<std::pair<double, option_t>>>,
            melon::vertex_map_t<contracted_graph_t, double>,
            melon::vertex_t<contracted_graph_t>,
            melon::vertex_t<instance_graph_t<I>>>;

        auto cases_contracted_data = instance.template create_case_map<
            std::vector<std::pair<gurobi_milp::variable, contracted_data_t>>>();

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

            for(const auto & original_t : melon::vertices(original_graph)) {
                if(original_quality_map[original_t] == 0 &&
                   original_vertex_options_map[original_t].empty()) {
                    continue;
                }
                cases_contracted_data[case_id].emplace_back(
                    model.add_variable(),
                    compute_contracted_graph(
                        instance_case, strong_arcs_map[original_t],
                        useless_arcs_map[original_t], original_t));
            }

            model.add_constraint(
                C_vars(case_id) <=
                xsum(std::views::keys(cases_contracted_data[case_id])));
        }

        // // add optimality cuts for empty solution
        for(auto && instance_case : instance.cases()) {
            for(auto && [var, contracted_data] :
                cases_contracted_data[instance_case.id()]) {
                auto [graph, quality_map, vertex_options_map, probability_map,
                      arc_options_map, big_M_map, t, original_t] =
                    contracted_data;
                auto [opt, rho_values] = _compute_dual_flow(
                    instance, graph, quality_map, vertex_options_map,
                    probability_map, arc_options_map, t, original_t,
                    instance_case.vertex_quality_map(),
                    instance_case.vertex_options_map(), solution);

                std::vector<melon::vertex_t<contracted_graph_t>> vertices;
                std::ranges::copy(graph.vertices(),
                                  std::back_inserter(vertices));
                std::vector<melon::arc_t<contracted_graph_t>> arcs;
                std::ranges::copy(graph.arcs(), std::back_inserter(arcs));

                model.add_constraint(
                    var <=
                    xsum(vertices, [&](auto && u) {
                        return (quality_map[u] +
                                xsum(vertex_options_map[u],
                                     [&](auto && p) {
                                         auto && [quality_gain, option] = p;
                                         return quality_gain * X_vars(option);
                                     })) *
                               rho_values[u];
                    }) + xsum(arcs, [&](auto && a) {
                        return xsum(arc_options_map[a], [&](auto && p) {
                            auto && [improved_prob, option] = p;
                            auto u = graph.arc_source(a);
                            auto v = graph.arc_target(a);
                            return big_M_map[u] * X_vars(option) *
                                   (improved_prob * rho_values[v] -
                                    rho_values[u]);
                        });
                    }) + xsum(vertex_options_map[t], [&](auto && p) {
                        auto && [quality_gain, option] = p;
                        return big_M_map[t] * X_vars(option) * quality_gain;
                    }));
            }
        }

        for(;;) {
            model.solve();
            const auto master_solution = model.get_solution();
            for(const auto & i : instance.options())
                solution[i] = (master_solution[X_vars(i)] > 0.5);
            ////////////////////////////////////////
            bool added_cut = false;
            for(auto && instance_case : instance.cases()) {
                spdlog::trace("C_{} = {}", instance_case.name(),
                              master_solution[C_vars(instance_case.id())]);
                spdlog::trace(
                    "------------------------------------------------------");
                spdlog::trace(
                    " feas. |  theta value  |      opt      |     delta    ");
                spdlog::trace(
                    "------------------------------------------------------");
                for(auto && [var, contracted_data] :
                    cases_contracted_data[instance_case.id()]) {
                    auto [graph, quality_map, vertex_options_map,
                          probability_map, arc_options_map, big_M_map, t,
                          original_t] = contracted_data;
                    auto [opt, rho_values] = _compute_dual_flow(
                        instance, graph, quality_map, vertex_options_map,
                        probability_map, arc_options_map, t, original_t,
                        instance_case.vertex_quality_map(),
                        instance_case.vertex_options_map(), solution);

                    const double violated_amount = master_solution[var] - opt;
                    const bool feasible =
                        (violated_amount <= feasibility_tol * opt);
                    spdlog::trace("   {}   | {:>13.2f} | {:>13.2f} | {:>12.2f}",
                                  static_cast<int>(feasible),
                                  master_solution[var], opt, violated_amount);

                    if(feasible) continue;

                    std::vector<melon::vertex_t<contracted_graph_t>> vertices;
                    std::ranges::copy(graph.vertices(),
                                      std::back_inserter(vertices));
                    std::vector<melon::arc_t<contracted_graph_t>> arcs;
                    std::ranges::copy(graph.arcs(), std::back_inserter(arcs));

                    auto cut =
                        xsum(vertices,
                             [&](auto && u) {
                                 return (quality_map[u] +
                                         xsum(vertex_options_map[u],
                                              [&](auto && p) {
                                                  auto && [quality_gain,
                                                           option] = p;
                                                  return quality_gain *
                                                         X_vars(option);
                                              })) *
                                        rho_values[u];
                             }) +
                        xsum(
                            arcs,
                            [&](auto && a) {
                                return xsum(arc_options_map[a], [&](auto && p) {
                                    auto && [improved_prob, option] = p;
                                    auto u = graph.arc_source(a);
                                    auto v = graph.arc_target(a);
                                    auto cai =
                                        solution[option]
                                            ? 0.0
                                            : std::max(0.0,
                                                       improved_prob *
                                                               rho_values[v] -
                                                           rho_values[u]);
                                    return big_M_map[u] * X_vars(option) * cai;
                                });
                            }) +
                        xsum(vertex_options_map[t], [&](auto && p) {
                            auto && [quality_gain, option] = p;
                            auto ri = solution[option] ? 0.0 : quality_gain;
                            return big_M_map[t] * X_vars(option) * ri;
                        });

                    model.add_constraint(var <= cut);
                    added_cut = true;
                }
            }
            if(added_cut) continue;
            ////////////////////////////////////////
            break;
        }
        return solution;
        /*/
        using contracted_graph_t = melon::mutable_digraph;
        using contracted_data_t = std::tuple<
            contracted_graph_t, melon::vertex_map_t<contracted_graph_t, double>,
            melon::vertex_map_t<contracted_graph_t,
                                std::vector<std::pair<double, option_t>>>,
            melon::arc_map_t<contracted_graph_t, double>,
            melon::arc_map_t<contracted_graph_t,
                             std::vector<std::pair<double, option_t>>>,
            melon::vertex_map_t<contracted_graph_t, double>,
            melon::vertex_t<contracted_graph_t>,
            melon::vertex_t<instance_graph_t<I>>>;

        auto cases_contracted_data =
            instance.template create_case_map<std::vector<contracted_data_t>>();

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

            for(const auto & original_t : melon::vertices(original_graph)) {
                if(original_quality_map[original_t] == 0 &&
                   original_vertex_options_map[original_t].empty()) {
                    continue;
                }
                cases_contracted_data[case_id].emplace_back(
                    compute_contracted_graph(
                        instance_case, strong_arcs_map[original_t],
                        useless_arcs_map[original_t], original_t));
            }
        }

        // add optimality cuts for empty solution
        for(auto && instance_case : instance.cases()) {
            mippp::runtime_linear_expression<gurobi_milp::variable, double> cut;
            for(auto && contracted_data :
                cases_contracted_data[instance_case.id()]) {
                auto [graph, quality_map, vertex_options_map, probability_map,
                      arc_options_map, big_M_map, t, original_t] =
                    contracted_data;
                auto [opt, rho_values] = _compute_dual_flow(
                    instance, graph, quality_map, vertex_options_map,
                    probability_map, arc_options_map, t, original_t,
                    instance_case.vertex_quality_map(),
                    instance_case.vertex_options_map(), solution);

                std::vector<melon::vertex_t<contracted_graph_t>> vertices;
                std::ranges::copy(graph.vertices(),
                                  std::back_inserter(vertices));
                std::vector<melon::arc_t<contracted_graph_t>> arcs;
                std::ranges::copy(graph.arcs(), std::back_inserter(arcs));

                cut += xsum(vertices,
                            [&](auto && u) {
                                return (quality_map[u] +
                                        xsum(vertex_options_map[u],
                                             [&](auto && p) {
                                                 auto && [quality_gain,
                                                          option] = p;
                                                 return quality_gain *
                                                        X_vars(option);
                                             })) *
                                       rho_values[u];
                            }) +
                       xsum(arcs,
                            [&](auto && a) {
                                return xsum(arc_options_map[a], [&](auto && p) {
                                    auto && [improved_prob, option] = p;
                                    auto u = graph.arc_source(a);
                                    auto v = graph.arc_target(a);
                                    return big_M_map[u] * X_vars(option) *
                                           (improved_prob * rho_values[v] -
                                            rho_values[u]);
                                });
                            }) +
                       xsum(vertex_options_map[t], [&](auto && p) {
                           auto && [quality_gain, option] = p;
                           return big_M_map[t] * X_vars(option) * quality_gain;
                       });
            }
            model.add_constraint(C_vars(instance_case.id()) <= std::move(cut));
        }

        for(;;) {
            model.solve();
            const auto master_solution = model.get_solution();
            for(const auto & i : instance.options())
                solution[i] = (master_solution[X_vars(i)] > 0.5);
            ////////////////////////////////////////
            bool added_cut = false;
            for(auto && instance_case : instance.cases()) {
                double cut_opt = 0.0;
                mippp::runtime_linear_expression<gurobi_milp::variable, double>
                    cut;
                for(auto && contracted_data :
                    cases_contracted_data[instance_case.id()]) {
                    auto [graph, quality_map, vertex_options_map,
                          probability_map, arc_options_map, big_M_map, t,
                          original_t] = contracted_data;
                    auto [opt, rho_values] = _compute_dual_flow(
                        instance, graph, quality_map, vertex_options_map,
                        probability_map, arc_options_map, t, original_t,
                        instance_case.vertex_quality_map(),
                        instance_case.vertex_options_map(), solution);

                    std::vector<melon::vertex_t<contracted_graph_t>> vertices;
                    std::ranges::copy(graph.vertices(),
                                      std::back_inserter(vertices));
                    std::vector<melon::arc_t<contracted_graph_t>> arcs;
                    std::ranges::copy(graph.arcs(), std::back_inserter(arcs));

                    cut_opt += opt;
                    cut +=
                        xsum(vertices,
                             [&](auto && u) {
                                 return (quality_map[u] +
                                         xsum(vertex_options_map[u],
                                              [&](auto && p) {
                                                  auto && [quality_gain,
                                                           option] = p;
                                                  return quality_gain *
                                                         X_vars(option);
                                              })) *
                                        rho_values[u];
                             }) +
                        xsum(
                            arcs,
                            [&](auto && a) {
                                return xsum(arc_options_map[a], [&](auto && p) {
                                    auto && [improved_prob, option] = p;
                                    auto u = graph.arc_source(a);
                                    auto v = graph.arc_target(a);
                                    auto cai =
                                        solution[option]
                                            ? 0.0
                                            : std::max(0.0,
                                                       improved_prob *
                                                               rho_values[v] -
                                                           rho_values[u]);
                                    return big_M_map[u] * X_vars(option) * cai;
                                });
                            }) +
                        xsum(vertex_options_map[t], [&](auto && p) {
                            auto && [quality_gain, option] = p;
                            auto ri = solution[option] ? 0.0 : quality_gain;
                            return big_M_map[t] * X_vars(option) * ri;
                        });
                }

                const double violated_amount =
                    master_solution[C_vars(instance_case.id())] - cut_opt;
                const bool feasible =
                    violated_amount < feasibility_tol * cut_opt;

                spdlog::trace(
                    "------------------------------------------------------");
                spdlog::trace("   {}   | {:>13.2f} | {:>13.2f} | {:>12.2f}",
                              static_cast<int>(feasible),
                              master_solution[C_vars(instance_case.id())],
                              cut_opt, violated_amount);
                spdlog::trace(
                    "------------------------------------------------------");

                if(feasible) continue;

                model.add_constraint(C_vars(instance_case.id()) <=
                                     std::move(cut));
                added_cut = true;
            }
            if(added_cut) continue;
            ////////////////////////////////////////
            break;
        }
        return solution;
        //*/
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_BENDERS_MIP_HPP
