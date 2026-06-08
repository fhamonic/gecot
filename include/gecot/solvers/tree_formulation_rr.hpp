#pragma once

#include <algorithm>
#include <flat_map>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <spdlog/spdlog.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/utils/chronometer.hpp"

#include "gecot/solvers/greedy_decremental.hpp"
#include "gecot/solvers/greedy_incremental.hpp"

// using MODEL_LP_API = fhamonic::mippp::clp_api;
// using MODEL_LP = fhamonic::mippp::clp_lp;
// using MODEL_MILP_API = fhamonic::mippp::cbc_api;
// using MODEL_MILP = fhamonic::mippp::cbc_milp;

using MODEL_LP_API = fhamonic::mippp::gurobi_api;
using MODEL_LP = fhamonic::mippp::gurobi_lp;
using MODEL_MILP_API = fhamonic::mippp::gurobi_api;
using MODEL_MILP = fhamonic::mippp::gurobi_milp;

namespace fhamonic {
namespace gecot {
namespace solvers {

struct tree_formulation_rr {
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
            using namespace fhamonic::mippp::operators;
            auto var = model.get().add_variable();
            std::vector<typename M::variable> vars;
            for(auto && e : f.values) vars.emplace_back(std::visit(*this, e));
            model.get().add_constraint(var == xsum(vars));
            return var;
        }
        auto operator()(const criterion_product & f) {
            using namespace fhamonic::mippp::operators;
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
            using namespace fhamonic::mippp::operators;
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

    using variable_t = fhamonic::mippp::model_variable<int, double>;
    using constraint_t = fhamonic::mippp::model_constraint<int>;

    template <instance_c I>
    struct sub_model_data {
        using vertex_t = melon::vertex_t<instance_graph_t<I>>;
        vertex_t target;
        MODEL_MILP sub_model;
        fhamonic::mippp::runtime_linear_expression<variable_t, double>
            default_objective;
        std::flat_map<option_t, variable_t> sub_X_vars_map;

        sub_model_data(const MODEL_MILP_API & milp_api, const auto & instance,
                       const double & budget, const auto & instance_case,
                       const auto & strong_arcs_map,
                       const auto & useless_arcs_map,
                       const vertex_t & original_t)
            : target(original_t), sub_model(milp_api), sub_X_vars_map() {
            using namespace fhamonic::mippp::operators;
            sub_model.set_optimality_tolerance(1e-10);
            sub_model.set_feasibility_tolerance(1e-8);
            sub_model.set_maximization();

            const auto & original_graph = instance_case.graph();
            const auto & original_target_quality_map =
                instance_case.target_quality_map();
            const auto & original_vertex_options_map =
                instance_case.vertex_options_map();

            const auto [graph, source_quality_map, vertex_options_map,
                        arc_no_map, probability_map, arc_option_map, t] =
                compute_contracted_generalized_flow_graph(
                    instance_case, strong_arcs_map[target],
                    useless_arcs_map[target], target);

            std::vector<option_t> options_vector;
            options_vector.append_range(std::views::join(std::views::transform(
                vertex_options_map, [](const auto & vertex_option) {
                    return std::views::values(vertex_option);
                })));
            options_vector.append_range(std::views::transform(
                std::views::filter(
                    arc_option_map,
                    [](auto const & o) { return o.has_value(); }),
                [](const auto & o) { return o.value(); }));
            std::ranges::sort(options_vector);
            const auto [first, last] = std::ranges::unique(
                options_vector.begin(), options_vector.end());
            options_vector.erase(first, last);
            sub_X_vars_map.insert_range(std::views::transform(
                options_vector, [this](const option_t & i) {
                    return std::make_pair(i, sub_model.add_binary_variable());
                }));

            const auto big_M_map = compute_big_M_map(
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

            const auto F_var = sub_model.add_variable();
            default_objective +=
                original_target_quality_map[original_t] * F_var;

            for(const auto & [sqg, target_quality_gain, option] :
                original_vertex_options_map[original_t]) {
                const auto F_prime_var = sub_model.add_variable();
                sub_model.add_constraint(F_prime_var <= F_var);
                sub_model.add_constraint(
                    F_prime_var <= big_M_map[t] * sub_X_vars_map.at(option));
                default_objective += target_quality_gain * F_prime_var;
            }

            const auto Phi_vars = sub_model.add_variables(
                graph.num_arcs(),
                [&arc_no_map](const melon::arc_t<decltype(graph)> & a) {
                    return arc_no_map[a];
                });
            sub_model.add_constraints(
                melon::vertices(graph),
                [&](const auto & u) {
                    return OPT(
                        (u == t),
                        F_var + xsum(graph.out_arcs(t), Phi_vars) <=
                            xsum(graph.in_arcs(t),
                                 [&](const auto & a) {
                                     return probability_map[a] * Phi_vars(a);
                                 }) +
                                source_quality_map[t] +
                                xsum(vertex_options_map[t],
                                     [this](const auto & p) {
                                         return p.first *
                                                sub_X_vars_map.at(p.second);
                                     }));
                },
                [&](const auto & u) {
                    return xsum(graph.out_arcs(u), Phi_vars) <=
                           xsum(graph.in_arcs(u),
                                [&](const auto & a) {
                                    return probability_map[a] * Phi_vars(a);
                                }) +
                               source_quality_map[u] +
                               xsum(vertex_options_map[u], [this](
                                                               const auto & p) {
                                   return p.first * sub_X_vars_map.at(p.second);
                               });
                });

            for(const auto & a : melon::arcs(graph)) {
                if(!arc_option_map[a].has_value()) continue;
                sub_model.add_constraint(
                    Phi_vars(a) <=
                    sub_X_vars_map.at(arc_option_map[a].value()) *
                        big_M_map[melon::arc_source(graph, a)]);
            }
        }

        void generate_initial_column(auto & master_data) {
            using namespace fhamonic::mippp;
            using namespace fhamonic::mippp::operators;
            sub_model.set_objective(default_objective);
            sub_model.solve();
            const auto solution = sub_model.get_solution();
            master_data.add_tree(target, sub_model.get_solution_value(),
                                 std::views::keys(std::views::filter(
                                     sub_X_vars_map, [&](const auto & e) {
                                         const auto & [option, X_var] = e;
                                         return solution[X_var] > 0.5;
                                     })));
        }

        bool try_generate_column(auto & master_data,
                                 const auto & dual_solution) {
            using namespace fhamonic::mippp;
            using namespace fhamonic::mippp::operators;
            sub_model.set_objective(
                /*dual_solution[contribution_constraint] **/ default_objective - xsum(sub_X_vars_map, [&](const auto & e) {
                    const auto & [option, X_var] = e;
                    return dual_solution[master_data.purchase_constraints_map
                                             .at(std::make_pair(target,
                                                                option))] *
                           X_var;
                }));
            sub_model.solve();
            spdlog::info(
                "{} : dual={} vs sub_model={}", target,
                dual_solution[master_data.uniqueness_constraint_map.at(target)],
                sub_model.get_solution_value());
            if((1 + 1e-7) *
                   dual_solution[master_data.uniqueness_constraint_map.at(
                       target)] >=
               sub_model.get_solution_value())
                return false;
            const auto solution = sub_model.get_solution();
            master_data.add_tree(target, evaluate(default_objective, solution),
                                 std::views::keys(std::views::filter(
                                     sub_X_vars_map, [&](const auto & e) {
                                         const auto & [option, X_var] = e;
                                         return solution[X_var] > 0.5;
                                     })));
            return true;
        }
    };

    template <instance_c I>
    struct master_model_case_data {
        using vertex_t = melon::vertex_t<instance_graph_t<I>>;
        std::reference_wrapper<MODEL_LP> master_model;
        variable_t contribution_variable;
        constraint_t contribution_constraint;
        std::flat_map<vertex_t, constraint_t> uniqueness_constraint_map;
        std::flat_map<std::pair<vertex_t, option_t>, constraint_t>
            purchase_constraints_map;
        std::vector<sub_model_data<I>> sub_models;

        master_model_case_data(MODEL_LP & model, const auto & X_vars,
                               const MODEL_MILP_API & milp_api,
                               const auto & instance, const double & budget,
                               const auto & instance_case,
                               const auto & strong_arcs,
                               const auto & useless_arcs)
            : master_model(model)
            , contribution_variable(model.add_variable())
            , contribution_constraint(
                  model.add_constraint(fhamonic::mippp::operators::operator<=(
                      contribution_variable, 0)))
            , uniqueness_constraint_map()
            , purchase_constraints_map() {
            using namespace fhamonic::mippp::operators;
            auto target_vertices = std::views::filter(
                melon::vertices(instance_case.graph()),
                [&instance_case](const vertex_t & t) {
                    return instance_case.target_quality_map()[t] > 0 ||
                           std::ranges::any_of(
                               instance_case.vertex_options_map()[t],
                               [](const auto & e) {
                                   const auto & [sqg, target_quality_gain,
                                                 option] = e;
                                   return target_quality_gain > 0;
                               });
                });
            uniqueness_constraint_map.insert_range(std::views::transform(
                target_vertices, [&model](const vertex_t & t) {
                    return std::make_pair(
                        t,
                        model.add_constraint(
                            empty_linear_expression<variable_t, double> <=
                            1));
                }));

            for(const vertex_t target : target_vertices) {
                sub_models.emplace_back(milp_api, instance, budget,
                                        instance_case, strong_arcs,
                                        useless_arcs, target);
            }

            purchase_constraints_map.insert_range(
                std::views::join(std::views::transform(
                    sub_models, [&model, &X_vars](const auto & sub_model) {
                        return std::views::transform(
                            sub_model.sub_X_vars_map.keys(),
                            [&model, &X_vars,
                             t = sub_model.target](const option_t & i) {
                                return std::make_pair(
                                    std::make_pair(t, i),
                                    model.add_constraint(
                                        empty_linear_expression<variable_t,
                                                                double> <=
                                        X_vars(i)));
                            });
                    })));
        }

        template <std::ranges::range R>
        void add_tree(const vertex_t & target, const double contribution,
                      R && used_options) {
            using namespace fhamonic::mippp::operators;

            spdlog::info("add_tree: {} , {}, {}", target, contribution,
                         std::format("{}", used_options));

            master_model.get().add_column(std::views::concat(
                std::views::single(
                    std::make_pair(contribution_constraint, -contribution)),
                std::views::single(
                    std::make_pair(uniqueness_constraint_map.at(target), 1.0)),
                std::views::transform(used_options, [this, target](option_t i) {
                    return std::make_pair(
                        purchase_constraints_map.at(std::make_pair(target, i)),
                        1.0);
                })));
        }
    };

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        using namespace fhamonic::mippp;
        using namespace fhamonic::mippp::operators;
        MODEL_LP_API lp_api;
        MODEL_LP model(lp_api);
        MODEL_MILP_API milp_api;
        model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();

        const auto X_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        std::vector<master_model_case_data<I>> master_model_cases;

        for(auto && instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            assert(master_model_cases.size() == instance_case.id());
            auto & master_model_case = master_model_cases.emplace_back(
                model, X_vars, milp_api, instance, budget, instance_case,
                strong_arcs_map, useless_arcs_map);

            for(auto & sub_model : master_model_case.sub_models) {
                sub_model.generate_initial_column(master_model_case);
            }
        }

        model.set_objective(std::visit(
            formula_variable_visitor{
                model,
                [&](const case_id_t & i) {
                    return master_model_cases[i].contribution_variable;
                }},
            instance.criterion()));

        spdlog::trace("tree decomposition model has:");
        spdlog::trace("  {:>10} variables", model.num_variables());
        spdlog::trace("  {:>10} constraints", model.num_constraints());
        spdlog::trace("  {:>10} entries", model.num_entries());

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

        for(bool improving = true; improving;) {
            model.solve();

            log_lambda();

            auto dual_solution = model.get_dual_solution();

            improving = false;
            for(auto & master_model_case : master_model_cases) {
                for(auto & sub_model : master_model_case.sub_models) {
                    improving |= sub_model.try_generate_column(
                        master_model_case, dual_solution);
                }
            }
        }

        const auto model_solution = model.get_solution();
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        log_lambda();

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic
