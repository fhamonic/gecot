#pragma once

#include <algorithm>
#include <flat_map>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <spdlog/spdlog.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/utils/chronometer.hpp"

#include "gecot/solvers/greedy_decremental.hpp"
#include "gecot/solvers/greedy_incremental.hpp"

// template <typename Key, typename Value,
//           typename DefaultF = std::function<Value(Key)>,
//           typename Container = std::flat_map<Key, Value>>
//     requires std::invocable<DefaultF &, const Key &> &&
//              std::convertible_to<std::invoke_result_t<DefaultF &, const Key
//              &>,
//                                  Value>
// class lazy_defaulted_map {
// private:
//     DefaultF _default_lambda;
//     Container _map;

// public:
//     template <typename F>
//     lazy_defaulted_map(F && f) : _default_lambda(std::forward<F>(f)) {}

//     template <std::convertible_to<Key> K>
//     Value & operator[](K && key) {
//         auto it = _map.lower_bound(key);

//         if(it == _map.end() || _map.key_comp()(key, it->first))
//             return _map
//                 .emplace_hint(it, std::forward<K>(key), _default_lambda(key))
//                 ->second;

//         return it->second;
//     }

//     const Value & operator[](const Key & key) const { return _map.at(key); }
// };

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
        fhamonic::mippp::gurobi_milp sub_model;
        fhamonic::mippp::runtime_linear_expression<variable_t, double>
            default_objective;
        std::flat_map<option_t, variable_t> sub_X_vars_map;

        sub_model_data(const fhamonic::mippp::gurobi_api & api,
                       const auto & instance, const double & budget,
                       const auto & instance_case, const auto & strong_arcs_map,
                       const auto & useless_arcs_map,
                       const vertex_t & original_t)
            : target(original_t), sub_model(api), sub_X_vars_map() {
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

            const auto big_M_map = compute_knapsack_big_M_map(
                instance, budget, graph, source_quality_map, vertex_options_map,
                probability_map,
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

                for(const auto & b :
                    melon::out_arcs(graph, melon::arc_source(graph, a))) {
                    if(melon::arc_target(graph, a) ==
                       melon::arc_target(graph, b))
                        continue;
                    if(a == b) continue;
                    if(probability_map[b] >= probability_map[a]) continue;
                    sub_model.add_constraint(
                        Phi_vars(b) <=
                        (1 - sub_X_vars_map.at(arc_option_map[a].value())) *
                            big_M_map[melon::arc_source(graph, a)]);
                }
            }
        }

        bool try_generate_column(auto & master_data,
                                 const auto & dual_solution) {
            using namespace fhamonic::mippp;
            using namespace fhamonic::mippp::operators;
            sub_model.set_objective(
                default_objective -
                xsum(sub_X_vars_map.keys(), [&](option_t i) {
                    return dual_solution[master_data.purchase_constraints_map
                                             .at(std::make_pair(target, i))] *
                           sub_X_vars_map.at(i);
                }));
            sub_model.solve();
            if((1 + 1e-8) * sub_model.get_solution_value() >
               dual_solution[master_data.uniqueness_constraint_map.at(target)])
                return false;
            const auto solution = sub_model.get_solution();
            master_data.add_tree(
                target, evaluate(default_objective, solution),
                std::views::filter(sub_X_vars_map.keys(), [&](option_t i) {
                    return solution[sub_X_vars_map.at(i)] > 0.5;
                }));
            return true;
        }
    };

    template <typename I>
    struct master_model_case_data {
        using vertex_t = melon::vertex_t<instance_graph_t<I>>;
        std::reference_wrapper<fhamonic::mippp::gurobi_lp> master_model;
        variable_t contribution_variable;
        constraint_t contribution_constraint;
        std::flat_map<vertex_t, constraint_t> uniqueness_constraint_map;
        std::flat_map<std::pair<vertex_t, option_t>, constraint_t>
            purchase_constraints_map;
        std::vector<sub_model_data<I>> sub_models;

        master_model_case_data(fhamonic::mippp::gurobi_lp & model,
                               const auto & X_vars,
                               const fhamonic::mippp::gurobi_api & api,
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
                            empty_linear_expression<variable_t, double> <= 1));
                }));

            for(const vertex_t target : target_vertices) {
                sub_models.emplace_back(api, instance, budget, instance_case,
                                        strong_arcs, useless_arcs, target);
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
                      R && taken_options) {
            using namespace fhamonic::mippp::operators;
            master_model.get().add_column(std::views::concat(
                std::views::single(
                    std::make_pair(contribution_constraint, -contribution)),
                std::views::single(
                    std::make_pair(uniqueness_constraint_map.at(target), 1.0)),
                std::views::transform(taken_options, [this,
                                                      target](option_t i) {
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
        gurobi_api api;
        gurobi_lp model(api);
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
            master_model_cases.emplace_back(model, X_vars, api, instance,
                                            budget, instance_case,
                                            strong_arcs_map, useless_arcs_map);
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

        for(;;) {
            model.solve();
            auto dual_solution = model.get_dual_solution();

            if(std::ranges::none_of(
                   master_model_cases,
                   [&](master_model_case_data<I> & master_model_case) {
                       return std::ranges::any_of(
                           master_model_case.sub_models,
                           [&](sub_model_data<I> & sub_model) {
                               return sub_model.try_generate_column(
                                   master_model_case, dual_solution);
                           });
                   }))
                break;
        }

        spdlog::trace("tree_decomposition relaxation has value: {}",
                      model.get_solution_value());

        const auto model_solution = model.get_solution();
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic
