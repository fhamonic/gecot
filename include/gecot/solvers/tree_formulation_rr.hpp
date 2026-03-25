#pragma once

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
    using constraint_t = fhamonic::mippp::model_constraint<int, double>;
    // using constraint_range_t = fhamonic::mippp::model_constraint<int,
    // double>;

    struct target_formulation_data {
        std::vector<option_t> options;
        constraint_t contribution_constraint;
        constraint_t uniqueness_constraint;
        constraint_range_t purchase_constraints;
        gurobi_milp sub_model;
        variable_t sub_Z_var;
        variable_range_t sub_Y_vars;

        template <std::ranges::range R>
        void add_tree(gurobi_lp & model, double Pi_T, R && options) {
            model.add_column(std::views::concat(
                std::views::single(
                    std::make_pair(contribution_constraint, -Pi_T)),
                std::views::single(std::make_pair(uniqueness_constraint, 1)),
                std::transform(options, [&purchase_constraints](option_t i) {
                    return std::make_pair(purchase_constraints(i), 1);
                })));
        }

        void generate_column(gurobi_lp & model, const auto & dual_solution,
                             const auto & Y_vars) {
            sub_model.set_objective(
                sub_Z_var - xsum(options, [&](option_t i) {
                    return dual_solution[purchase_constraints(i)] *
                           sub_Y_vars(i);
                }));
            sub_model.solve();
            if(sub_model.get_solution_value() >
               dual_solution[uniqueness_constraint])
                return;
            const auto solution = sub_model.get_solution();
            add_tree(model, solution[sub_Z_var],
                     std::views::filter(options, [&](option_t i) {
                         return solution[sub_Y_vars(i)] > 0. 5;
                     }));
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
        model.set_optimality_tolerance(1e-10);
        model.set_feasibility_tolerance(feasibility_tol);

        const auto C_vars = model.add_variables(
            instance.cases().size(), [](const case_id_t & id) { return id; });

        model.set_maximization();
        model.set_objective(std::visit(formula_variable_visitor{model, C_vars},
                                       instance.criterion()));

        const auto Y_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * Y_vars(o);
                             }) <= budget);

        auto truc = instance.template create_case_map<
            std::vector<target_formulation_data>>();

        for(auto && instance_case : instance.cases()) {
            const auto case_id = instance_case.id();
            const auto & original_graph = instance_case.graph();
            const auto & original_target_quality_map =
                instance_case.target_quality_map();
            const auto & original_vertex_options_map =
                instance_case.vertex_options_map();

            auto tree_variables = melon::create_vertices_map<
                std::vector<std::tuple<model_variable<int, double>, double,
                                       std::vector<option_t>>>>(original_graph);

            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);

            auto tree_contribution_constraint =
                model.add_constraint(C_vars(instance_case.id()) <= 0);

            for(const auto & original_t : melon::vertices(original_graph)) {
                if(original_target_quality_map[original_t] == 0 &&
                   instance_case.vertex_options_map()[original_t].empty())
                    continue;

                // auto one_tree_constraint =
                // model.add_constraint(xsum(std::views::elements<0>(
                //                          tree_variables[original_t])) <= 1);
                // auto bought_option_constraints =
                //     model.add_constraints(instance.options(), [&](option i) {
                //         return
                //         xsum(std::views::elements<0>(std::views::filter(
                //                    tree_variables[original_t], [i](auto && T)
                //                    {
                //                        return std::get<2>(T).contains(i);
                //                    }))) <= Y_vars(i);
                //     });

                auto one_tree_constraint = model.add_constraint(
                    empty_linear_expression<int, double> <= 1);
                auto bought_option_constraints = model.add_constraints(
                    instance.options(),
                    [&](option i) { return 0 <= Y_vars(i); });

                for(auto && [_, Pi_T, options] : tree_variables[original_t]) {
                    model.add_column(std::views::concat(
                        std::views::single(std::make_pair(
                            tree_contribution_constraint, -Pi_T)),
                        std::views::single(
                            std::make_pair(one_tree_constraint, 1)),
                        std::transform(options,
                                       [&bought_option_constraints](option i) {
                                           return std::make_pair(
                                               bought_option_constraints(i), 1);
                                       })));
                }
            }
            // auto tree_contribution_constraint = model.add_constraint(
            //     C_vars(instance_case.id()) <=
            //     xsum(melon::vertices(original_graph),
            //          [&tree_variables](const vertex_t & original_t) {
            //              return xsum(tree_variables[original_t], [](auto &&
            //              T) {
            //                  auto && [X_T, Pi_T, options] = T;
            //                  return Pi_T * X_T;
            //              });
            //          }));
        }

        spdlog::trace("prep_mip model has:");
        spdlog::trace("  {:>10} variables", model.num_variables());
        spdlog::trace("  {:>10} constraints", model.num_constraints());
        spdlog::trace("  {:>10} entries", model.num_entries());

        // {
        //     const auto greedy_solution = GreedyIncremental{}.solve(instance,
        //     budget); model.add_mip_start(
        //         std::views::transform(instance.options(), [&](auto o) {
        //             return std::make_pair(Y_vars(o), greedy_solution[o]);
        //         }));
        // }
        {
            const auto start_solution =
                GreedyDecremental{}.solve(instance, budget);
            model.add_mip_start(
                std::views::transform(instance.options(), [&](auto o) {
                    return std::make_pair(Y_vars(o), start_solution[o]);
                }));
        }

        model.solve();
        const auto model_solution = model.get_solution();

        spdlog::trace("prep_mip solution found with value: {}",
                      model.get_solution_value());
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[Y_vars(i)];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic
