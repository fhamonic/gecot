#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/model_concepts.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/flow_rows.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

namespace gecot {
namespace solvers {

// The flow formulation of docs/formulations/flow.md. Each block of solve()
// builds the variables and the constraints of the section it names.
struct flow_mip : public mip_solver_base {
    // plain for comparison, tight experimental
    big_M_kind big_Ms = big_M_kind::knapsack_bnb;
    // experimental: the options continuous, for the root relaxation's bound
    bool relax_integrality = false;
    // experimental: no big-M is then computed, and big_Ms is not read
    bool indicator_constraints = false;

    // by how much the tight or exact big-Ms lower the knapsack ones
    struct lowered_big_M_statistics {
        double arc_sum = 0, arc_knapsack_sum = 0, target_sum = 0,
               target_knapsack_sum = 0;
        std::size_t num_arcs = 0, num_arcs_halved = 0, num_arcs_kept = 0;

        void add(const contracted_flow_graph & G_t, const auto & target_gains) {
            if(!G_t.lowered_big_Ms) return;
            const auto & knapsack_big_M_map = G_t.big_M_map.value();
            for(const auto & [source_gain, target_gain, i] : target_gains) {
                target_sum += G_t.target_big_M(i);
                target_knapsack_sum += knapsack_big_M_map[G_t.t];
            }
            for(auto && a : melon::arcs(G_t.graph)) {
                if(!G_t.improvement_option_map[a].has_value()) continue;
                const double knapsack_M =
                    knapsack_big_M_map[melon::arc_source(G_t.graph, a)];
                arc_sum += G_t.improvement_big_M(a);
                arc_knapsack_sum += knapsack_M;
                ++num_arcs;
                num_arcs_halved += G_t.improvement_big_M(a) <= knapsack_M / 2;
                num_arcs_kept += G_t.improvement_big_M(a) >= knapsack_M;
            }
        }
        void log(const std::string_view kind) const {
            spdlog::debug(
                "flow_mip {} big-Ms / knapsack's: arcs {:.4g}, targets {:.4g}",
                kind, arc_knapsack_sum > 0 ? arc_sum / arc_knapsack_sum : 1.0,
                target_knapsack_sum > 0 ? target_sum / target_knapsack_sum
                                        : 1.0);
            spdlog::debug("Arcs: {}, halved at least: {}, not lowered: {}",
                          num_arcs, num_arcs_halved, num_arcs_kept);
        }
    };

    template <typename M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using vertex = melon::vertex_t<instance_graph_t<I>>;
        _check_criterion(instance);
        if(indicator_constraints && !has_indicator_constraints<M>)
            throw std::invalid_argument(
                "this solver has no indicator constraints");
        if(indicator_constraints && relax_integrality)
            throw std::invalid_argument(
                "an indicator constraint needs a binary option: not with "
                "the relaxed ones");
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);

        // Cases and criterion: the scores z_k under their criterion
        // (Linearizing the criterion), the options x_i within the budgets
        // and the conflict sets
        const auto z = model.add_variables(
            instance.cases().size(), [](const case_id_t & k) { return k; });
        model.set_maximization();
        formula_variable_visitor formula{model, z};
        model.set_objective(formula.variable(instance.criterion()));
        const auto x = model.add_binary_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        _add_budget_rows(model, instance, budget, x);
        if(relax_integrality) {
            if constexpr(requires { model.set_continuous(x(0)); }) {
                for(auto && i : instance.options()) model.set_continuous(x(i));
            } else
                throw std::invalid_argument(
                    "this solver cannot relax the options");
        }

        const std::optional<big_M_kind> computed_big_Ms =
            indicator_constraints ? std::nullopt : std::optional(big_Ms);
        lowered_big_M_statistics lowered_statistics;
        // with the vertices and the arcs of the graphs G_t, on average
        std::vector<preprocessing_row<double, double>> preprocessing;
        for(auto && instance_case : instance.cases()) {
            const case_id_t k = instance_case.id();
            const auto & original_graph = instance_case.graph();
            const auto & target_quality_map =
                instance_case.target_quality_map();
            const auto & vertex_options_map =
                instance_case.vertex_options_map();

            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions{} of the '{}' graph",
                          computed_big_Ms ? " and big-Ms" : "",
                          instance_case.name());

            // What flow_mip solves: the value v_t of each vertex, 0 for one
            // that can never be a target
            const auto v =
                model.add_variables(original_graph.num_vertices(),
                                    [](const vertex & t) { return t; });
            graph_sizes sizes;
            for(const auto & original_t : melon::vertices(original_graph)) {
                if(!_is_target(instance_case, original_t)) {
                    model.set_variable_upper_bound(v(original_t), 0);
                    continue;
                }
                const contracted_flow_graph G_t = compute_contracted_flow_graph(
                    instance, instance_case, budget, original_t,
                    strong_arcs_map[original_t], useless_arcs_map[original_t],
                    computed_big_Ms, feasibility_tol, !no_pruning,
                    _options_model<M>(instance, budget));
                const auto & graph = G_t.graph;
                const auto & t = G_t.t;
                lowered_statistics.add(G_t, vertex_options_map[original_t]);
                sizes.add(graph);

                // A linear program for the score: the flow f_t that reaches
                // t. Target quality gains: for each gain f^i_t, equal to f_t
                // if i is bought and to 0 otherwise
                const auto f_t = model.add_variable();
                std::vector<std::pair<model_variable_t<M>, double>>
                    target_gains;
                for(const auto & [source_gain, target_gain, i] :
                    vertex_options_map[original_t]) {
                    const auto f_ti = model.add_variable();
                    model.add_constraint(f_ti <= f_t);
                    if(indicator_constraints) {
                        if constexpr(has_indicator_constraints<M>)
                            model.add_indicator_constraint(x(i), false,
                                                           f_ti <= 0);
                    } else
                        model.add_constraint(f_ti <=
                                             G_t.target_big_M(i) * x(i));
                    target_gains.emplace_back(f_ti, target_gain);
                }

                // A linear program for the score, Source quality gains: the
                // flow phi_a that enters each arc on its way to t, and its
                // conservation at each vertex
                const auto phi = model.add_variables(
                    graph.num_arcs(),
                    [&G_t](const contracted_flow_graph::arc & a) {
                        return G_t.arc_no_map[a];
                    });
                add_flow_rows(model, graph, t, f_t, phi, x, G_t.probability_map,
                              G_t.source_quality_map, G_t.source_options_map);

                // Improved arcs: an improved copy carries flow only if its
                // option is bought
                for(auto && a : melon::arcs(graph)) {
                    if(!G_t.improvement_option_map[a].has_value()) continue;
                    const option_t i = G_t.improvement_option_map[a].value();
                    if(indicator_constraints) {
                        if constexpr(has_indicator_constraints<M>)
                            model.add_indicator_constraint(x(i), false,
                                                           phi(a) <= 0);
                    } else
                        model.add_constraint(phi(a) <=
                                             x(i) * G_t.improvement_big_M(a));
                }

                // What flow_mip solves: the value of t
                model.add_constraint(
                    v(original_t) <=
                    target_quality_map[original_t] * f_t +
                        xsum(target_gains, [](const auto & gain) {
                            return gain.first * gain.second;
                        }));
            }
            // What flow_mip solves: the score of the case, at most the sum
            // of the v_t
            model.add_constraint(z(k) <= v);
            preprocessing.push_back(_sizes_row(instance_case, strong_arcs_map,
                                               useless_arcs_map, sizes));
        }
        _log_graph_sizes(preprocessing);
        if(computed_big_Ms == big_M_kind::tight)
            lowered_statistics.log("tight");
        if(computed_big_Ms == big_M_kind::exact)
            lowered_statistics.log("exact");
        _add_mip_start(model, instance, x);

        spdlog::info("Solving the optimization model");
        const auto [has_solution, report] = _solve(model, sw);
        if(!has_solution) return {_fallback_solution(instance, budget), report};
        if(!relax_integrality)
            return {_bought_options(instance, model.get_solution(), x), report};

        // Of the relaxation, only the options bought entirely: those bought
        // by half can cost more than the budget, and their score is then
        // reported above the bound of the relaxation.
        const auto relaxation = model.get_solution();
        auto solution = instance.create_option_map(false);
        for(const auto & i : instance.options())
            solution[i] = relaxation[x(i)] >= 1 - feasibility_tol;
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
