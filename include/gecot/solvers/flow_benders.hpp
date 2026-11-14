#pragma once

#include <cstddef>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

namespace gecot {
namespace solvers {

// The decomposition of docs/formulations/benders.md with a cut per target.
// Each block of solve() builds what the section it names states.
struct flow_benders : public mip_solver_base {
    // plain for comparison, tight experimental
    big_M_kind big_Ms = big_M_kind::knapsack_bnb;
    // experimental: on the target quality gains only, the cuts keep their
    // big-Ms
    bool indicator_constraints = false;

    template <lazy_constraint_model M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        _check_criterion(instance);
        if(indicator_constraints && !has_indicator_constraints<M>)
            throw std::invalid_argument(
                "this solver has no indicator constraints");
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);

        // Cases and criterion, in flow.md: the scores z_k under their
        // criterion (Linearizing the criterion), the options x_i within the
        // budgets and the conflict sets
        const auto z = model.add_variables(
            instance.cases().size(), [](const case_id_t & k) { return k; });
        model.set_maximization();
        formula_variable_visitor formula{model, z};
        model.set_objective(formula.variable(instance.criterion()));
        const auto x = model.add_binary_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        _add_budget_rows(model, instance, budget, x);

        struct target {
            model_variable_t<M> f_t;
            contracted_graph_data<instance_case_t<I>> G_t;
        };
        auto targets = instance.template create_case_map<std::vector<target>>();
        // the f^i_t: case by case, target by target, in the order of the
        // target's options
        std::vector<model_variable_t<M>> f_gains;
        // with the vertices and the arcs of the graphs G_t, on average
        std::vector<preprocessing_row<double, double>> preprocessing;
        for(auto && instance_case : instance.cases()) {
            const case_id_t k = instance_case.id();
            const auto & target_quality_map =
                instance_case.target_quality_map();
            const auto & vertex_options_map =
                instance_case.vertex_options_map();
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions and big-Ms of the '{}' graph",
                          instance_case.name());

            // flow_benders, the master: the score of the case, a row that
            // the columns of its f_t and f^i_t fill
            const auto z_row = model.add_constraint(z(k) <= 0);
            progress_bar<spdlog::level::info, 64> pb(
                instance_case.graph().num_vertices());
            for(const auto & original_t :
                melon::vertices(instance_case.graph())) {
                pb.tick();
                if(!_is_target(instance_case, original_t)) continue;
                // the flow f_t of the target, and an f^i_t, equal to f_t if
                // i is bought and to 0 otherwise, for each option of the
                // vertex, a target quality gain of 0 included: the callback
                // walks f_gains with this same loop
                const auto & [f_t, G_t] = targets[k].emplace_back(
                    model.add_column({std::make_pair(
                        z_row, -target_quality_map[original_t])}),
                    compute_contracted_graph_data(
                        instance, instance_case, budget, original_t,
                        strong_arcs_map[original_t],
                        useless_arcs_map[original_t], big_Ms, feasibility_tol,
                        !no_pruning));
                for(const auto & [source_gain, target_gain, i] :
                    vertex_options_map[original_t]) {
                    const auto f_ti = f_gains.emplace_back(model.add_column(
                        {std::make_pair(z_row, -target_gain)}));
                    model.add_constraint(f_ti <= f_t);
                    if(indicator_constraints) {
                        if constexpr(has_indicator_constraints<M>)
                            model.add_indicator_constraint(x(i), false,
                                                           f_ti <= 0);
                    } else
                        model.add_constraint(f_ti <=
                                             G_t.target_big_M(i) * x(i));
                }
            }
            std::size_t num_vertices = 0, num_arcs = 0;
            for(const auto & [f_t, G_t] : targets[k]) {
                num_vertices += G_t.graph.num_vertices();
                num_arcs += G_t.graph.num_arcs();
            }
            const auto num_graphs = static_cast<double>(
                std::max<std::size_t>(targets[k].size(), 1));
            preprocessing.emplace_back(
                instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map),
                static_cast<double>(num_vertices) / num_graphs,
                static_cast<double>(num_arcs) / num_graphs);
        }
        _log_preprocessing(
            preprocessing,
            log_column<10>{"vertices per target", log_format::fixed(2)},
            log_column<10>{"arcs per target", log_format::fixed(2)});

        // The cut of a target: cut_{t,y}(x), with the potentials rho of y
        auto cut = [&](const contracted_graph_data<instance_case_t<I>> & G_t,
                       const auto & y, const auto & rho) {
            return xsum(G_t.graph.vertices(),
                        [&](auto && u) {
                            return (G_t.source_quality_map[u] +
                                    xsum(G_t.source_options_map[u],
                                         [&](auto && gain) {
                                             auto && [source_gain, i] = gain;
                                             return source_gain * x(i);
                                         })) *
                                   rho[u];
                        }) +
                   xsum(G_t.graph.arcs(), [&](auto && a) {
                       return xsum(
                           std::views::iota(std::size_t{0},
                                            G_t.arc_options_map[a].size()),
                           [&, a](std::size_t n) {
                               auto && [improved_probability, i] =
                                   G_t.arc_options_map[a][n];
                               auto u = G_t.graph.arc_source(a);
                               auto v = G_t.graph.arc_target(a);
                               auto sigma =
                                   y[i] ? 0.0
                                        : std::max(0.0, improved_probability *
                                                                rho[v] -
                                                            rho[u]);
                               return G_t.improvement_big_M(a, n) * x(i) *
                                      sigma;
                           });
                   });
        };
        auto seed_cuts = [&](const auto & y) {
            for(auto && instance_case : instance.cases())
                for(const auto & [f_t, G_t] : targets[instance_case.id()]) {
                    const auto [flow, rho] = G_t.flow_and_potentials(y);
                    model.add_constraint(f_t <= cut(G_t, y, rho));
                }
        };
        // The search, step 1: the cuts of the scenario that buys nothing
        seed_cuts(instance.create_option_map(false));

        // The search, steps 2 to 4: the flows of a candidate y, the cut at y
        // of each target whose f_t exceeds its flow, and y handed back
        std::size_t num_cuts = 0;
        model.set_candidate_solution_callback(
            [&](typename M::candidate_solution_callback_handle & handle) {
                auto candidate = handle.get_solution();
                auto y = melon::maps::function([&](const option_t i) {
                    return _is_bought(candidate[x(i)]);
                });
                auto case_scores = instance.create_case_map(0.0);
                const std::size_t num_cuts_before = num_cuts;
                auto f_gain = f_gains.begin();
                for(auto && instance_case : instance.cases()) {
                    const case_id_t k = instance_case.id();
                    for(const auto & [f_t, G_t] : targets[k]) {
                        const auto [flow, rho] = G_t.flow_and_potentials(y);
                        double quality =
                            instance_case.target_quality_map()[G_t.original_t];
                        for(const auto & [source_gain, target_gain, i] :
                            instance_case
                                .vertex_options_map()[G_t.original_t]) {
                            candidate[*f_gain++] = y[i] ? flow : 0.0;
                            if(y[i]) quality += target_gain;
                        }
                        case_scores[k] += quality * flow;
                        if(std::exchange(candidate[f_t], flow) - flow <=
                           feasibility_tol * flow)
                            continue;
                        ++num_cuts;
                        handle.add_lazy_constraint(f_t <= cut(G_t, y, rho));
                    }
                }
                if(num_cuts > num_cuts_before)
                    _hand_back(handle, candidate, instance, x, y, z,
                               case_scores, formula);
            });

        // The search, step 1: the cuts of the MIP start
        if(const auto start = _add_mip_start(model, instance, x))
            seed_cuts(*start);

        auto [has_solution, report] = _solve(model, sw);
        report.stats.emplace_back("lazy_constraints", num_cuts);
        if(!has_solution) return {_fallback_solution(instance, budget), report};
        return {_bought_options(instance, model.get_solution(), x), report};
    }
};

}  // namespace solvers
}  // namespace gecot
