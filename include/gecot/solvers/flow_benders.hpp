#pragma once

#include <cstddef>
#include <ranges>
#include <stdexcept>
#include <tuple>
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

#include "gecot/solvers/benders_base.hpp"

namespace gecot {
namespace solvers {

// The decomposition of docs/formulations/benders.md with a cut per target.
// Each block of solve() builds what the section it names states.
struct flow_benders : public benders_base {
    // experimental: on the target quality gains only, the cuts keep their
    // big-Ms
    bool indicator_constraints = false;

    template <mippp::has_column_generation M, instance_c I>
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
            // (q^i_t, i, f^i_t) for each option i of the vertex
            std::vector<std::tuple<double, option_t, model_variable_t<M>>>
                f_gains;
        };
        auto targets = instance.template create_case_map<std::vector<target>>();
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
                // vertex, a target quality gain of 0 included
                auto & [f_t, G_t, f_gains] = targets[k].emplace_back(
                    model.add_column({std::make_pair(
                        z_row, -target_quality_map[original_t])}),
                    compute_contracted_graph_data(
                        instance, instance_case, budget, original_t,
                        strong_arcs_map[original_t],
                        useless_arcs_map[original_t], big_Ms, feasibility_tol,
                        pruning, _options_model<M>(instance, budget)));
                for(const auto & [source_gain, target_gain, i] :
                    vertex_options_map[original_t]) {
                    const auto f_ti =
                        model.add_column({std::make_pair(z_row, -target_gain)});
                    f_gains.emplace_back(target_gain, i, f_ti);
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
            graph_sizes sizes;
            for(const auto & [f_t, G_t, f_gains] : targets[k])
                sizes.add(G_t.graph);
            preprocessing.push_back(_sizes_row(instance_case, strong_arcs_map,
                                               useless_arcs_map, sizes));
        }
        _log_graph_sizes(preprocessing);

        // The cut of a target: f_t <= cut_{t,y}(x), with the potentials rho of
        // y
        auto seed_cuts = [&](const auto & y) {
            for(auto && instance_case : instance.cases())
                for(const auto & [f_t, G_t, f_gains] :
                    targets[instance_case.id()]) {
                    const auto [flow, rho] = G_t.flow_and_potentials(y);
                    model.add_constraint(f_t <= G_t.cut(x, y, rho));
                }
        };
        // The search, step 1: the cuts of the scenario that buys nothing
        seed_cuts(instance.create_option_map(false));

        // The search, steps 2 to 4: the flows of a candidate y and the cut at
        // y of each target whose f_t exceeds its flow
        auto cuts = [&](auto & candidate, const auto & y, auto && add) {
            auto case_scores = instance.create_case_map(0.0);
            for(auto && instance_case : instance.cases()) {
                const case_id_t k = instance_case.id();
                for(const auto & [f_t, G_t, f_gains] : targets[k]) {
                    const auto [flow, rho] = G_t.flow_and_potentials(y);
                    double quality =
                        instance_case.target_quality_map()[G_t.original_t];
                    for(const auto & [target_gain, i, f_ti] : f_gains) {
                        candidate[f_ti] = y[i] ? flow : 0.0;
                        if(y[i]) quality += target_gain;
                    }
                    case_scores[k] += quality * flow;
                    if(std::exchange(candidate[f_t], flow) - flow <=
                       feasibility_tol * flow)
                        continue;
                    add(f_t <= G_t.cut(x, y, rho));
                }
            }
            return case_scores;
        };

        // The search, step 1: the cuts of the MIP start
        if(const auto start = _add_mip_start(model, instance, x))
            seed_cuts(*start);

        spdlog::info("Solving the optimization model");
        auto [solution, report] =
            _benders_search(model, sw, instance, budget, x, z, formula, cuts);
        if(!solution) return {_fallback_solution(instance, budget), report};
        return {std::move(*solution), report};
    }
};

}  // namespace solvers
}  // namespace gecot
