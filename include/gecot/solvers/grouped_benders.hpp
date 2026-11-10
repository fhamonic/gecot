#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <ranges>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

namespace gecot {
namespace solvers {

// Benders decomposition with a cut per group of targets.
//
// Buying the options X within the budget, c(X) <= B, a case k scores the sum
// over its targets t of
//     v_t(X) = q_t(X) Σ_s q_s(X) P_st(X),
// the target quality with its gains times the source qualities with their
// gains, each times the probability of its most reliable path to t, and the
// criterion φ combines the cases' scores. For any options Y, with
// ρ_u = q_t(Y) P_ut(Y) on the graph contracted for t,
//     v_t(X) <= cut_tY(x) = Σ_u (q_u + Σ_o Δq_uo x_o) ρ_u
//                         + Σ_{o∉Y} Σ_{a=(u,w)∈o} M_ao (p'_ao ρ_w - ρ_u)⁺ x_o
//                         + Σ_{o∉Y} M_to Δq_to x_o
// for every X within the budget, x its indicator, with equality at X = Y:
// M_ao bounds the flow of quality through a improved by o, M_to the flow that
// reaches t. The targets of each case are split into groups, and the master
//
//     max  φ(z)
//     s.t. z_k <= Σ_{g∈k} θ_g
//          θ_g <= Σ_{t∈g} cut_tY(x)    for each Y cut so far
//          Σ_o c_o x_o <= B,  x binary
//
// receives the cuts of a candidate Y for the groups whose θ_g exceeds
// Σ_{t∈g} v_t(Y). One group per case gives naive_benders' cuts, one group
// per target target_benders': fewer groups mean fewer, weaker cuts.
struct grouped_benders : public mip_solver_base {
    // groups per case, of consecutive targets; 0: a group per target
    std::size_t num_groups = 2;
    bool tight_big_M = false;

    // v_t(Y) and the ρ of cut_tY
    template <case_c C>
    auto _dual_flow(const contracted_graph_data<C> & data,
                    const auto & Y) const {
        const auto & instance_case = data.instance_case.get();
        double t_quality = instance_case.target_quality_map()[data.original_t];
        for(auto && [sqg, tqg, o] :
            instance_case.vertex_options_map()[data.original_t])
            if(Y[o]) t_quality += tqg;

        double flow = 0.0;
        auto rho = melon::create_vertex_map<double>(data.graph, 0.0);
        const auto & reversed_graph = melon::views::reverse(data.graph);
        for(const auto & [u, prob] : melon::dijkstra(
                detail::pc_num_dijkstra_traits<decltype(reversed_graph),
                                               double>{},
                reversed_graph,
                [&](auto && a) {
                    double p = data.probability_map[a];
                    for(auto && [improved_prob, o] : data.arc_options_map[a])
                        if(Y[o]) p = std::max(p, improved_prob);
                    return p;
                },
                data.t)) {
            double quality = data.source_quality_map[u];
            for(auto && [sqg, o] : data.source_options_map[u])
                if(Y[o]) quality += sqg;
            flow += quality * prob;
            rho[u] = t_quality * prob;
        }
        return std::make_pair(t_quality * flow, std::move(rho));
    }

    template <lazy_constraint_model M, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using target_data = contracted_graph_data<instance_case_t<I>>;
        spdlog::stopwatch sw;

        // The master: z with the criterion, x with the budget row, and for
        // each case its targets and the θ of its groups.
        M model;
        model.set_optimality_tolerance(1e-10);
        model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();
        const auto z = model.add_variables(
            instance.cases().size(), [](const case_id_t & k) { return k; });
        model.set_objective(std::visit(formula_variable_visitor{model, z},
                                       instance.criterion()));
        const auto x = model.add_binary_variables(
            instance.options().size(), [](const option_t & o) { return o; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * x(o);
                             }) <= budget);

        // θ_g and the targets [first, last) of its case that g holds
        struct group {
            model_variable_t<M> theta;
            std::size_t first, last;
        };
        auto targets =
            instance.template create_case_map<std::vector<target_data>>();
        auto groups = instance.template create_case_map<std::vector<group>>();
        for(auto && instance_case : instance.cases()) {
            const auto & graph = instance_case.graph();
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions and big-Ms of the '{}' graph",
                          instance_case.name());
            auto & case_targets = targets[instance_case.id()];
            {
                progress_bar<spdlog::level::info, 64> pb(graph.num_vertices());
                for(const auto & t : melon::vertices(graph)) {
                    pb.tick();
                    if(instance_case.target_quality_map()[t] == 0 &&
                       std::ranges::none_of(
                           instance_case.vertex_options_map()[t],
                           [](auto && e) {
                               auto && [sqg, tqg, o] = e;
                               return tqg > 0;
                           }))
                        continue;
                    case_targets.push_back(compute_contracted_graph_data(
                        instance, instance_case, budget, t, strong_arcs_map[t],
                        useless_arcs_map[t], tight_big_M, feasibility_tol));
                }
            }
            const std::size_t n = case_targets.size();
            const std::size_t G = num_groups ? std::min(num_groups, n) : n;
            auto & case_groups = groups[instance_case.id()];
            for(std::size_t g = 0; g < G; ++g)
                case_groups.push_back(
                    {model.add_variable(), g * n / G, (g + 1) * n / G});
            model.add_constraint(
                z(instance_case.id()) <=
                xsum(std::views::transform(case_groups, &group::theta)));
        }

        auto cut = [&](const target_data & data, const auto & Y,
                       const auto & rho) {
            return xsum(data.graph.vertices(),
                        [&](auto && u) {
                            return (data.source_quality_map[u] +
                                    xsum(data.source_options_map[u],
                                         [&](auto && e) {
                                             auto && [sqg, o] = e;
                                             return sqg * x(o);
                                         })) *
                                   rho[u];
                        }) +
                   xsum(
                       data.graph.arcs(),
                       [&](auto && a) {
                           return xsum(
                               std::views::iota(std::size_t{0},
                                                data.arc_options_map[a].size()),
                               [&, a](const std::size_t i) {
                                   auto && [improved_prob, o] =
                                       data.arc_options_map[a][i];
                                   const auto u = data.graph.arc_source(a);
                                   const auto w = data.graph.arc_target(a);
                                   const double gain =
                                       Y[o] ? 0.0
                                            : std::max(0.0,
                                                       improved_prob * rho[w] -
                                                           rho[u]);
                                   return data.improvement_big_M(a, i) * gain *
                                          x(o);
                               });
                       }) +
                   xsum(data.instance_case.get()
                            .vertex_options_map()[data.original_t],
                        [&](auto && e) {
                            auto && [sqg, tqg, o] = e;
                            return data.target_big_M(o) * (Y[o] ? 0.0 : tqg) *
                                   x(o);
                        });
        };
        // visit(k, g, v_g, cut) for each group g of each case k, with
        // v_g = Σ_{t∈g} v_t(Y) and its cut at Y. The cut is a lazy sum that
        // reads flows: it is only valid inside visit.
        auto for_each_group = [&](const auto & Y, auto && visit) {
            for(auto && instance_case : instance.cases()) {
                const case_id_t k = instance_case.id();
                const auto flows =
                    std::ranges::to<std::vector>(std::views::transform(
                        targets[k], [&](const target_data & data) {
                            return _dual_flow(data, Y);
                        }));
                for(const group & g : groups[k]) {
                    double value = 0;
                    for(std::size_t i = g.first; i < g.last; ++i)
                        value += flows[i].first;
                    visit(k, g, value,
                          g.theta <= xsum(std::views::iota(g.first, g.last),
                                          [&](const std::size_t i) {
                                              return cut(targets[k][i], Y,
                                                         flows[i].second);
                                          }));
                }
            }
        };
        auto seed_cuts = [&](const auto & Y) {
            for_each_group(
                Y, [&](case_id_t, const group &, double, auto && group_cut) {
                    model.add_constraint(
                        std::forward<decltype(group_cut)>(group_cut));
                });
        };
        seed_cuts(instance.create_option_map(false));
        if(mip_start) {
            const auto start = _mip_start_solution(instance);
            model.add_mip_start(std::views::transform(
                instance.options(),
                [&](auto o) { return std::make_pair(x(o), start[o]); }));
            seed_cuts(start);
        }

        // The solver drops a candidate that receives cuts, although its
        // options are a solution of score φ(v(Y)), and may never propose
        // them again: the best one is kept for the end.
        std::size_t num_cuts = 0;
        double best_score = -std::numeric_limits<double>::infinity();
        auto best_candidate = instance.create_option_map(false);
        model.set_candidate_solution_callback(
            [&](typename M::candidate_solution_callback_handle & handle) {
                const auto candidate = handle.get_solution();
                auto Y = melon::maps::function(
                    [&](const option_t o) { return candidate[x(o)] > 0.5; });
                auto case_scores = instance.create_case_map(0.0);
                for_each_group(Y, [&](const case_id_t k, const group & g,
                                      const double value, auto && group_cut) {
                    case_scores[k] += value;
                    if(candidate[g.theta] - value <= feasibility_tol * value)
                        return;
                    ++num_cuts;
                    handle.add_lazy_constraint(
                        std::forward<decltype(group_cut)>(group_cut));
                });
                if(const double score = instance.eval_criterion(case_scores);
                   score > best_score) {
                    best_score = score;
                    for(auto && o : instance.options())
                        best_candidate[o] = Y[o];
                }
            });

        auto [has_solution, report] = _solve(model, sw);
        report.stats.emplace_back("lazy_constraints", num_cuts);
        if(!has_solution) return {_fallback_solution(instance, budget), report};
        const auto master_solution = model.get_solution();
        auto solution = instance.create_option_map(false);
        for(auto && o : instance.options())
            solution[o] = master_solution[x(o)] > 0.5;
        auto case_scores = instance.create_case_map(0.0);
        for_each_group(solution,
                       [&](const case_id_t k, const group &, const double value,
                           auto &&) { case_scores[k] += value; });
        if(best_score > instance.eval_criterion(case_scores))
            solution = best_candidate;
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
