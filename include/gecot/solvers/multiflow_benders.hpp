#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/parallel_for.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// Benders decomposition with a flow per source and target.
//
// Buying the options X within the budget, c(X) <= B, the quality that
// reaches a target t is
//     F_t(X) = q_t(X) + Σ_{s≠t} q_s(X) P_st(X),
// the source qualities with their gains, each times the probability of its
// most reliable path to t; t scores (tq_t + Σ_{o∈X} Δtq_to) F_t(X) and the
// criterion φ combines the cases' scores. On the graph contracted for t,
// the part of a source s is the optimum at x, the indicator of X, of
//
//     max  f
//     s.t. Σ_{a∈δ⁺(u)} φ_a + [u = t] f
//              <= Σ_{a∈δ⁻(u)} p_a φ_a + [u = s] q_s(x)    ρ_u   ∀ vertex u
//          φ_a <= m_sa x_o                               σ_a   ∀ a improved by
//          o φ, f >= 0
//
// on the arcs A_s that a most reliable path from s to t can take with some
// options, where m_sa = q_s^max P^max(s, u) bounds what s brings to a = (u, w):
// its quality with every gain times the probability of its most reliable
// path to u with every option. Its dual does not depend on x: any ρ with
// ρ_t = 1 and ρ_u >= p_a ρ_w on the arcs of A_s without option gives
//     F_t <= cut_ρ(x) = q_t(x) + Σ_s [ ρ^s_s q_s(x)
//                         + Σ_{a=(u,w)∈A_s improved by o}
//                               m_sa (p_a ρ^s_w - ρ^s_u)⁺ x_o ]
// for every X. flow_benders' cut has one ρ and one big-M per arc for all the
// sources; here an option pays for a source only what that source can bring
// to its arcs. The master is flow_benders':
//
//     max  φ(z)
//     s.t. z_k <= Σ_{t∈k} (tq_t F_t + Σ_o Δtq_to F'_to)
//          F'_to <= F_t,  F'_to <= M_t x_o
//          F_t <= cut_ρ(x)               for each ρ separated
//          Σ_o c_o x_o <= B,  x binary
//
// The cut of a point x̄, fractional or not, takes for each source the ρ of
// its largest flow at x̄. It is separated first on the linear relaxation,
// until its optimum violates no cut, then at the solver's candidates.
struct multiflow_benders : public mip_solver_base {
    // for comparison: no cut on the linear relaxation before the search, and
    // each optimum of the relaxation separated itself, without the center
    bool no_root_cuts = false;
    bool no_in_out = false;

    // The graph contracted for a target t, each improvement an arc of its
    // own, and for each source the arcs that can carry its flow to t.
    struct target_block {
        using vertex = std::uint32_t;
        using arc_id = std::uint32_t;
        static constexpr double infinity =
            std::numeric_limits<double>::infinity();
        static constexpr option_t no_option =
            std::numeric_limits<option_t>::max();

        struct arc {
            vertex tail, head;
            double probability;
            option_t option;  // no_option: there without option
        };
        // A_s, with m_sa for its improved arcs and infinity for the others
        struct source {
            vertex s;
            std::vector<arc_id> arcs;
            std::vector<double> big_Ms;
        };
        // F_t <= constant + Σ_o coefficients[o] x_o for every X; value is
        // the flow that reaches t at the point separated
        struct cut {
            double value = 0, constant = 0;
            std::vector<double> coefficients;
        };

        case_id_t case_id;
        vertex t;
        double target_quality;  // tq_t
        std::vector<std::pair<double, option_t>> target_quality_gains;
        std::vector<double> qualities;  // q_u
        std::vector<std::vector<std::pair<double, option_t>>> quality_gains;
        std::vector<arc> arcs;
        std::vector<std::vector<arc_id>> in_arcs, out_arcs;
        std::vector<source> sources;

        template <case_c C>
        explicit target_block(const contracted_graph_data<C> & data)
            : case_id(data.instance_case.get().id())
            , t(static_cast<vertex>(data.t))
            , target_quality(data.instance_case.get()
                                 .target_quality_map()[data.original_t])
            , in_arcs(data.graph.num_vertices())
            , out_arcs(data.graph.num_vertices()) {
            for(const auto & [sqg, tqg, o] :
                data.instance_case.get().vertex_options_map()[data.original_t])
                if(tqg > 0) target_quality_gains.emplace_back(tqg, o);
            for(const auto & u : melon::vertices(data.graph)) {
                qualities.push_back(data.source_quality_map[u]);
                quality_gains.push_back(data.source_options_map[u]);
            }
            auto add_arc = [this](const vertex u, const vertex w,
                                  const double p, const option_t o) {
                out_arcs[u].push_back(static_cast<arc_id>(arcs.size()));
                in_arcs[w].push_back(static_cast<arc_id>(arcs.size()));
                arcs.push_back({u, w, p, o});
            };
            for(const auto & a : melon::arcs(data.graph)) {
                const auto u = static_cast<vertex>(data.graph.arc_source(a));
                const auto w = static_cast<vertex>(data.graph.arc_target(a));
                if(data.probability_map[a] > 0)
                    add_arc(u, w, data.probability_map[a], no_option);
                for(const auto & [p, o] : data.arc_options_map[a])
                    add_arc(u, w, p, o);
            }
        }

        // the probabilities of the most reliable paths to root, or from it,
        // with every option or none
        std::vector<double> reach(const vertex root, const bool to_root,
                                  const bool with_options) const {
            std::vector<double> probabilities(qualities.size(), 0.0);
            std::priority_queue<std::pair<double, vertex>> heap;
            probabilities[root] = 1;
            heap.emplace(1.0, root);
            while(!heap.empty()) {
                const auto [p, u] = heap.top();
                heap.pop();
                if(p < probabilities[u]) continue;
                for(const arc_id a : to_root ? in_arcs[u] : out_arcs[u]) {
                    if(!with_options && arcs[a].option != no_option) continue;
                    const vertex w = to_root ? arcs[a].tail : arcs[a].head;
                    const double q = p * arcs[a].probability;
                    if(q <= probabilities[w]) continue;
                    probabilities[w] = q;
                    heap.emplace(q, w);
                }
            }
            return probabilities;
        }

        // An arc a = (u, w) is in A_s if the best path of s through it, with
        // every option, is as reliable as its best path without option: a
        // most reliable path of s is at least that reliable whatever X.
        void compute_sources() {
            const auto base_to_t = reach(t, true, false);
            const auto best_to_t = reach(t, true, true);
            for(vertex s = 0; s < qualities.size(); ++s) {
                double best_quality = qualities[s];
                for(const auto & [gain, o] : quality_gains[s])
                    best_quality += gain;
                if(s == t || best_quality <= 0 || best_to_t[s] <= 0) continue;
                const auto best_from_s = reach(s, false, true);
                source & source_s = sources.emplace_back(s);
                for(arc_id a = 0; a < arcs.size(); ++a) {
                    const double through = best_from_s[arcs[a].tail] *
                                           arcs[a].probability *
                                           best_to_t[arcs[a].head];
                    if(through <= 0 || through < base_to_t[s] * (1 - 1e-9))
                        continue;
                    source_s.arcs.push_back(a);
                    source_s.big_Ms.push_back(
                        arcs[a].option == no_option
                            ? infinity
                            : best_quality * best_from_s[arcs[a].tail]);
                }
            }
        }

        // The cut of the point x. For each source, its largest flow to t at
        // x by highest-gain augmenting paths: ρ_u is the gain of the best
        // path from u to t in the residual graph, and the ρ of the last one
        // are an optimal dual solution. At the indicator of X one path
        // takes the whole supply, so value is F_t(X).
        cut separate(const std::vector<double> & x) const {
            const std::size_t n = qualities.size();
            cut result{qualities[t], qualities[t],
                       std::vector<double>(x.size(), 0.0)};
            for(const auto & [gain, o] : quality_gains[t]) {
                result.coefficients[o] += gain;
                result.value += gain * x[o];
            }
            std::vector<double> capacities(arcs.size()), flows(arcs.size()),
                rho(n);
            std::vector<char> in_A(arcs.size(), 0), queued(n, 0);
            // the first arc of the best residual path from a vertex to t,
            // and whether it is taken forward
            std::vector<std::pair<arc_id, bool>> next(n);
            std::vector<vertex> queue;

            for(const source & source_s : sources) {
                const vertex s = source_s.s;
                double supply = qualities[s];
                for(const auto & [gain, o] : quality_gains[s])
                    supply += gain * x[o];
                for(std::size_t k = 0; k < source_s.arcs.size(); ++k) {
                    const arc_id a = source_s.arcs[k];
                    in_A[a] = 1;
                    flows[a] = 0;
                    capacities[a] =
                        arcs[a].option == no_option
                            ? infinity
                            : source_s.big_Ms[k] * x[arcs[a].option];
                }
                const double negligible = 1e-12 * supply;
                // Label-correcting from t: a residual arc taken backward has
                // a gain above 1, which rules Dijkstra out. False if the
                // labels do not settle.
                const auto label = [&] {
                    std::ranges::fill(rho, 0.0);
                    rho[t] = 1;
                    queue.assign(1, t);
                    for(std::size_t i = 0; i < queue.size(); ++i) {
                        if(i > 1000 * n) {
                            std::ranges::fill(queued, 0);
                            return false;
                        }
                        const vertex w = queue[i];
                        queued[w] = 0;
                        auto improve = [&](const vertex u, const double gain,
                                           const arc_id a, const bool forward) {
                            if(gain <= rho[u] * (1 + 1e-12)) return;
                            rho[u] = gain;
                            next[u] = {a, forward};
                            if(queued[u]) return;
                            queued[u] = 1;
                            queue.push_back(u);
                        };
                        // infinity first: inf - flow <= 1e-11 inf holds, and
                        // would saturate the arcs without option
                        for(const arc_id a : in_arcs[w])
                            if(in_A[a] && (capacities[a] == infinity ||
                                           capacities[a] - flows[a] >
                                               1e-11 * capacities[a]))
                                improve(arcs[a].tail,
                                        arcs[a].probability * rho[w], a, true);
                        for(const arc_id a : out_arcs[w])
                            if(in_A[a] && flows[a] > negligible)
                                improve(arcs[a].head,
                                        rho[w] / arcs[a].probability, a, false);
                    }
                    return true;
                };
                double remaining = supply, arrived = 0;
                bool settled = true;
                for(std::size_t pushes = 0;; ++pushes) {
                    settled = label();
                    if(!settled || remaining <= negligible || rho[s] <= 0)
                        break;
                    // the most the path takes, in units leaving s
                    double amount = remaining, gain = 1;
                    std::size_t length = 0;
                    for(vertex u = s; u != t && settled;
                        settled = ++length <= 4 * n) {
                        const auto [a, forward] = next[u];
                        if(forward) {
                            amount = std::min(
                                amount, (capacities[a] - flows[a]) / gain);
                            gain *= arcs[a].probability;
                            u = arcs[a].head;
                        } else {
                            amount = std::min(
                                amount, flows[a] * arcs[a].probability / gain);
                            gain /= arcs[a].probability;
                            u = arcs[a].tail;
                        }
                    }
                    if(!settled || pushes == 100000) {
                        settled = false;
                        break;
                    }
                    remaining -= amount;
                    for(vertex u = s; u != t;) {
                        const auto [a, forward] = next[u];
                        if(forward) {
                            flows[a] += amount;
                            amount *= arcs[a].probability;
                            u = arcs[a].head;
                        } else {
                            amount /= arcs[a].probability;
                            flows[a] -= amount;
                            u = arcs[a].tail;
                        }
                    }
                    arrived += amount;
                }
                if(!settled) {
                    // the ρ with every arc of A_s free: feasible too, and
                    // the options then pay nothing for s
                    for(const arc_id a : source_s.arcs) {
                        flows[a] = 0;
                        capacities[a] = infinity;
                    }
                    label();
                    arrived = rho[s] * supply;
                }
                result.value += arrived;
                result.constant += rho[s] * qualities[s];
                for(const auto & [gain, o] : quality_gains[s])
                    result.coefficients[o] += rho[s] * gain;
                for(std::size_t k = 0; k < source_s.arcs.size(); ++k) {
                    const arc & a = arcs[source_s.arcs[k]];
                    in_A[source_s.arcs[k]] = 0;
                    if(a.option == no_option) continue;
                    const double sigma =
                        a.probability * rho[a.head] - rho[a.tail];
                    if(sigma > 0)
                        result.coefficients[a.option] +=
                            source_s.big_Ms[k] * sigma;
                }
            }
            return result;
        }
    };

    template <lazy_constraint_model M, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using cut = target_block::cut;
        spdlog::stopwatch sw;

        // The master: z with the criterion, x with the budget row, and for
        // each target its block, F_t and its F'_to, in the order of the
        // block's target quality gains.
        M model;
        model.set_optimality_tolerance(1e-10);
        model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();
        const auto z = model.add_variables(
            instance.cases().size(), [](const case_id_t & k) { return k; });
        formula_variable_visitor formula{model, z};
        model.set_objective(formula.variable(instance.criterion()));
        const auto x = model.add_binary_variables(
            instance.options().size(), [](const option_t & o) { return o; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * x(o);
                             }) <= budget);

        std::vector<target_block> blocks;
        std::vector<model_variable_t<M>> F, F_prime;
        for(auto && instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions of the '{}' graph",
                          instance_case.name());
            const auto z_row = model.add_constraint(z(instance_case.id()) <= 0);
            progress_bar<spdlog::level::info, 64> pb(
                instance_case.graph().num_vertices());
            for(const auto & t : melon::vertices(instance_case.graph())) {
                pb.tick();
                if(!_is_target(instance_case, t)) continue;
                const auto data = compute_contracted_graph_data(
                    instance, instance_case, budget, t, strong_arcs_map[t],
                    useless_arcs_map[t], big_M_kind::knapsack, feasibility_tol,
                    !no_pruning);
                const target_block & block = blocks.emplace_back(data);
                // M_t bounds F_t: without it the relaxation is unbounded
                // until t receives a cut
                const double M_t = data.big_M_map[data.t];
                F.push_back(model.add_column(
                    {std::make_pair(z_row, -block.target_quality)},
                    {.upper_bound = M_t}));
                for(const auto & [gain, o] : block.target_quality_gains) {
                    const auto F_o = F_prime.emplace_back(
                        model.add_column({std::make_pair(z_row, -gain)}));
                    model.add_constraint(F_o <= F.back());
                    model.add_constraint(F_o <= M_t * x(o));
                }
            }
        }
        tbb::parallel_for(
            std::size_t{0}, blocks.size(),
            [&](const std::size_t i) { blocks[i].compute_sources(); });
        std::size_t num_sources = 0, num_source_arcs = 0;
        for(const target_block & block : blocks) {
            num_sources += block.sources.size();
            for(const auto & source : block.sources)
                num_source_arcs += source.arcs.size();
        }
        spdlog::debug("  {} targets, {} sources, {} arcs over the sources",
                      blocks.size(), num_sources, num_source_arcs);

        auto separate = [&](const std::vector<double> & point) {
            std::vector<cut> cuts(blocks.size());
            tbb::parallel_for(std::size_t{0}, blocks.size(),
                              [&](const std::size_t i) {
                                  cuts[i] = blocks[i].separate(point);
                              });
            return cuts;
        };
        // a lazy sum that reads the cut: only valid while the cut lives
        auto cut_constraint = [&](const std::size_t i, const cut & c) {
            return F[i] <=
                   c.constant + xsum(std::views::filter(
                                         instance.options(),
                                         [&c](const option_t o) {
                                             return c.coefficients[o] != 0;
                                         }),
                                     [&](const option_t o) {
                                         return c.coefficients[o] * x(o);
                                     });
        };
        std::vector<double> point(instance.options().size());
        if(mip_start) {
            const auto start = _mip_start_solution(instance);
            model.add_mip_start(std::views::transform(
                instance.options(),
                [&](auto o) { return std::make_pair(x(o), start[o]); }));
            for(auto && o : instance.options()) point[o] = start[o];
            const auto cuts = separate(point);
            for(std::size_t i = 0; i < blocks.size(); ++i)
                model.add_constraint(cut_constraint(i, cuts[i]));
        }

        // The cuts of the linear relaxation, within half of the time limit.
        // In-out: as long as it cuts, the point separated is a center that
        // moves halfway to each optimum, whose cuts are deeper than the
        // optimum's own; then the optimum itself, until it violates none.
        auto root_time_left = [&] {
            return time_limit.value() / 2 - sw.elapsed().count();
        };
        for(auto && o : instance.options()) model.set_continuous(x(o));
        std::size_t root_rounds = 0, root_cuts = 0;
        std::optional<double> root_value;
        std::vector<double> center, flows(blocks.size());
        for(bool in_out = !no_in_out; !no_root_cuts && !stop_requested &&
                                      !(time_limit && root_time_left() <= 0);
            ++root_rounds) {
            if(time_limit)
                model.set_time_limit(std::chrono::duration<double>(
                    std::max(0.0, root_time_left())));
            model.solve();
            if(!std::holds_alternative<status::optimal>(model.get_status()))
                break;
            root_value = model.get_solution_value();
            report_progress({.relaxation = root_value, .round = root_rounds});
            {
                // readable until the model changes
                const auto relaxation = model.get_solution();
                for(auto && o : instance.options())
                    point[o] = std::clamp(relaxation[x(o)], 0.0, 1.0);
                for(std::size_t i = 0; i < blocks.size(); ++i)
                    flows[i] = relaxation[F[i]];
            }
            if(in_out) {
                if(center.empty()) center = point;
                for(auto && o : instance.options())
                    center[o] = (center[o] + point[o]) / 2;
            }
            const auto cuts = separate(in_out ? center : point);
            std::size_t num_added = 0;
            for(std::size_t i = 0; i < blocks.size(); ++i) {
                double bound = cuts[i].constant;
                for(auto && o : instance.options())
                    bound += cuts[i].coefficients[o] * point[o];
                if(flows[i] <= bound * (1 + 1e-7) + 1e-7) continue;
                model.add_constraint(cut_constraint(i, cuts[i]));
                ++num_added;
            }
            root_cuts += num_added;
            if(num_added) continue;
            if(!in_out) break;
            in_out = false;
        }
        for(auto && o : instance.options()) model.set_binary(x(o));
        spdlog::debug("  {} rounds and {} cuts on the relaxation", root_rounds,
                      root_cuts);

        // The solver drops a candidate that receives cuts and may never
        // propose its options again, although they are a solution: it gets
        // them back with their flows and scores.
        std::size_t num_cuts = 0;
        model.set_candidate_solution_callback(
            [&](typename M::candidate_solution_callback_handle & handle) {
                auto candidate = handle.get_solution();
                std::vector<double> Y(instance.options().size());
                for(auto && o : instance.options())
                    Y[o] = candidate[x(o)] > 0.5;
                const auto cuts = separate(Y);
                auto case_scores = instance.create_case_map(0.0);
                const std::size_t num_cuts_before = num_cuts;
                auto F_o = F_prime.begin();
                for(std::size_t i = 0; i < blocks.size(); ++i) {
                    const double flow = cuts[i].value;
                    double quality = blocks[i].target_quality;
                    for(const auto & [gain, o] :
                        blocks[i].target_quality_gains) {
                        candidate[*F_o++] = Y[o] * flow;
                        quality += Y[o] * gain;
                    }
                    case_scores[blocks[i].case_id] += quality * flow;
                    if(std::exchange(candidate[F[i]], flow) - flow <=
                       feasibility_tol * flow)
                        continue;
                    ++num_cuts;
                    handle.add_lazy_constraint(cut_constraint(i, cuts[i]));
                }
                if(num_cuts == num_cuts_before) return;
                for(auto && o : instance.options()) candidate[x(o)] = Y[o];
                for(auto && instance_case : instance.cases())
                    candidate[z(instance_case.id())] =
                        case_scores[instance_case.id()];
                formula.set_values(candidate, case_scores);
                handle.add_solution(candidate);
            });

        auto [has_solution, report] = _solve(model, sw);
        report.stats.emplace_back("sources", num_sources);
        report.stats.emplace_back("root_rounds", root_rounds);
        report.stats.emplace_back("root_cuts", root_cuts);
        report.stats.emplace_back("lazy_constraints", num_cuts);
        // the relaxation with its cuts bounds the optimum too, better than
        // the solver's bound when the search is cut short
        if(root_value) {
            report.best_bound =
                std::min(report.best_bound.value_or(*root_value), *root_value);
            report.score_stats.emplace_back("root_relaxation_value",
                                            *root_value);
        }
        if(!has_solution) return {_fallback_solution(instance, budget), report};
        const auto master_solution = model.get_solution();
        auto solution = instance.create_option_map(false);
        for(auto && o : instance.options())
            solution[o] = master_solution[x(o)] > 0.5;
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
