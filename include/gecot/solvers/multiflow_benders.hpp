#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <type_traits>
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
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The decomposition of docs/formulations/benders.md with a flow per source
// and target. Each block of solve() builds what the section it names states.
struct multiflow_benders : public mip_solver_base {
    // for comparison: no cut on the linear relaxation before the search, and
    // each optimum of the relaxation separated itself, without the center
    bool no_root_cuts = false;
    bool no_in_out = false;
    // A cut is added to the relaxation when f_t exceeds it by more than
    // this, relative and absolute. It does not follow feasibility_tol.
    static constexpr double root_cut_tol = 1e-7;

    // The graph G_t of a target t, each improved copy an arc of its own, and
    // for each source the arcs that can carry its flow to t.
    struct target_block {
        using graph_t = melon::static_digraph;
        using vertex = melon::vertex_t<graph_t>;
        using arc = melon::arc_t<graph_t>;
        template <typename V>
        using vertex_map = melon::vertex_map_t<graph_t, V>;
        template <typename V>
        using arc_map = melon::arc_map_t<graph_t, V>;
        static constexpr double infinity =
            std::numeric_limits<double>::infinity();
        // the relative slack of the test of A_s against the best path
        // without option
        static constexpr double path_tie = 1e-9;
        // In the search of a flow, relative: an arc is full within arc_full
        // of its capacity, and a flow under no_flow times the supply is none.
        static constexpr double arc_full = 1e-11, no_flow = 1e-12;
        // Past this many pushes, the search gives up and the source takes
        // the fallback.
        static constexpr std::size_t max_pushes = 100000;
        using reversed_traits = detail::pc_num_dijkstra_traits<
            std::decay_t<decltype(melon::views::reverse(
                std::declval<const graph_t &>()))>,
            double>;

        // A_s, and its improved copies with their m_sa
        struct source {
            vertex s;
            std::vector<arc> arcs;
            std::vector<std::pair<arc, double>> improved_copies;
        };
        // f_t <= constant + Σ_i coefficients[i] x_i in every scenario; value
        // is the flow that reaches t at the point separated. When the flow
        // of a source did not settle, it is a bound on that flow at a
        // fractional point, and still the flow at a scenario.
        struct cut {
            double value = 0, constant = 0;
            std::vector<double> coefficients;
        };

        case_id_t case_id;
        double target_quality;  // q_t
        std::vector<std::pair<double, option_t>> target_quality_gains;
        graph_t graph;
        vertex t;
        vertex_map<double> source_quality_map;  // w_u
        vertex_map<std::vector<std::pair<double, option_t>>> source_options_map;
        // of an improved copy: its probability with its option
        arc_map<double> probability_map;
        arc_map<std::optional<option_t>> option_map;
        std::vector<source> sources;

        // G_t: the graph of original_t as compute_contracted_flow_graph
        // gives it. Its vertices are renumbered here.
        template <case_c C>
        target_block(const C & instance_case, const auto & original_t,
                     contracted_flow_graph && G_t)
            : case_id(instance_case.id())
            , target_quality(instance_case.target_quality_map()[original_t]) {
            for(const auto & [source_gain, target_gain, i] :
                instance_case.vertex_options_map()[original_t])
                if(target_gain > 0)
                    target_quality_gains.emplace_back(target_gain, i);
            const auto vertex_cmp = [](auto && u, auto && v) { return u < v; };
            t = static_cast<vertex>(std::ranges::count_if(
                melon::vertices(G_t.graph),
                [&](auto && u) { return vertex_cmp(u, G_t.t); }));
            std::tie(graph, source_quality_map, source_options_map,
                     probability_map, option_map) =
                melon::make_static_digraph(
                    G_t.graph, vertex_cmp,
                    std::tie(G_t.source_quality_map, G_t.source_options_map),
                    std::tie(G_t.probability_map, G_t.improvement_option_map));
        }

        // An arc a = (u, v) is in A_s if the best path of s through it, with
        // every option, is as reliable as its best path without option: a
        // most reliable path of s is at least that reliable in every
        // scenario.
        void compute_sources() {
            const auto reversed_graph = melon::views::reverse(graph);
            // the probabilities of connection to t, without option and with
            // every one
            auto base_to_t = melon::create_vertex_map<double>(graph, 0.0);
            for(const auto & [u, probability] : melon::dijkstra(
                    reversed_traits{}, reversed_graph,
                    melon::maps::function([this](const arc & a) {
                        return option_map[a] ? 0.0 : probability_map[a];
                    }),
                    t))
                base_to_t[u] = probability;
            auto best_to_t = melon::create_vertex_map<double>(graph, 0.0);
            for(const auto & [u, probability] : melon::dijkstra(
                    reversed_traits{}, reversed_graph, probability_map, t))
                best_to_t[u] = probability;

            auto from_s = melon::dijkstra(
                detail::pc_num_dijkstra_traits<graph_t, double>{}, graph,
                probability_map);
            auto best_from_s = melon::create_vertex_map<double>(graph);
            for(const vertex s : melon::vertices(graph)) {
                double best_quality = source_quality_map[s];
                for(const auto & [source_gain, i] : source_options_map[s])
                    best_quality += source_gain;
                if(s == t || best_quality <= 0 || best_to_t[s] <= 0) continue;
                best_from_s.fill(0.0);
                from_s.reset().add_source(s);
                for(const auto & [u, probability] : from_s)
                    best_from_s[u] = probability;
                source & source_s = sources.emplace_back(s);
                for(const arc a : melon::arcs(graph)) {
                    const vertex u = melon::arc_source(graph, a);
                    const double through =
                        best_from_s[u] * probability_map[a] *
                        best_to_t[melon::arc_target(graph, a)];
                    if(through <= 0 || through < base_to_t[s] * (1 - path_tie))
                        continue;
                    source_s.arcs.push_back(a);
                    if(option_map[a])
                        source_s.improved_copies.emplace_back(
                            a, best_quality * best_from_s[u]);
                }
            }
        }

        // What the searches of a separation reuse from a source to the next.
        struct flow_search {
            // of an arc without option: infinity, whatever the source
            arc_map<double> capacities;
            arc_map<double> flows;
            arc_map<char> in_A_s;
            // 0 outside reached, which the next search resets
            vertex_map<double> rho;
            // the labels of the last search, only read where it reached
            vertex_map<double> potentials;
            // 0 outside the heap, 1 in it, 2 settled
            vertex_map<char> status;
            // the first arc of the best residual path from a vertex to t,
            // and whether it is taken forward
            vertex_map<std::pair<arc, bool>> next;
            std::vector<vertex> reached;
            detail::pc_num_dijkstra_traits<graph_t, double>::heap heap;

            explicit flow_search(const graph_t & graph)
                : capacities(melon::create_arc_map<double>(graph, infinity))
                , flows(melon::create_arc_map<double>(graph))
                , in_A_s(melon::create_arc_map<char>(graph, 0))
                , rho(melon::create_vertex_map<double>(graph, 0.0))
                , potentials(melon::create_vertex_map<double>(graph))
                , status(melon::create_vertex_map<char>(graph, 0))
                , next(melon::create_vertex_map<std::pair<arc, bool>>(graph))
                , heap(std::greater<double>{},
                       melon::create_vertex_map<std::size_t>(graph)) {}
        };

        // The largest flow of the source to t at x̂, by highest-gain
        // augmenting paths: returns what reaches t, and leaves in search.rho
        // an optimal dual solution, ρ_u being the gain of the best path from
        // u to t in the last residual graph. If the search gives up, ρ is
        // the fallback's and the value returned a bound, exact at a scenario.
        double _largest_flow(const source & source_s, const double supply,
                             const std::vector<double> & x_hat,
                             flow_search & search) const {
            auto & [capacities, flows, in_A_s, rho, potentials, status, next,
                    reached, heap] = search;
            const vertex s = source_s.s;
            for(const arc a : source_s.arcs) {
                in_A_s[a] = 1;
                flows[a] = 0;
            }
            for(const auto & [a, big_M] : source_s.improved_copies)
                capacities[a] = big_M * x_hat[option_map[a].value()];
            const double negligible = no_flow * supply;
            // Dijkstra from t, the highest label first. A residual arc taken
            // backward has a gain above 1, so past the first search the order
            // is that of the labels reduced by those of the last search,
            // ρ_u / p_u: a push adds residual arcs on its path only, where
            // the p are tight, so no reduced gain exceeds 1 and the search
            // reaches no vertex that the last one did not.
            const auto label = [&](const bool first) {
                for(const vertex v : reached) {
                    potentials[v] = rho[v];
                    rho[v] = 0;
                    status[v] = 0;
                }
                rho[t] = 1;
                reached.assign(1, t);
                heap.push({t, 1.0});
                while(!heap.empty()) {
                    const vertex v = heap.top().first;
                    heap.pop();
                    status[v] = 2;
                    auto improve = [&](const vertex u, const double gain,
                                       const arc a, const bool forward) {
                        if(status[u] == 2 || gain <= rho[u]) return;
                        rho[u] = gain;
                        next[u] = {a, forward};
                        const double key = first ? gain : gain / potentials[u];
                        if(status[u] == 1) {
                            heap.promote(u, key);
                            return;
                        }
                        status[u] = 1;
                        heap.push({u, key});
                        reached.push_back(u);
                    };
                    // infinity first: inf - flow <= arc_full inf holds, and
                    // would fill the arcs without option
                    for(const arc a : melon::in_arcs(graph, v))
                        if(in_A_s[a] && (capacities[a] == infinity ||
                                         capacities[a] - flows[a] >
                                             arc_full * capacities[a]))
                            improve(melon::arc_source(graph, a),
                                    probability_map[a] * rho[v], a, true);
                    for(const arc a : melon::out_arcs(graph, v))
                        if(in_A_s[a] && flows[a] > negligible)
                            improve(melon::arc_target(graph, a),
                                    rho[v] / probability_map[a], a, false);
                }
            };
            double remaining = supply, arrived = 0;
            bool settled = true;
            for(std::size_t pushes = 0;; ++pushes) {
                label(pushes == 0);
                if(remaining <= negligible || rho[s] <= 0) break;
                if(pushes == max_pushes) {
                    settled = false;
                    break;
                }
                // the most the path takes, in units leaving s
                double amount = remaining, gain = 1;
                for(vertex u = s; u != t;) {
                    const auto [a, forward] = next[u];
                    if(forward) {
                        amount =
                            std::min(amount, (capacities[a] - flows[a]) / gain);
                        gain *= probability_map[a];
                        u = melon::arc_target(graph, a);
                    } else {
                        amount = std::min(amount,
                                          flows[a] * probability_map[a] / gain);
                        gain /= probability_map[a];
                        u = melon::arc_source(graph, a);
                    }
                }
                remaining -= amount;
                // whether the push changed the arcs that a label can take in
                // the residual graph: it took one backward, or filled one
                bool changed = false;
                for(vertex u = s; u != t;) {
                    const auto [a, forward] = next[u];
                    if(forward) {
                        flows[a] += amount;
                        changed = changed || (capacities[a] != infinity &&
                                              capacities[a] - flows[a] <=
                                                  arc_full * capacities[a]);
                        amount *= probability_map[a];
                        u = melon::arc_target(graph, a);
                    } else {
                        amount /= probability_map[a];
                        flows[a] -= amount;
                        changed = true;
                        u = melon::arc_source(graph, a);
                    }
                }
                arrived += amount;
                // The labels are then those of the next search: an arc that
                // received flow gives its head, taken backward, the label it
                // already has.
                if(!changed && remaining <= negligible) break;
            }
            if(!settled) {
                // The ρ of the arcs of A_s open at x̂, without any flow:
                // feasible too. At a scenario they are the probabilities of
                // connection, so the value returned stays the true flow: the
                // callback hands it to the solver as the score of the
                // candidate.
                for(const arc a : source_s.arcs) flows[a] = 0;
                label(true);
                arrived = rho[s] * supply;
            }
            for(const arc a : source_s.arcs) in_A_s[a] = 0;
            return arrived;
        }

        // The cut of the point x̂: for each source, its largest flow to t at
        // x̂ and the dual solution that comes with it. At a scenario y one
        // path takes the whole supply, so value is f_t(y).
        cut separate(const std::vector<double> & x_hat) const {
            cut result{source_quality_map[t], source_quality_map[t],
                       std::vector<double>(x_hat.size(), 0.0)};
            for(const auto & [source_gain, i] : source_options_map[t]) {
                result.coefficients[i] += source_gain;
                result.value += source_gain * x_hat[i];
            }
            flow_search search(graph);
            const auto & rho = search.rho;
            for(const source & source_s : sources) {
                const vertex s = source_s.s;
                double supply = source_quality_map[s];
                for(const auto & [source_gain, i] : source_options_map[s])
                    supply += source_gain * x_hat[i];
                result.value += _largest_flow(source_s, supply, x_hat, search);
                result.constant += rho[s] * source_quality_map[s];
                for(const auto & [source_gain, i] : source_options_map[s])
                    result.coefficients[i] += rho[s] * source_gain;
                for(const auto & [a, big_M] : source_s.improved_copies) {
                    const double sigma =
                        probability_map[a] * rho[melon::arc_target(graph, a)] -
                        rho[melon::arc_source(graph, a)];
                    if(sigma > 0)
                        result.coefficients[option_map[a].value()] +=
                            big_M * sigma;
                }
            }
            return result;
        }
    };

    // the sources of every block; returns their number
    static std::size_t _compute_sources(std::vector<target_block> & blocks) {
        tbb::parallel_for(
            std::size_t{0}, blocks.size(),
            [&](const std::size_t b) { blocks[b].compute_sources(); });
        std::size_t num_sources = 0, num_source_arcs = 0;
        for(const target_block & block : blocks) {
            num_sources += block.sources.size();
            for(const auto & source : block.sources)
                num_source_arcs += source.arcs.size();
        }
        spdlog::debug("{} targets, {} sources, {} arcs over the sources",
                      blocks.size(), num_sources, num_source_arcs);
        return num_sources;
    }

    static std::vector<target_block::cut> _separate(
        const std::vector<target_block> & blocks,
        const std::vector<double> & x_hat) {
        std::vector<target_block::cut> cuts(blocks.size());
        tbb::parallel_for(
            std::size_t{0}, blocks.size(),
            [&](const std::size_t b) { cuts[b] = blocks[b].separate(x_hat); });
        return cuts;
    }

    template <lazy_constraint_model M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using cut = target_block::cut;
        _check_criterion(instance);
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);
        // the rounds on the relaxation solve the model before _solve does
        _set_numeric_focus(model);

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

        std::vector<target_block> blocks;
        // f_t block by block, and the f^i_t: block by block, in the order of
        // the block's target quality gains
        std::vector<model_variable_t<M>> f, f_gains;
        std::vector<preprocessing_row<>> preprocessing;
        for(auto && instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            preprocessing.emplace_back(
                instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map));
            spdlog::debug("Computing contractions of the '{}' graph",
                          instance_case.name());

            // flow_benders, the master: the score of the case, a row that
            // the columns of its f_t and f^i_t fill
            const case_id_t k = instance_case.id();
            const auto z_row = model.add_constraint(z(k) <= 0);
            progress_bar<spdlog::level::info, 64> pb(
                instance_case.graph().num_vertices());
            for(const auto & original_t :
                melon::vertices(instance_case.graph())) {
                pb.tick();
                if(!_is_target(instance_case, original_t)) continue;
                contracted_flow_graph G_t = compute_contracted_flow_graph(
                    instance, instance_case, budget, original_t,
                    strong_arcs_map[original_t], useless_arcs_map[original_t],
                    std::nullopt, feasibility_tol, !no_pruning);
                // multiflow_benders: f_t within M^t_t, the only big-M of
                // the vertices that is read here, without which the
                // relaxation is unbounded until t receives a cut, and f^i_t
                // for each positive target quality gain
                const double M_t = compute_knapsack_big_M_map(
                    instance, budget, false, G_t.graph, G_t.source_quality_map,
                    G_t.source_options_map, G_t.probability_map,
                    std::views::single(G_t.t))[G_t.t];
                const target_block & block = blocks.emplace_back(
                    instance_case, original_t, std::move(G_t));
                f.push_back(model.add_column(
                    {std::make_pair(z_row, -block.target_quality)},
                    {.upper_bound = M_t}));
                for(const auto & [target_gain, i] :
                    block.target_quality_gains) {
                    const auto f_ti = f_gains.emplace_back(model.add_column(
                        {std::make_pair(z_row, -target_gain)}));
                    model.add_constraint(f_ti <= f.back());
                    model.add_constraint(f_ti <= M_t * x(i));
                }
            }
        }
        _log_preprocessing(preprocessing);
        const std::size_t num_sources = _compute_sources(blocks);

        // multiflow_benders: the cut of the block b as a row. A lazy sum
        // that reads the cut: only valid while the cut lives.
        auto cut_constraint = [&](const std::size_t b, const cut & c) {
            return f[b] <=
                   c.constant + xsum(std::views::filter(
                                         instance.options(),
                                         [&c](const option_t i) {
                                             return c.coefficients[i] != 0;
                                         }),
                                     [&](const option_t i) {
                                         return c.coefficients[i] * x(i);
                                     });
        };
        // The search, step 1: the cuts of the MIP start
        std::vector<double> point(instance.options().size());
        if(const auto start = _add_mip_start(model, instance, x)) {
            for(auto && i : instance.options()) point[i] = (*start)[i];
            const auto cuts = _separate(blocks, point);
            for(std::size_t b = 0; b < blocks.size(); ++b)
                model.add_constraint(cut_constraint(b, cuts[b]));
        }

        // Cuts on the relaxation: the options are continuous for these
        // rounds, which take at most half of the time limit
        auto root_time_left = [&] {
            return time_limit.value() / 2 - sw.elapsed().count();
        };
        for(auto && i : instance.options()) model.set_continuous(x(i));
        std::size_t root_rounds = 0, root_cuts = 0;
        std::optional<double> root_value;
        std::vector<double> center, f_relaxed(blocks.size());
        for(bool in_out = !no_in_out; !no_root_cuts && !stop_requested &&
                                      !(time_limit && root_time_left() <= 0);) {
            if(time_limit)
                model.set_time_limit(std::chrono::duration<double>(
                    std::max(0.0, root_time_left())));
            model.solve();
            if(!std::holds_alternative<status::optimal>(model.get_status()))
                break;
            ++root_rounds;
            root_value = model.get_solution_value();
            report_progress({.relaxation = root_value, .round = root_rounds});
            {
                // readable until the model changes
                const auto relaxation = model.get_solution();
                for(auto && i : instance.options())
                    point[i] = std::clamp(relaxation[x(i)], 0.0, 1.0);
                for(std::size_t b = 0; b < blocks.size(); ++b)
                    f_relaxed[b] = relaxation[f[b]];
            }
            if(in_out) {
                if(center.empty()) center = point;
                for(auto && i : instance.options())
                    center[i] = (center[i] + point[i]) / 2;
            }
            const auto cuts = _separate(blocks, in_out ? center : point);
            std::size_t num_added = 0;
            for(std::size_t b = 0; b < blocks.size(); ++b) {
                double cut_at_optimum = cuts[b].constant;
                for(auto && i : instance.options())
                    cut_at_optimum += cuts[b].coefficients[i] * point[i];
                if(f_relaxed[b] <=
                   cut_at_optimum * (1 + root_cut_tol) + root_cut_tol)
                    continue;
                model.add_constraint(cut_constraint(b, cuts[b]));
                ++num_added;
            }
            root_cuts += num_added;
            if(num_added) continue;
            if(!in_out) break;
            in_out = false;
        }
        for(auto && i : instance.options()) model.set_binary(x(i));
        report_progress({.end = true});
        spdlog::debug("{} rounds and {} cuts on the relaxation", root_rounds,
                      root_cuts);

        // The search, steps 2 to 4: the flows of a candidate y, the cut at y
        // of each target whose f_t exceeds its flow, and y handed back
        std::size_t num_cuts = 0;
        model.set_candidate_solution_callback(
            [&](typename M::candidate_solution_callback_handle & handle) {
                auto candidate = handle.get_solution();
                std::vector<double> y(instance.options().size());
                for(auto && i : instance.options())
                    y[i] = _is_bought(candidate[x(i)]);
                const auto cuts = _separate(blocks, y);
                auto case_scores = instance.create_case_map(0.0);
                const std::size_t num_cuts_before = num_cuts;
                auto f_gain = f_gains.begin();
                for(std::size_t b = 0; b < blocks.size(); ++b) {
                    const double flow = cuts[b].value;
                    double quality = blocks[b].target_quality;
                    for(const auto & [target_gain, i] :
                        blocks[b].target_quality_gains) {
                        candidate[*f_gain++] = y[i] * flow;
                        quality += y[i] * target_gain;
                    }
                    case_scores[blocks[b].case_id] += quality * flow;
                    if(std::exchange(candidate[f[b]], flow) - flow <=
                       feasibility_tol * flow)
                        continue;
                    ++num_cuts;
                    handle.add_lazy_constraint(cut_constraint(b, cuts[b]));
                }
                if(num_cuts > num_cuts_before)
                    _hand_back(handle, candidate, instance, x, y, z,
                               case_scores, formula);
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
        return {_bought_options(instance, model.get_solution(), x), report};
    }
};

}  // namespace solvers
}  // namespace gecot
