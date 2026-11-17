#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/mapping.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

namespace gecot {
namespace solvers {

// The subproblem of multiflow_benders for a target t (benders.md): its graph
// G_t, each improved copy an arc of its own, for each source the arcs that
// can carry its flow to t, and the cut of a point.
// MaxPushes is a parameter for the tests, which reach the fallback of a
// source with 0 and 1.
template <std::size_t MaxPushes = 100000>
struct multiflow_target_block {
    using graph_t = melon::static_digraph;
    using vertex = melon::vertex_t<graph_t>;
    using arc = melon::arc_t<graph_t>;
    template <typename V>
    using vertex_map = melon::vertex_map_t<graph_t, V>;
    template <typename V>
    using arc_map = melon::arc_map_t<graph_t, V>;
    static constexpr double infinity = std::numeric_limits<double>::infinity();
    // the relative slack of the test of A_s against the best path
    // without option
    static constexpr double path_tie = 1e-9;
    // In the search of a flow, relative: an arc is full within arc_full
    // of its capacity, and a flow under no_flow times the supply is none.
    static constexpr double arc_full = 1e-11, no_flow = 1e-12;
    // Past this many pushes, the search gives up and the source takes
    // the fallback.
    static constexpr std::size_t max_pushes = MaxPushes;
    using reversed_traits = detail::pc_num_dijkstra_traits<
        std::decay_t<decltype(melon::views::reverse(
            std::declval<const graph_t &>()))>,
        double>;

    // A_s, and its improved copies with their m_sa, kept with
    // stored_arcs: they add up to at most the arcs of G_t times its
    // sources
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
        bool settled = true;  // the flow of every source
    };

    case_id_t case_id;
    double target_quality;  // q_t
    double flow_bound;      // M^t_t, the bound of f_t
    // with the constant M^t_i of the target quality gain of i
    std::vector<std::tuple<double, option_t, double>> target_quality_gains;
    graph_t graph;
    vertex t;
    vertex_map<double> source_quality_map;  // w_u
    vertex_map<std::vector<std::pair<double, option_t>>> source_options_map;
    // of an improved copy: its probability with its option
    arc_map<double> probability_map;
    arc_map<std::optional<option_t>> option_map;
    std::vector<source> sources;
    // the probabilities of connection to t, without option and with
    // every one
    vertex_map<double> base_to_t, best_to_t;

    // G_t: the graph of original_t as compute_contracted_flow_graph
    // gives it
    template <case_c C>
    multiflow_target_block(const C & instance_case, const auto & original_t,
                           contracted_flow_graph && G_t)
        : case_id(instance_case.id())
        , target_quality(instance_case.target_quality_map()[original_t]) {
        for(const auto & [source_gain, target_gain, i] :
            instance_case.vertex_options_map()[original_t])
            if(target_gain > 0)
                target_quality_gains.emplace_back(target_gain, i,
                                                  G_t.target_big_M(i));
        flow_bound = G_t.big_M_map.value()[G_t.t];
        std::tie(t, graph, source_quality_map, source_options_map,
                 probability_map, option_map) = static_flow_graph(G_t);
    }

    // w̄_s, with every source quality gain
    double _best_quality(const vertex s) const {
        double best_quality = source_quality_map[s];
        for(const auto & [source_gain, i] : source_options_map[s])
            best_quality += source_gain;
        return best_quality;
    }

    // the sources: the vertices that can bring something to t, with
    // their A_s if keep_arcs
    void compute_sources(const bool keep_arcs) {
        const auto reversed_graph = melon::views::reverse(graph);
        base_to_t = melon::create_vertex_map<double>(graph, 0.0);
        for(const auto & [u, probability] :
            melon::dijkstra(reversed_traits{}, reversed_graph,
                            melon::maps::function([this](const arc & a) {
                                return option_map[a] ? 0.0 : probability_map[a];
                            }),
                            t))
            base_to_t[u] = probability;
        best_to_t = melon::create_vertex_map<double>(graph, 0.0);
        for(const auto & [u, probability] : melon::dijkstra(
                reversed_traits{}, reversed_graph, probability_map, t))
            best_to_t[u] = probability;
        source_search search(graph);
        for(const vertex s : melon::vertices(graph)) {
            if(s == t || _best_quality(s) <= 0 || best_to_t[s] <= 0) continue;
            source & source_s = sources.emplace_back(s);
            if(keep_arcs) _source_arcs(source_s, search);
        }
    }

    // What the search from a source reuses from a source to the next.
    struct source_search {
        // Π̄(s, u), 0 outside the vertices of visited, which the next
        // search resets
        vertex_map<double> best_from_s;
        // 0 outside the heap, 1 in it, 2 settled
        vertex_map<char> status;
        std::vector<vertex> visited;
        detail::pc_num_dijkstra_traits<graph_t, double>::heap heap;

        explicit source_search(const graph_t & graph)
            : best_from_s(melon::create_vertex_map<double>(graph, 0.0))
            , status(melon::create_vertex_map<char>(graph, 0))
            , heap(std::greater<double>{},
                   melon::create_vertex_map<std::size_t>(graph)) {}
    };

    // An arc a = (u, v) is in A_s if the best path of s through it, with
    // every option, is as reliable as its best path without option: a
    // most reliable path of s is at least that reliable in every
    // scenario.
    void _source_arcs(source & source_s, source_search & search) const {
        auto & [best_from_s, status, visited, heap] = search;
        const vertex s = source_s.s;
        const double best_quality = _best_quality(s);
        const double threshold = base_to_t[s] * (1 - path_tie);
        source_s.arcs.clear();
        source_s.improved_copies.clear();
        for(const vertex v : visited) {
            best_from_s[v] = 0;
            status[v] = 0;
        }
        // Dijkstra from s, which does not go on from a vertex u with
        // Π̄(s, u) Π̄(u, t) under the threshold: no arc of A_s leaves u or
        // a vertex reached through it. A best path to the other vertices
        // crosses none of those, so their Π̄(s, u) are exact. The slack
        // covers the rounding of Π̄(s, u) π_a Π̄(v, t) against it.
        best_from_s[s] = 1;
        visited.assign(1, s);
        heap.push({s, 1.0});
        while(!heap.empty()) {
            const vertex u = heap.top().first;
            heap.pop();
            status[u] = 2;
            if(best_from_s[u] * best_to_t[u] < threshold * (1 - 1e-12))
                continue;
            for(const arc a : melon::out_arcs(graph, u)) {
                const vertex v = melon::arc_target(graph, a);
                const double probability = best_from_s[u] * probability_map[a];
                if(status[v] == 2 || probability <= best_from_s[v]) continue;
                best_from_s[v] = probability;
                if(status[v] == 1) {
                    heap.promote(v, probability);
                    continue;
                }
                status[v] = 1;
                heap.push({v, probability});
                visited.push_back(v);
            }
        }
        for(const arc a : melon::arcs(graph)) {
            const vertex u = melon::arc_source(graph, a);
            const double through = best_from_s[u] * probability_map[a] *
                                   best_to_t[melon::arc_target(graph, a)];
            if(through <= 0 || through < threshold) continue;
            source_s.arcs.push_back(a);
            if(option_map[a])
                source_s.improved_copies.emplace_back(
                    a, best_quality * best_from_s[u]);
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
    // the fallback's and the value returned a bound, exact at a scenario:
    // the second value returned is then false.
    std::pair<double, bool> _largest_flow(const source & source_s,
                                          const double supply,
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
                    if(in_A_s[a] &&
                       (capacities[a] == infinity ||
                        capacities[a] - flows[a] > arc_full * capacities[a]))
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
                    amount =
                        std::min(amount, flows[a] * probability_map[a] / gain);
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
        return {arrived, settled};
    }

    // The cut of the point x̂: for each source, its largest flow to t at
    // x̂ and the dual solution that comes with it. At a scenario y one
    // path takes the whole supply, so value is f_t(y). rebuild: the
    // sources have no A_s kept.
    cut separate(const std::vector<double> & x_hat, const bool rebuild) const {
        cut result{source_quality_map[t], source_quality_map[t],
                   std::vector<double>(x_hat.size(), 0.0)};
        for(const auto & [source_gain, i] : source_options_map[t]) {
            result.coefficients[i] += source_gain;
            result.value += source_gain * x_hat[i];
        }
        flow_search search(graph);
        const auto & rho = search.rho;
        source_search from_s(graph);
        source rebuilt{};
        for(const source & kept : sources) {
            if(rebuild) {
                rebuilt.s = kept.s;
                _source_arcs(rebuilt, from_s);
            }
            const source & source_s = rebuild ? rebuilt : kept;
            const vertex s = source_s.s;
            double supply = source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                supply += source_gain * x_hat[i];
            const auto [flow, settled] =
                _largest_flow(source_s, supply, x_hat, search);
            result.value += flow;
            result.settled = result.settled && settled;
            result.constant += rho[s] * source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                result.coefficients[i] += rho[s] * source_gain;
            for(const auto & [a, big_M] : source_s.improved_copies) {
                const double sigma =
                    probability_map[a] * rho[melon::arc_target(graph, a)] -
                    rho[melon::arc_source(graph, a)];
                if(sigma > 0)
                    result.coefficients[option_map[a].value()] += big_M * sigma;
            }
        }
        return result;
    }
};

}  // namespace solvers
}  // namespace gecot
