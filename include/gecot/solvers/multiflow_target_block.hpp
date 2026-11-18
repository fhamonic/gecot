#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
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
// MaxPushes is a parameter for the tests, which stop the flow of a source
// with 0 and 1.
template <std::size_t MaxPushes = 100000>
struct multiflow_target_block {
    using graph_t = melon::static_digraph;
    using vertex = melon::vertex_t<graph_t>;
    using arc = melon::arc_t<graph_t>;
    template <typename V>
    using vertex_map = melon::vertex_map_t<graph_t, V>;
    template <typename V>
    using arc_map = melon::arc_map_t<graph_t, V>;
    // The capacity of an arc without option. Finite, so that the test of
    // an open arc needs no case: inf - flow > arc_full inf does not hold.
    static constexpr double unbounded = std::numeric_limits<double>::max();
    // the relative slack of the test of A_s against the best path
    // without option
    static constexpr double path_tie = 1e-9;
    // In the search of a flow, relative: an arc is full within arc_full
    // of its capacity, and a flow under no_flow times the supply is none.
    static constexpr double arc_full = 1e-11, no_flow = 1e-12;
    // Past this many pushes, the flow of a source stops where it is.
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
    // of a source did not settle, it is the value of the cut at that
    // point, a bound on the flow, and still the flow at a scenario.
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
        // A_s and its improved copies before the source gets them at their
        // size: arrays of one more than they can hold, filled by
        // `*end = x; end += keep;`, without a branch on a test that goes
        // either way
        std::unique_ptr<arc[]> arcs;
        std::unique_ptr<std::pair<arc, double>[]> improved_copies;
        detail::pc_num_dijkstra_traits<graph_t, double>::heap heap;

        explicit source_search(const graph_t & graph)
            : best_from_s(melon::create_vertex_map<double>(graph, 0.0))
            , status(melon::create_vertex_map<char>(graph, 0))
            , arcs(std::make_unique_for_overwrite<arc[]>(
                  melon::num_arcs(graph) + 1))
            , improved_copies(
                  std::make_unique_for_overwrite<std::pair<arc, double>[]>(
                      melon::num_arcs(graph) + 1))
            , heap(std::greater<double>{},
                   melon::create_vertex_map<std::size_t>(graph)) {}
    };

    // An arc a = (u, v) is in A_s if the best path of s through it, with
    // every option, is as reliable as its best path without option: a
    // most reliable path of s is at least that reliable in every
    // scenario.
    void _source_arcs(source & source_s, source_search & search) const {
        auto & [best_from_s, status, visited, arcs, improved_copies, heap] =
            search;
        const vertex s = source_s.s;
        const double best_quality = _best_quality(s);
        const double threshold = base_to_t[s] * (1 - path_tie);
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
        arc * arcs_end = arcs.get();
        for(const arc a : melon::arcs(graph)) {
            const double through = best_from_s[melon::arc_source(graph, a)] *
                                   probability_map[a] *
                                   best_to_t[melon::arc_target(graph, a)];
            *arcs_end = a;
            arcs_end += !((through <= 0) | (through < threshold));
        }
        std::pair<arc, double> * copies_end = improved_copies.get();
        for(const arc a : std::span(arcs.get(), arcs_end)) {
            *copies_end = {
                a, best_quality * best_from_s[melon::arc_source(graph, a)]};
            copies_end += option_map[a].has_value();
        }
        source_s.arcs.assign(arcs.get(), arcs_end);
        source_s.improved_copies.assign(improved_copies.get(), copies_end);
    }

    // What the searches of a separation reuse from a source to the next.
    struct flow_search {
        // Both 0 outside A_s, which closes its arcs forward and backward;
        // the capacity of an arc of A_s without option is unbounded.
        arc_map<double> capacities;
        arc_map<double> flows;
        // Feasible potentials of the residual graph of the source: 1 at t,
        // and p_u >= g p_v on each of its arcs, of gain g from u to v.
        // Between two sources, the labels of t over every arc open at x̂.
        vertex_map<double> potentials;
        // the labels of the last residual graph, 0 outside reached
        vertex_map<double> rho;
        // of a search from the source: the reduced gain of its best path
        // to a vertex, 0 outside visited, and the arc that path ends with,
        // taken forward or not
        vertex_map<double> gain_from_s;
        vertex_map<std::pair<arc, bool>> last_arc;
        // 0 outside the heap, 1 in it, 2 settled
        vertex_map<char> status;
        // Whether flow left a vertex or entered it since the source began:
        // only its arcs can then be taken backward. And whether the vertex
        // is in changed.
        static constexpr char left = 1, entered = 2, listed = 4;
        vertex_map<char> marks;
        // the vertices with a label; those that the last search from the
        // source went to; those whose potentials or marks are the source's
        std::vector<vertex> reached, visited, changed;
        std::vector<std::pair<arc, bool>> path;
        detail::pc_num_dijkstra_traits<graph_t, double>::heap heap;

        flow_search(const graph_t & graph, const vertex_map<double> & open_to_t)
            : capacities(melon::create_arc_map<double>(graph, 0.0))
            , flows(melon::create_arc_map<double>(graph, 0.0))
            , potentials(open_to_t)
            , rho(melon::create_vertex_map<double>(graph, 0.0))
            , gain_from_s(melon::create_vertex_map<double>(graph, 0.0))
            , last_arc(melon::create_vertex_map<std::pair<arc, bool>>(graph))
            , status(melon::create_vertex_map<char>(graph, 0))
            , marks(melon::create_vertex_map<char>(graph, 0))
            , heap(std::greater<double>{},
                   melon::create_vertex_map<std::size_t>(graph)) {}
    };

    // The largest flow of the source to t at x̂, by highest-gain
    // augmenting paths: returns what reaches t, and leaves in search.rho
    // an optimal dual solution, ρ_u being the gain of the best path from
    // u to t in the last residual graph. open_to_t: the labels of t over
    // every arc open at x̂. Past max_pushes the flow is not the largest:
    // ρ is still feasible, the value returned is that of its cut at x̂, a
    // bound on the flow, and the second value returned is false.
    std::pair<double, bool> _largest_flow(const source & source_s,
                                          const double supply,
                                          const std::vector<double> & x_hat,
                                          const vertex_map<double> & open_to_t,
                                          flow_search & search) const {
        auto & [capacities, flows, potentials, rho, gain_from_s, last_arc,
                status, marks, reached, visited, changed, path, heap] = search;
        const vertex s = source_s.s;
        for(const vertex v : reached) {
            rho[v] = 0;
            status[v] = 0;
        }
        for(const arc a : source_s.arcs) capacities[a] = unbounded;
        for(const auto & [a, big_M] : source_s.improved_copies)
            capacities[a] = big_M * x_hat[option_map[a].value()];
        const double negligible = no_flow * supply;
        const auto open = [&](const arc a) {
            return capacities[a] - flows[a] > arc_full * capacities[a];
        };
        // The best residual path from s to t, in path: Dijkstra from s on
        // the gains reduced by the potentials, g p_v / p_u <= 1, stopped
        // once t is settled. The potentials make it go toward t: it
        // settles the vertices that a path at least as good as the best
        // one could cross if their potentials still held. Those vertices
        // have a reduced gain D_u >= D_t, and their potentials become
        // p_u D_t / D_u: feasible again, and tight on the path, so that the
        // arcs that its push opens backward have a reduced gain of 1.
        const auto path_from_s = [&]() -> bool {
            if(!(potentials[s] > 0)) return false;
            gain_from_s[s] = 1;
            status[s] = 1;
            visited.assign(1, s);
            heap.push({s, 1.0});
            while(!heap.empty()) {
                const vertex u = heap.top().first;
                heap.pop();
                status[u] = 2;
                if(!(marks[u] & flow_search::listed)) {
                    marks[u] |= flow_search::listed;
                    changed.push_back(u);
                }
                if(u == t) break;
                const double scale = gain_from_s[u] / potentials[u];
                auto improve = [&](const vertex v, const double gain,
                                   const arc a, const bool forward) {
                    if(status[v] == 2 || gain <= gain_from_s[v]) return;
                    gain_from_s[v] = gain;
                    last_arc[v] = {a, forward};
                    if(status[v] == 1) {
                        heap.promote(v, gain);
                        return;
                    }
                    status[v] = 1;
                    heap.push({v, gain});
                    visited.push_back(v);
                };
                for(const arc a : melon::out_arcs(graph, u))
                    if(open(a)) {
                        const vertex v = melon::arc_target(graph, a);
                        improve(v, scale * probability_map[a] * potentials[v],
                                a, true);
                    }
                if(!(marks[u] & flow_search::entered)) continue;
                for(const arc a : melon::in_arcs(graph, u))
                    if(flows[a] > negligible) {
                        const vertex w = melon::arc_source(graph, a);
                        improve(w, scale * potentials[w] / probability_map[a],
                                a, false);
                    }
            }
            heap.clear();
            const bool found = status[t] == 2;
            if(found) {
                for(const vertex u : visited)
                    if(status[u] == 2)
                        potentials[u] *= gain_from_s[t] / gain_from_s[u];
                path.clear();
                for(vertex v = t; v != s;) {
                    const auto [a, forward] = last_arc[v];
                    path.emplace_back(a, forward);
                    v = forward ? melon::arc_source(graph, a)
                                : melon::arc_target(graph, a);
                }
                std::ranges::reverse(path);
            }
            for(const vertex v : visited) {
                gain_from_s[v] = 0;
                status[v] = 0;
            }
            return found;
        };
        double remaining = supply, arrived = 0;
        for(std::size_t pushes = 0;
            remaining > negligible && pushes < max_pushes && path_from_s();
            ++pushes) {
            // the most the path takes, in units leaving s
            double amount = remaining, gain = 1;
            for(const auto & [a, forward] : path) {
                if(forward) {
                    amount =
                        std::min(amount, (capacities[a] - flows[a]) / gain);
                    gain *= probability_map[a];
                } else {
                    amount =
                        std::min(amount, flows[a] * probability_map[a] / gain);
                    gain /= probability_map[a];
                }
            }
            remaining -= amount;
            for(const auto & [a, forward] : path) {
                if(forward) {
                    flows[a] += amount;
                    marks[melon::arc_source(graph, a)] |= flow_search::left;
                    marks[melon::arc_target(graph, a)] |= flow_search::entered;
                    amount *= probability_map[a];
                } else {
                    amount /= probability_map[a];
                    flows[a] -= amount;
                }
            }
            arrived += amount;
        }
        // The labels of the last residual graph: Dijkstra from t, the
        // highest label first. A residual arc taken backward has a gain
        // above 1, so the order is that of the labels reduced by the
        // potentials, ρ_u / p_u.
        rho[t] = 1;
        reached.assign(1, t);
        heap.push({t, 1.0});
        while(!heap.empty()) {
            const vertex v = heap.top().first;
            heap.pop();
            status[v] = 2;
            auto improve = [&](const vertex u, const double gain) {
                if(status[u] == 2 || gain <= rho[u]) return;
                rho[u] = gain;
                const double key = gain / potentials[u];
                if(status[u] == 1) {
                    heap.promote(u, key);
                    return;
                }
                status[u] = 1;
                heap.push({u, key});
                reached.push_back(u);
            };
            for(const arc a : melon::in_arcs(graph, v))
                if(open(a))
                    improve(melon::arc_source(graph, a),
                            probability_map[a] * rho[v]);
            if(!(marks[v] & flow_search::left)) continue;
            for(const arc a : melon::out_arcs(graph, v))
                if(flows[a] > negligible)
                    improve(melon::arc_target(graph, a),
                            rho[v] / probability_map[a]);
        }
        for(const arc a : source_s.arcs) {
            capacities[a] = 0;
            flows[a] = 0;
        }
        for(const vertex v : changed) {
            potentials[v] = open_to_t[v];
            marks[v] = 0;
        }
        changed.clear();
        // the flow is the largest once the supply is sent or s no longer
        // reaches t
        if(remaining <= negligible || rho[s] <= 0) return {arrived, true};
        return {arrived + rho[s] * remaining, false};
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
        // One search for all the sources: the labels of t over every arc
        // open at x̂ are feasible potentials for the first residual graph
        // of each source, whose arcs are among these.
        auto open_to_t = melon::create_vertex_map<double>(graph, 0.0);
        for(const auto & [u, probability] : melon::dijkstra(
                reversed_traits{}, melon::views::reverse(graph),
                melon::maps::function([&](const arc & a) {
                    return option_map[a] && !(x_hat[option_map[a].value()] > 0)
                               ? 0.0
                               : probability_map[a];
                }),
                t))
            open_to_t[u] = probability;
        flow_search search(graph, open_to_t);
        const auto & rho = search.rho;
        std::optional<source_search> from_s;
        if(rebuild) from_s.emplace(graph);
        source rebuilt{};
        for(const source & kept : sources) {
            if(rebuild) {
                rebuilt.s = kept.s;
                _source_arcs(rebuilt, *from_s);
            }
            const source & source_s = rebuild ? rebuilt : kept;
            const vertex s = source_s.s;
            double supply = source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                supply += source_gain * x_hat[i];
            const auto [flow, settled] =
                _largest_flow(source_s, supply, x_hat, open_to_t, search);
            result.value += flow;
            result.settled = result.settled && settled;
            result.constant += rho[s] * source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                result.coefficients[i] += rho[s] * source_gain;
            // a term of 0 where σ is not positive, without a branch
            for(const auto & [a, big_M] : source_s.improved_copies) {
                const double sigma =
                    probability_map[a] * rho[melon::arc_target(graph, a)] -
                    rho[melon::arc_source(graph, a)];
                result.coefficients[option_map[a].value()] +=
                    big_M * std::max(sigma, 0.0);
            }
        }
        return result;
    }
};

}  // namespace solvers
}  // namespace gecot
