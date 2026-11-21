#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/onaga.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/mapping.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

namespace gecot {
namespace solvers {

// How multiflow_target_block::make_groups splits the sources of a target:
// voronoi, the cells of a network Voronoi diagram on G_t around sources of
// flow far from each other; consecutive, by vertex of G_t; random, drawn at
// random, as a control
enum class source_grouping { voronoi, consecutive, random };

// The subproblem of multiflow_benders for a target t (benders.md): its graph
// G_t, each improved copy an arc of its own, for each source the arcs that
// can carry its flow to t, and the cut of a point.
// Variable is that of the master, for f_t and the f^i_t. MaxPushes is a
// parameter for the tests, which stop the flow of a source with 0 and 1.
template <typename Variable, std::size_t MaxPushes = 100000>
struct multiflow_target_block {
    using graph_t = melon::static_digraph;
    using vertex = melon::vertex_t<graph_t>;
    using arc = melon::arc_t<graph_t>;
    template <typename V>
    using vertex_map = melon::vertex_map_t<graph_t, V>;
    template <typename V>
    using arc_map = melon::arc_map_t<graph_t, V>;
    // the capacity of an arc without option
    static constexpr double unbounded = std::numeric_limits<double>::infinity();
    // the relative slack of the test of A_s against the best path
    // without option
    static constexpr double path_tie = 1e-9;
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
    // The part of a source in a cut: f_st <= constant + Σ coefficients in
    // every scenario, by option, sorted, without duplicate or 0. value is
    // the flow of the source at the point separated, or the bound of its
    // part there when its flow did not settle. Of a group: the sum of the
    // parts of its sources.
    struct source_cut {
        std::size_t source;  // in sources, or in groups
        double value = 0, constant = 0;
        std::vector<std::pair<option_t, double>> coefficients{};

        double at(auto && x) const {
            double result = constant;
            for(const auto & [i, coefficient] : coefficients)
                result += coefficient * x(i);
            return result;
        }
    };
    // f_t <= constant + Σ_i coefficients[i] x_i in every scenario; value
    // is the flow that reaches t at the point separated. When the flow
    // of a source did not settle, it is the value of the cut at that
    // point, a bound on the flow, and still the flow at a scenario.
    struct cut {
        double value = 0, constant = 0;
        std::vector<double> coefficients;
        bool settled = true;  // the flow of every source
        // asked by per_source: the part of each source, whose sum with the
        // target's own quality is the cut
        std::vector<source_cut> sources{};
    };

    // sorted by option, the duplicates summed, without 0
    static void _merge(std::vector<std::pair<option_t, double>> & terms) {
        std::ranges::sort(terms, {}, &std::pair<option_t, double>::first);
        auto out = terms.begin();
        for(auto it = terms.begin(); it != terms.end();) {
            const option_t i = it->first;
            double sum = 0;
            for(; it != terms.end() && it->first == i; ++it) sum += it->second;
            if(sum != 0) *out++ = {i, sum};
        }
        terms.erase(out, terms.end());
    }

    // w̄_s Π̄(s, t): the flow of the source in every scenario and at every
    // point is at most its best quality on its best path
    double source_flow_bound(const source & source_s) const {
        return _best_quality(source_s.s) * best_to_t[source_s.s];
    }

    // The variables of the master for the sources, when make_groups made two
    // groups or more; without, the cuts bound f_t. A source whose A_s has no
    // improved copy has the flow Π(s, t) w^x_s at every point, affine, which
    // fixed_part() writes exactly; the others are split into groups, each
    // with a variable f_tg within the sum of the bounds of its sources, that
    // receives the sum of their parts.
    struct source_group {
        Variable f{};
        std::vector<std::size_t> members;  // in sources
        double bound = 0;
    };
    std::vector<source_group> groups;
    std::vector<std::size_t> affine_sources;  // in sources

    // Splits the n sources not affine into ⌈n / group_size⌉ groups, by
    // grouping, rng for random; group_size 0: a single group. fold_affine:
    // the affine sources in no group, which needs their A_s. A single group
    // leaves no group and no affine source: the cuts on f_t are then as
    // tight.
    void make_groups(const std::size_t group_size,
                     const source_grouping grouping, const bool fold_affine,
                     std::mt19937_64 & rng) {
        groups.clear();
        affine_sources.clear();
        if(group_size == 0) return;
        std::vector<std::size_t> grouped;
        for(std::size_t k = 0; k < sources.size(); ++k)
            (fold_affine && sources[k].improved_copies.empty()
                 ? affine_sources
                 : grouped)
                .push_back(k);
        const std::size_t n = grouped.size();
        const std::size_t g_count = (n + group_size - 1) / group_size;
        if(g_count <= 1) {
            affine_sources.clear();
            return;
        }
        std::vector<std::size_t> cells(n);
        // a group per source needs no cell
        if(grouping == source_grouping::voronoi && g_count < n)
            cells = _voronoi_cells(grouped, g_count);
        else {
            if(grouping == source_grouping::random)
                std::ranges::shuffle(grouped, rng);
            for(std::size_t j = 0; j < n; ++j) cells[j] = j * g_count / n;
        }
        groups.assign(g_count, source_group{});
        for(std::size_t j = 0; j < n; ++j) {
            source_group & group = groups[cells[j]];
            group.members.push_back(grouped[j]);
            group.bound += source_flow_bound(sources[grouped[j]]);
        }
    }

    // The cell of each source of grouped, by its index there, among g_count
    // kernels on G_t without t, its arcs in both directions at their best
    // probability: the way of a vertex to a kernel is the most reliable, P,
    // then of fewest arcs, which decide between the ways of probability 1.
    // The kernels are the source of largest bound w̄_s Π̄(s, t), then each
    // time the one of largest w̄_s Π̄(s, t) (1 - P), then of most arcs to
    // its kernel, as multicut_benders picks its kernels. A source falls in
    // the cell of the kernel of its best way, ahead of the kernels after it:
    // the cells gather sources close in G_t, by ways that can go against the
    // arcs, which only approximates sources whose flows to t share arcs and
    // options. Without t, whose arcs link every source to every other. A
    // source that reaches no kernel falls in the smallest cell. Each kernel takes its cell by a
    // search from it through the vertices whose way it betters, the only
    // ones whose cell changes.
    std::vector<std::size_t> _voronoi_cells(
        const std::vector<std::size_t> & grouped,
        const std::size_t g_count) const {
        const std::size_t n = grouped.size();
        constexpr std::size_t far = std::numeric_limits<std::size_t>::max();
        // the way of each vertex to its kernel, P and its arcs, and its
        // cell, g_count without
        auto to_kernel = melon::create_vertex_map<double>(graph, 0.0);
        auto hops = melon::create_vertex_map<std::size_t>(graph, far);
        auto cell = melon::create_vertex_map<std::size_t>(graph, g_count);
        std::vector<double> bounds(n);
        std::vector<char> is_kernel(n, false);
        auto key = [&](const std::size_t j) {
            const vertex s = sources[grouped[j]].s;
            return std::make_pair(bounds[j] * (1 - to_kernel[s]), hops[s]);
        };
        // the sources by their key when pushed: keys only decrease, so the
        // first entry popped at the key of its source has the largest
        std::priority_queue<
            std::pair<std::pair<double, std::size_t>, std::size_t>>
            candidates;
        for(std::size_t j = 0; j < n; ++j) {
            bounds[j] = source_flow_bound(sources[grouped[j]]);
            candidates.emplace(key(j), j);
        }
        // the best way first: P, then far minus its arcs
        std::priority_queue<std::tuple<double, std::size_t, vertex>> heap;
        auto reach = [&](const vertex v, const double probability,
                         const std::size_t arcs, const std::size_t c) {
            if(v == t || probability < to_kernel[v] ||
               (probability == to_kernel[v] && arcs >= hops[v]))
                return;
            to_kernel[v] = probability;
            hops[v] = arcs;
            cell[v] = c;
            heap.emplace(probability, far - arcs, v);
        };
        for(std::size_t c = 0; c < g_count; ++c) {
            // g_count <= n: a source not a kernel remains in candidates
            std::size_t kernel = 0;
            for(;;) {
                const auto [pushed_key, j] = candidates.top();
                candidates.pop();
                if(is_kernel[j]) continue;
                if(const auto current = key(j); current < pushed_key) {
                    candidates.emplace(current, j);
                    continue;
                }
                kernel = j;
                break;
            }
            is_kernel[kernel] = true;
            // its own cell, even at a way of probability 1 to a kernel
            const vertex k = sources[grouped[kernel]].s;
            to_kernel[k] = 1;
            hops[k] = 0;
            cell[k] = c;
            heap.emplace(1.0, far, k);
            while(!heap.empty()) {
                const auto [probability, far_arcs, u] = heap.top();
                heap.pop();
                const std::size_t arcs = far - far_arcs;
                // bettered since
                if(probability != to_kernel[u] || arcs != hops[u]) continue;
                for(const arc a : melon::out_arcs(graph, u))
                    reach(melon::arc_target(graph, a),
                          probability * probability_map[a], arcs + 1, c);
                for(const arc a : melon::in_arcs(graph, u))
                    reach(melon::arc_source(graph, a),
                          probability * probability_map[a], arcs + 1, c);
            }
        }
        std::vector<std::size_t> cells(n), sizes(g_count, 0);
        for(std::size_t j = 0; j < n; ++j) {
            cells[j] = cell[sources[grouped[j]].s];
            if(cells[j] < g_count) ++sizes[cells[j]];
        }
        for(std::size_t j = 0; j < n; ++j)
            if(cells[j] == g_count) {
                cells[j] = static_cast<std::size_t>(
                    std::ranges::min_element(sizes) - sizes.begin());
                ++sizes[cells[j]];
            }
        return cells;
    }

    // The fixed part of f_t: the target's own quality, w_t + Σ_i w^i_t x_i,
    // and the flow Π(s, t) w^x_s of each affine source, its constant and its
    // terms, merged
    std::pair<double, std::vector<std::pair<option_t, double>>> fixed_part()
        const {
        double constant = source_quality_map[t];
        std::vector<std::pair<option_t, double>> terms;
        for(const auto & [source_gain, i] : source_options_map[t])
            terms.emplace_back(i, source_gain);
        for(const std::size_t k : affine_sources) {
            const vertex s = sources[k].s;
            constant += base_to_t[s] * source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                terms.emplace_back(i, base_to_t[s] * source_gain);
        }
        _merge(terms);
        return {constant, std::move(terms)};
    }

    // the part of the group g in the cut c, separated per_source
    source_cut group_part(const std::size_t g, const cut & c) const {
        source_cut part{g};
        for(const std::size_t k : groups[g].members) {
            const source_cut & source_part = c.sources[k];
            part.value += source_part.value;
            part.constant += source_part.constant;
            part.coefficients.insert(part.coefficients.end(),
                                     source_part.coefficients.begin(),
                                     source_part.coefficients.end());
        }
        _merge(part.coefficients);
        return part;
    }

    case_id_t case_id;
    double target_quality;  // q_t
    double flow_bound;      // M^t_t, the bound of f_t
    // f_t, and the f^i_t of target_quality_gains: the block leaves them
    // unset, the master sets them
    Variable f_t{};
    // with the constant M^t_i of the target quality gain of i, and f^i_t
    std::vector<std::tuple<double, option_t, double, Variable>>
        target_quality_gains;
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
                target_quality_gains.emplace_back(
                    target_gain, i, G_t.target_big_M(i), Variable{});
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

    // What the searches of a separation reuse from a source to the next:
    // the capacities, 0 outside A_s, and the algorithm that reads them.
    // Not movable: algorithm refers to capacities, and a moved flow_search
    // would read those of the object it was moved from.
    struct flow_search {
        using algorithm_t =
            melon::onaga<melon::views::graph_all_t<const graph_t &>,
                         melon::maps::mapping_all_t<const arc_map<double> &>,
                         melon::maps::mapping_all_t<const arc_map<double> &>>;
        arc_map<double> capacities;
        algorithm_t algorithm;

        flow_search(const graph_t & graph, const arc_map<double> & gains,
                    const vertex_map<double> & open_to_t)
            : capacities(melon::create_arc_map<double>(graph, 0.0))
            , algorithm(graph, gains, std::as_const(capacities)) {
            algorithm.set_start_labels(open_to_t);
        }
        flow_search(flow_search &&) = delete;
    };

    // The largest flow of the source to t at x̂: returns what reaches t,
    // and leaves in search.algorithm.potential() an optimal dual solution.
    // Past max_pushes the flow is not the largest: the potentials are still
    // feasible, the value returned is that of their cut at x̂, a bound on the
    // flow, and the second value returned is false.
    std::pair<double, bool> _largest_flow(const source & source_s,
                                          const double supply,
                                          const auto & x_hat,
                                          flow_search & search) const {
        auto & [capacities, algorithm] = search;
        const vertex s = source_s.s;
        for(const arc a : source_s.arcs) capacities[a] = unbounded;
        for(const auto & [a, big_M] : source_s.improved_copies)
            capacities[a] = big_M * x_hat[option_map[a].value()];
        algorithm.reset().set_source(s, supply).set_target(t);
        for(std::size_t pushes = 0;
            pushes < max_pushes && !algorithm.finished(); ++pushes)
            algorithm.advance();
        algorithm.compute_potentials();
        for(const arc a : source_s.arcs) capacities[a] = 0;
        // the flow is the largest once the supply is sent or s no longer
        // reaches t
        const double rho_s = algorithm.potential(s);
        if(algorithm.finished() || rho_s <= 0)
            return {algorithm.flow_value(), true};
        return {algorithm.flow_value() + rho_s * algorithm.remaining_supply(),
                false};
    }

    // The best gains to t over the arcs open at x̂
    vertex_map<double> _open_to_t(const auto & x_hat) const {
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
        return open_to_t;
    }

    // the quality of the source s at x̂
    double _supply(const vertex s, const auto & x_hat) const {
        double supply = source_quality_map[s];
        for(const auto & [source_gain, i] : source_options_map[s])
            supply += source_gain * x_hat[i];
        return supply;
    }

    // f_t(y) at a scenario y, the value of separate(y) without its cut: each
    // source sends its quality along a best path to t, which lies in its
    // A_s, so the labels of one search from t over the arcs open at y give
    // every flow.
    // on_source(k, flow) receives the flow of each source k of sources.
    double flow_at(const melon::mapping<option_t> auto & y,
                   auto && on_source) const {
        const auto open_to_t = _open_to_t(y);
        double value = _supply(t, y);
        for(std::size_t k = 0; k < sources.size(); ++k) {
            const double flow =
                _supply(sources[k].s, y) * open_to_t[sources[k].s];
            on_source(k, flow);
            value += flow;
        }
        return value;
    }
    double flow_at(const melon::mapping<option_t> auto & y) const {
        return flow_at(y, [](std::size_t, double) {});
    }

    // The cut of the point x̂: for each source, its largest flow to t at
    // x̂ and the dual solution that comes with it. At a scenario y one
    // path takes the whole supply, so value is f_t(y). rebuild: the
    // sources have no A_s kept. x̂ has a value in [0, 1] for each option,
    // x_hat.size() of them. per_source: the cut also gives the part of each
    // source.
    cut separate(const melon::mapping<option_t> auto & x_hat,
                 const bool rebuild, const bool per_source = false) const {
        cut result{source_quality_map[t], source_quality_map[t],
                   std::vector<double>(x_hat.size(), 0.0)};
        for(const auto & [source_gain, i] : source_options_map[t]) {
            result.coefficients[i] += source_gain;
            result.value += source_gain * x_hat[i];
        }
        // One search for all the sources: the labels of t over every arc
        // open at x̂ are feasible potentials for the first residual graph
        // of each source, whose arcs are among these.
        const auto open_to_t = _open_to_t(x_hat);
        flow_search search(graph, probability_map, open_to_t);
        const auto & algorithm = search.algorithm;
        std::optional<source_search> from_s;
        if(rebuild) from_s.emplace(graph);
        source rebuilt{};
        if(per_source) result.sources.reserve(sources.size());
        for(std::size_t k = 0; k < sources.size(); ++k) {
            const source & kept = sources[k];
            if(rebuild) {
                rebuilt.s = kept.s;
                _source_arcs(rebuilt, *from_s);
            }
            const source & source_s = rebuild ? rebuilt : kept;
            const vertex s = source_s.s;
            const auto [flow, settled] =
                _largest_flow(source_s, _supply(s, x_hat), x_hat, search);
            result.value += flow;
            result.settled = result.settled && settled;
            const double rho_s = algorithm.potential(s);
            result.constant += rho_s * source_quality_map[s];
            for(const auto & [source_gain, i] : source_options_map[s])
                result.coefficients[i] += rho_s * source_gain;
            // a term of 0 where σ is not positive, without a branch
            for(const auto & [a, big_M] : source_s.improved_copies) {
                const double sigma =
                    probability_map[a] *
                        algorithm.potential(melon::arc_target(graph, a)) -
                    algorithm.potential(melon::arc_source(graph, a));
                result.coefficients[option_map[a].value()] +=
                    big_M * std::max(sigma, 0.0);
            }
            if(!per_source) continue;
            // the same terms, for this source alone
            source_cut & part = result.sources.emplace_back(
                k, flow, rho_s * source_quality_map[s]);
            for(const auto & [source_gain, i] : source_options_map[s])
                part.coefficients.emplace_back(i, rho_s * source_gain);
            for(const auto & [a, big_M] : source_s.improved_copies) {
                const double sigma =
                    probability_map[a] *
                        algorithm.potential(melon::arc_target(graph, a)) -
                    algorithm.potential(melon::arc_source(graph, a));
                if(sigma > 0)
                    part.coefficients.emplace_back(option_map[a].value(),
                                                   big_M * sigma);
            }
            _merge(part.coefficients);
        }
        return result;
    }
};

}  // namespace solvers
}  // namespace gecot
