#ifndef GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <fmt/core.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/competing_dijkstras.hpp"
#include "melon/algorithm/strongly_connected_components.hpp"
#include "melon/views/subgraph.hpp"

#include "gecot/helper.hpp"

#include "gecot/utils/progress_bar.hpp"

namespace gecot {
namespace detail {
template <melon::outward_incidence_graph _Graph, typename _ValueType>
struct strong_arc_default_traits {
    using semiring = melon::shortest_path_semiring<_ValueType>;
    using entry = std::pair<_ValueType, bool>;
    static bool compare_entries(const entry & e1, const entry & e2) {
        if(e1.first == e2.first) {
            return e1.second && !e2.second;
        }
        return semiring::less(e1.first, e2.first);
    }
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return compare_entries(e1, e2);
        }
    };
    using heap = melon::updatable_d_ary_heap<
        2, std::pair<melon::vertex_t<_Graph>, entry>, entry_cmp,
        melon::vertex_map_t<_Graph, std::size_t>, melon::maps::element<1>,
        melon::maps::element<0>>;

    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};
template <melon::outward_incidence_graph _Graph, typename _ValueType>
struct useless_arc_default_traits {
    using semiring = melon::shortest_path_semiring<_ValueType>;
    using entry = std::pair<_ValueType, bool>;
    static bool compare_entries(const entry & e1, const entry & e2) {
        if(e1.first == e2.first) {
            return e2.second && !e1.second;
        }
        return semiring::less(e1.first, e2.first);
    }
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return compare_entries(e1, e2);
        }
    };
    using heap = melon::updatable_d_ary_heap<
        2, std::pair<melon::vertex_t<_Graph>, entry>, entry_cmp,
        melon::vertex_map_t<_Graph, std::size_t>, melon::maps::element<1>,
        melon::maps::element<0>>;

    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};

// The lengths are the integers -log(max(p, resolution)) / log1p(resolution).
// Throws unless the longest is positive and a path through every vertex at
// this length fits in 62 bits, the rest being for the penalties of the
// constrained labelling: lengths that are all 0, or that wrap around, label
// strong arcs that are not, and the optimum is lost without a warning.
template <typename G>
void check_resolution(const G & graph, const double resolution) {
    const double max_length = -std::log(resolution) / std::log1p(resolution);
    // false for the NaN of a negative resolution too
    if(!(max_length > 0 &&
         max_length * static_cast<double>(graph.num_vertices()) < 0x1p62))
        throw std::invalid_argument(fmt::format(
            "resolution {}: must be in ]0, 1[, and not so small that the "
            "lengths of a path of {} vertices overflow",
            resolution, graph.num_vertices()));
}

// the arcs of length 0 on a cycle of such arcs. The improvable ones count: blue
// spreads on the base lengths, so through them it comes back to the arc's tail
template <typename G, typename LM>
auto zero_cycle_arcs_map(const G & graph, const LM & length_map) {
    auto component_map = melon::create_vertex_map<std::size_t>(graph);
    std::size_t num_components = 0;
    for(auto && component :
        melon::strongly_connected_components(melon::views::subgraph(
            graph, {}, [&length_map](const melon::arc_t<G> & a) {
                return length_map[a] == 0;
            }))) {
        for(auto && v : component) component_map[v] = num_components;
        ++num_components;
    }
    auto zero_cycle_arcs = melon::create_arc_map<bool>(graph, false);
    for(auto && a : melon::arcs(graph))
        zero_cycle_arcs[a] = length_map[a] == 0 &&
                             component_map[melon::arc_source(graph, a)] ==
                                 component_map[melon::arc_target(graph, a)];
    return zero_cycle_arcs;
}

template <typename GR, typename V>
struct path_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<V>;

    using heap = melon::updatable_d_ary_heap<
        4, std::pair<melon::vertex_t<GR>, V>, typename semiring::less_t,
        melon::vertex_map_t<GR, std::size_t>, melon::maps::element<1>,
        melon::maps::element<0>>;

    static constexpr bool store_paths = true;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <case_c C>
auto compute_strong_and_useless_arcs(
    const C & instance_case,
    const auto & option_predicate = [](const option_t & o) { return true; },
    double probability_resolution = 0.00000001,
    const bool all_targets = false) {
    using graph_t = case_graph_t<C>;
    using arc_t = melon::arc_t<graph_t>;
    using log_probability_t = uint64_t;

    detail::check_resolution(instance_case.graph(), probability_resolution);
    spdlog::stopwatch prep_sw;
    spdlog::info("Preprocessing the '{}' graph", instance_case.name());

    auto prob_to_length =
        [probability_resolution](const double p) -> log_probability_t {
        return static_cast<log_probability_t>(
            -std::log(std::max(p, probability_resolution)) /
            std::log1p(probability_resolution));
    };
    const auto & graph = instance_case.graph();
    const auto & target_quality_map = instance_case.target_quality_map();

    auto useless_vertices_map = melon::create_vertex_map<bool>(graph);
    for(const auto & v : melon::vertices(graph)) {
        useless_vertices_map[v] =
            !all_targets &&
            (target_quality_map[v] == 0 &&
             std::ranges::none_of(instance_case.vertex_options_map()[v],
                                  [](auto && e) {
                                      auto && [sqg, tqg, o] = e;
                                      return tqg > 0;
                                  }));
    }

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();

    auto base_length_map = melon::create_arc_map<log_probability_t>(graph);
    auto improved_length_map = melon::create_arc_map<log_probability_t>(graph);

    const auto & arc_options_map = instance_case.arc_options_map();
    for(const arc_t & a : arcs_range) {
        improved_length_map[a] = base_length_map[a] =
            prob_to_length(base_probability_map[a]);
        for(const auto & [improved_prob, option] : arc_options_map[a]) {
            if(!option_predicate(option)) continue;
            improved_length_map[a] =
                std::min(improved_length_map[a], prob_to_length(improved_prob));
        }
    }
    const auto zero_cycle_arcs_map =
        detail::zero_cycle_arcs_map(graph, base_length_map);

    {
        progress_bar<spdlog::level::info, 64> pb(2 * arcs_range.size());

        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                    arcs_block) {
                arc_t uv;
                auto sgraph = melon::views::subgraph(
                    graph, {},
                    [&uv](const arc_t & a) -> bool { return a != uv; });
                auto algo = melon::competing_dijkstras(
                    detail::strong_arc_default_traits<graph_t,
                                                      log_probability_t>{},
                    sgraph, base_length_map, improved_length_map);
                for(const auto & a : arcs_block) {
                    uv = a;
                    auto && u = melon::arc_source(graph, uv);
                    auto && v = melon::arc_target(graph, uv);
                    algo.reset();
                    algo.add_red_source(u);
                    // blue wins ties, enough since a vertex contracts a single
                    // strong arc, but not on a zero-length cycle: its arcs
                    // would all be strong and contract onto each other
                    algo.add_blue_source(
                        v, base_length_map[uv] + zero_cycle_arcs_map[uv]);
                    for(const auto & [w, w_dist] : algo) {
                        if(useless_vertices_map[w]) continue;
                        strong_arcs_map[w].push_back(uv);
                    }
                    pb.tick();
                }
            });

        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                    arcs_block) {
                arc_t uv;
                auto sgraph = melon::views::subgraph(
                    graph, {},
                    [&uv](const arc_t & a) -> bool { return a != uv; });
                // red wins ties: two arcs tied for a target would both be
                // useless, and removed together
                auto algo = melon::competing_dijkstras(
                    detail::useless_arc_default_traits<graph_t,
                                                       log_probability_t>{},
                    sgraph, base_length_map, improved_length_map);
                for(const auto & a : arcs_block) {
                    uv = a;
                    auto && u = melon::arc_source(graph, uv);
                    auto && v = melon::arc_target(graph, uv);
                    algo.reset();
                    algo.add_blue_source(u);
                    algo.add_red_source(v, improved_length_map[uv]);
                    for(const auto & [w, w_dist] : algo) {
                        if(useless_vertices_map[w]) continue;
                        useless_arcs_map[w].push_back(uv);
                    }
                    pb.tick();
                }
            });
    }
    // the parallel loops push in thread order: sorted, the contracted graphs
    // and the models built on them are the same from run to run
    for(auto && v : melon::vertices(graph)) {
        std::sort(strong_arcs_map[v].begin(), strong_arcs_map[v].end());
        std::sort(useless_arcs_map[v].begin(), useless_arcs_map[v].end());
    }

    if(spdlog::should_log(spdlog::level::debug)) {
        std::size_t num_strong, num_useless, num_sinks;
        num_strong = num_useless = num_sinks = 0;
        for(auto && v : melon::vertices(graph)) {
            if(useless_vertices_map[v]) continue;
            num_strong += strong_arcs_map[v].size();
            num_useless += useless_arcs_map[v].size();
            ++num_sinks;
        }
        spdlog::debug(
            "  {:>10.2f} strong arcs on average",
            static_cast<double>(num_strong) / static_cast<double>(num_sinks));
        spdlog::debug(
            "  {:>10.2f} useless arcs on average",
            static_cast<double>(num_useless) / static_cast<double>(num_sinks));
        spdlog::debug("          (took {} ms)",
                      std::chrono::duration_cast<std::chrono::milliseconds>(
                          prep_sw.elapsed())
                          .count());
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP