#ifndef LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"

#include "landscape_opt/helper.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {
template <typename G, typename T>
struct strong_arc_default_traits {
    using semiring = melon::most_reliable_path_semiring<T>;
    template <typename CMP = std::less<std::pair<melon::vertex_t<G>, T>>>
    using heap = melon::d_ary_heap<2, melon::vertex_t<G>, T, CMP,
                                   melon::vertex_map_t<G, std::size_t>>;

    static constexpr bool strictly_strong = false;
    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};
template <typename G, typename T>
struct useless_arc_default_traits {
    using semiring = melon::most_reliable_path_semiring<T>;
    template <typename CMP = std::less<std::pair<melon::vertex_t<G>, T>>>
    using heap = melon::d_ary_heap<2, melon::vertex_t<G>, T, CMP,
                                   melon::vertex_map_t<G, std::size_t>>;

    static constexpr bool strictly_strong = true;
    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};

}  // namespace detail

template <concepts::InstanceCase I>
auto compute_strong_and_useless_arcs(const I & instance_case,
                                     const bool parallel = false) {
    using LS = typename I::Landscape;
    using GR = typename LS::Graph;
    using PM = typename LS::ProbabilityMap;
    using arc_t = melon::arc_t<GR>;

    const LS & landscape = instance_case.landscape();
    const GR & graph = landscape.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const PM & upper_length_map = landscape.probability_map();
    PM lower_length_map = upper_length_map;
    const auto & arc_options_map = instance_case.arc_options_map();
    for(const arc_t & a : arcs_range) {
        for(const auto & [improved_prob, option] : arc_options_map[a]) {
            lower_length_map[a] = std::max(lower_length_map[a], improved_prob);
        }
    }

    auto compute_strong_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            melon::strong_fiber<GR, PM, PM,
                                detail::strong_arc_default_traits<GR, double>>
                algo(graph, lower_length_map, upper_length_map);
            for(const auto & uv : arcs_block) {
                algo.reset().add_strong_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    strong_arcs_map[w].push_back(uv);
                }
            }
        };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_strong_arcs);
    } else {
        compute_strong_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    auto compute_useless_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            melon::strong_fiber<GR, PM, PM,
                                detail::useless_arc_default_traits<GR, double>>
                algo(graph, lower_length_map, upper_length_map);
            for(const auto & uv : arcs_block) {
                useless_arcs_map[graph.arc_source(uv)].push_back(uv);
                algo.reset().add_useless_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    useless_arcs_map[w].push_back(uv);
                }
            }
        };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_useless_arcs);
    } else {
        compute_useless_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP