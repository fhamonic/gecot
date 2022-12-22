#ifndef LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"

#include "helper.hpp"
#include "preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_constrained_strong_and_useless_arcs(const I & instance_case,
                                     const bool parallel = false) {
    using LS = typename I::Landscape;
    using GR = typename LS::Graph;
    using PM = typename LS::ProbabilityMap;
    using arc_t = melon::arc_t<typename GR>;

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    const LS & landscape = instance_case.landscape();
    const GR & graph = landscape.graph();
    auto arcs_range = graph.arcs();
    const PM & upper_length = landscape.probability_map();
    PM lower_length = upper_length;
    const auto & arc_options_map = instance_case.arc_options_map();
    for(const arc_t & a : arcs_range) {
        for(const auto & [improved_prob, option] : arc_options_map[a]) {
            lower_length[a] = std::max(lower_length[a], improved_prob);
        }
    }

    auto compute_strong_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            melon::strong_fiber<GR, PM, PM,
                                detail::strong_arc_default_traits<GR, double>>
                algo(graph, lower_lengths, upper_lengths);
            for(const auto & uv : arcs_block) {
                algo.reset().add_strong_arc_source(uv);
                for(const auto & [u, u_dist] : algo) {
                    strong_arcs_map[u].push_back(uv);
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
            melon::strong_fiber<GR, PM, LM,
                                detail::useless_arc_default_traits<GR, double>>
                algo(graph, lower_lengths, upper_lengths);
            for(const auto & uv : arcs_block) {
                algo.reset().add_useless_arc_source(uv);
                for(const auto & [u, u_dist] : algo) {
                    useless_arcs_map[u].push_back(uv);
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