#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <utility>
#include <vector>

#include "concepts/instance.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<
    std::vector<std::pair<typename I::Landscape::Graph::vertex_t, double>>>
computeOptionsForNodes(const I & instance) noexcept {
    std::vector<
        std::vector<std::pair<typename I::Landscape::Graph::vertex_t, double>>>
        nodeOptionsMap(instance.options().size());
    for(auto && u : instance.landscape().graph().vertices()) {
        for(auto && [quality_gain, option] : instance.node_options_map()[u]) {
            nodeOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return nodeOptionsMap;
}

template <concepts::Instance I>
std::vector<std::vector<std::pair<typename I::Landscape::Graph::arc_t, double>>>
computeOptionsForArcs(const I & instance) noexcept {
    std::vector<
        std::vector<std::pair<typename I::Landscape::Graph::arc_t, double>>>
        arcOptionsMap(instance.options().size());
    for(auto && a : instance.landscape().graph().arcs())
        for(auto && [enhanced_prob, option] : instance.arc_options_map()[a])
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    return arcOptionsMap;
}

// template <concepts::Instance I>
// typename Instance::Option compute_best_option(instance, taken_options ou
// solutionc)

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace detail
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP