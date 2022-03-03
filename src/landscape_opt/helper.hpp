#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <utility>
#include <vector>

#include "concepts/instance.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<std::pair<typename I::Landscape::Graph::Node, double>>
computeOptionsForNodes(const I & instance) noexcept {
    std::vector<std::pair<typename I::Landscape::Graph::Node, double>>
        nodeOptionsMap;
    nodeOptionsMap.reserve(instance.options().size());
    for(auto && u : instance.landscape.graph().nodes())
        for(auto && [quality_gain, option] : instance.node_options_map(u))
            nodeOptionsMap[option].emplace_back(u, quality_gain);
    return nodeOptionsMap;
}

template <concepts::Instance I>
std::vector<std::pair<typename I::Landscape::Graph::Arc, double>>
computeOptionsForArcs(const I & instance) noexcept {
    std::vector<std::pair<typename I::Landscape::Graph::Arc, double>>
        arcOptionsMap;
    arcOptionsMap.reserve(instance.options().size());
    for(auto && a : instance.landscape.graph().arcs())
        for(auto && [enhanced_prob, option] : instance.arc_options_map(a))
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    return arcOptionsMap;
}

}  // namespace detail
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP