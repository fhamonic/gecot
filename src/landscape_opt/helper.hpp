#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/concepts/graph.hpp"
#include "melon/static_digraph.hpp"
#include "melon/static_digraph_builder.hpp"

#include "concepts/instance.hpp"
#include "indices/eca.hpp"
#include "indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<std::vector<
    std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
computeOptionsForNodes(const I & instance) noexcept {
    std::vector<std::vector<
        std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
        nodeOptionsMap(instance.options().size());
    for(auto && u : instance.landscape().graph().vertices()) {
        for(auto && [quality_gain, option] : instance.vertex_options_map()[u]) {
            nodeOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return nodeOptionsMap;
}

template <concepts::Instance I>
std::vector<
    std::vector<std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
computeOptionsForArcs(const I & instance) noexcept {
    std::vector<std::vector<
        std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
        arcOptionsMap(instance.options().size());
    for(auto && a : instance.landscape().graph().arcs())
        for(auto && [enhanced_prob, option] : instance.arc_options_map()[a])
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    return arcOptionsMap;
}

}  // namespace detail

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP