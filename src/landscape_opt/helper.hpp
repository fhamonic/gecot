#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

#include "concepts/instance.hpp"
#include "indices/eca.hpp"
#include "indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<std::vector<
    std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
computeOptionsForVertices(const I & instance) noexcept {
    std::vector<std::vector<
        std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
        vertexOptionsMap(instance.options().size());
    for(auto && u : instance.landscape().graph().vertices()) {
        for(auto && [quality_gain, option] : instance.vertex_options_map()[u]) {
            vertexOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return vertexOptionsMap;
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

template <concepts::Instance I>
double compute_solution_eca(const I & instance,
                            const typename I::Solution & solution) {
    using Landscape = typename I::Landscape;
    using QualityMap = typename Landscape::QualityMap;
    using ProbabilityMap = typename Landscape::ProbabilityMap;

    const auto vertexOptions = detail::computeOptionsForVertices(instance);
    const auto arcOptions = detail::computeOptionsForArcs(instance);

    QualityMap enhanced_qm = instance.landscape().quality_map();
    ProbabilityMap enhanced_pm = instance.landscape().probability_map();

    for(auto && option : instance.options()) {
        if(!solution[option]) continue;
        for(auto && [u, quality_gain] : vertexOptions[option])
            enhanced_qm[u] += quality_gain;
        for(auto && [a, enhanced_prob] : arcOptions[option])
            enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
    }

    return eca(instance.landscape().graph(), enhanced_qm, enhanced_pm);
}
}  // namespace detail

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP