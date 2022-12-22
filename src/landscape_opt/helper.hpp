#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <utility>
#include <vector>

#include "concepts/instance.hpp"
#include "indices/eca.hpp"

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

template <concepts::Instance I>
double compute_solution_eca(const I & instance,
                            const typename I::Solution & solution) {
    using Landscape = typename I::Landscape;
    using QualityMap = typename Landscape::QualityMap;
    using ProbabilityMap = typename Landscape::ProbabilityMap;

    const auto nodeOptions = detail::computeOptionsForNodes(instance);
    const auto arcOptions = detail::computeOptionsForArcs(instance);

    QualityMap enhanced_qm = instance.landscape().quality_map();
    ProbabilityMap enhanced_pm = instance.landscape().probability_map();

    for(auto && option : instance.options()) {
        if(!solution[option]) continue;
        for(auto && [u, quality_gain] : nodeOptions[option])
            enhanced_qm[u] += quality_gain;
        for(auto && [a, enhanced_prob] : arcOptions[option])
            enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
    }

    return eca(instance.landscape().graph(), enhanced_qm, enhanced_pm);
}

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace detail
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP