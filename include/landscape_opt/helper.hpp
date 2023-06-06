#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/graph.hpp"
#include "melon/container/static_map.hpp"

#include "landscape_opt/concepts/instance.hpp"
#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <case_c C>
auto computeOptionsForVertices(const C & instance_case,
                               const std::size_t nb_options) noexcept {
    melon::static_map<
        option_t,
        std::vector<std::pair<melon::vertex_t<case_graph_t<C>>, double>>>
        vertexOptionsMap(nb_options, {});

    for(auto && u : melon::vertices(instance_case.graph())) {
        const auto & vertex_options_map = instance_case.vertex_options_map();
        for(auto && [quality_gain, option] : vertex_options_map[u]) {
            vertexOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return vertexOptionsMap;
}

template <case_c C>
auto computeOptionsForArcs(const C & instance_case,
                           const std::size_t nb_options) noexcept {
    melon::static_map<
        option_t, std::vector<std::pair<melon::arc_t<case_graph_t<C>>, double>>>
        arcOptionsMap(nb_options, {});
    for(auto && a : melon::arcs(instance_case.graph())) {
        const auto & arc_options_map = instance_case.arc_options_map()[a];
        for(auto && [enhanced_prob, option] : arc_options_map[a])
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    }
    return arcOptionsMap;
}

template <instance_c I>
double compute_solution_eca(const I & instance,
                            const typename I::Solution & solution) {
    double sum = 0;
    const std::size_t nb_options = instance.nb_options();

    for(auto && instance_case : instance.cases()) {
        const auto vertexOptions =
            detail::computeOptionsForVertices(instance_case, nb_options);
        const auto arcOptions =
            detail::computeOptionsForArcs(instance_case, nb_options);

        auto enhanced_qm = instance_case.vertex_quality_map();
        auto enhanced_pm = instance_case.arc_probability_map();

        for(auto && option : instance.options()) {
            if(!solution[option]) continue;
            for(auto && [u, quality_gain] : vertexOptions[option])
                enhanced_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[option])
                enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
        }
 
        sum += eca(instance_case.graph(), enhanced_qm, enhanced_pm);
    }
    return sum;
}
}  // namespace detail

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP