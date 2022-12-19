#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include "melon/static_digraph.hpp"
#include "melon/static_digraph_builder.hpp"

#include "concepts/instance.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<
    std::vector<std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
computeOptionsForNodes(const I & instance) noexcept {
    std::vector<
        std::vector<std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
        nodeOptionsMap(instance.options().size());
    for(auto && u : instance.landscape().graph().vertices()) {
        for(auto && [quality_gain, option] : instance.node_options_map()[u]) {
            nodeOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return nodeOptionsMap;
}

template <concepts::Instance I>
std::vector<std::vector<std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
computeOptionsForArcs(const I & instance) noexcept {
    std::vector<
        std::vector<std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
        arcOptionsMap(instance.options().size());
    for(auto && a : instance.landscape().graph().arcs())
        for(auto && [enhanced_prob, option] : instance.arc_options_map()[a])
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    return arcOptionsMap;
}

}  // namespace detail

template <concepts::InstanceCase I>
auto generalized_flow_graph(const I & instance_case) {
    using Landscape = typename I::Landscape;
    using Graph = typename Landscape::Graph;
    using ProbabilityMap = typename Landscape::ProbabilityMap;
    using Option = typename I::Option;

    const Graph & original_graph = instance_case.landscape().graph();
    const ProbabilityMap & original_probability_map =
        instance_case.landscape().probability_map();

    melon::static_digraph_builder<melon::static_digraph, double,
                                  std::optional<Option>>
        builder(original_graph.nb_vertices());

    for(auto && a : original_graph.arcs()) {
        builder.add_arc(original_graph.source(a), original_graph.target(a),
                        original_probability_map[a], std::nullopt);
        for(auto && [enhanced_prob, option] :
            instance_case.arc_options_map()[a]) {
            builder.add_arc(original_graph.source(a), original_graph.target(a),
                            enhanced_prob, std::make_optional(option));
        }
    }

    auto [graph, probability_map, arc_options_map] = builder.build();

    return std::make_tuple(graph, instance_case.landscape().quality_map(),
                           instance_case.node_options_map(), probability_map,
                           arc_options_map);
}

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP