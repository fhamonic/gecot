#ifndef LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP

#include <optional>

#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

#include "landscape_opt/concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_generalized_flow_graph(const I & instance_case) {
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

    for(auto && a : melon::arcs(original_graph)) {
        builder.add_arc(melon::arc_source(original_graph, a),
                        melon::arc_target(original_graph, a),
                        original_probability_map[a], std::nullopt);
        for(auto && [enhanced_prob, option] :
            instance_case.arc_options_map()[a]) {
            builder.add_arc(melon::arc_source(original_graph, a),
                            melon::arc_target(original_graph, a), enhanced_prob,
                            std::make_optional(option));
        }
    }

    auto [graph, probability_map, arc_options_map] = builder.build();

    return std::make_tuple(graph, instance_case.landscape().quality_map(),
                           instance_case.vertex_options_map(), probability_map,
                           arc_options_map);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP