#ifndef GECOT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP
#define GECOT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP

#include <optional>

#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

#include "gecot/concepts/instance.hpp"

namespace fhamonic {
namespace gecot {

template <case_c C>
auto compute_generalized_flow_graph(const C & instance_case) {
    const auto & original_graph = instance_case.graph();
    const auto & original_probability_map = instance_case.arc_probability_map();

    melon::static_digraph_builder<melon::static_digraph, double,
                                  std::optional<option_t>>
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

    return std::make_tuple(graph, instance_case.vertex_quality_map(),
                           instance_case.vertex_options_map(), probability_map,
                           arc_options_map);
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP