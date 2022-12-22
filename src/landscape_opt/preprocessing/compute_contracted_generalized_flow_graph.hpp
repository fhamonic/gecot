#ifndef LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP

#include "melon/concepts/graph.hpp"
#include "melon/mutable_digraph.hpp"

#include "concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_contracted_generalized_flow_graph(const I & instance_case,
                                               const auto & strong_arcs_map,
                                               const auto & useless_arcs_map,
                                               const auto & original_t) {
    using Landscape = typename I::Landscape;
    using Graph = typename Landscape::Graph;
    using ProbabilityMap = typename Landscape::ProbabilityMap;
    using Option = typename I::Option;

    const Graph & original_graph = instance_case.landscape().graph();
    const ProbabilityMap & original_probability_map =
        instance_case.landscape().probability_map();

    melon::mutable_digraph graph;
    for(const auto & v : melon::vertices(original_graph)) {
        graph.create_vertex();
    }
    for(const auto & a : melon::arcs(original_graph)) {
        graph.create_arc(melon::source(original_graph, a),
                         melon::target(original_graph, a));
    }
    auto quality_map = instance_case.landscape().quality_map();
    auto vertex_options_map = instance_case.landscape().vertex_options_map();
    auto probability_map = instance_case.landscape().probability_map();
    auto arc_options_map = instance_case.landscape().arc_options_map();

    for(const auto & a : useless_arcs_map) graph.remove_arc(a);

    std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    for(const auto & uv : strong_arcs_map) {
        if(!arc_options_map[uv].empty()) continue;
        if(!graph.is_valid_arc(uv)) continue;
        const auto u = graph.source(uv);
        const auto v = graph.target(uv);
        const auto uv_prob = probability_map[uv];

        in_arcs_tmp.clear();
        std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));
        for(const auto & wu : in_arcs_tmp) {
            const auto w = graph.source(wu);
            probability_map[wu] *= uv_prob;
            for(auto & [improved_prob, option] : arc_options_map[wu])
                improved_prob *= uv_prob;
            graph.change_arc_target(wu, v);
        }

        quality_map[v] += quality_map[u];
        for(const auto & [quality_gain, option] : vertex_options_map[u])
            vertex_options_map[v].emplace_back(uv_prob * quality_gain, option);
        graph.remove_vertex(u);
    }

    return std::make_tuple(graph, quality_map, vertex_options_map,
                           probability_map, arc_options_map, original_t);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP