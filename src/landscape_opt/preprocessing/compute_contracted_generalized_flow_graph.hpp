#ifndef LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP

#include "melon/algorithm/breadth_first_search.hpp"
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

    const Graph & original_graph = instance_case.landscape().graph();
    melon::mutable_digraph graph;
    for(const auto & v : melon::vertices(original_graph)) {
        graph.create_vertex();
    }
    for(const auto & a : melon::arcs(original_graph)) {
        graph.create_arc(melon::source(original_graph, a),
                         melon::target(original_graph, a));
    }
    auto quality_map = instance_case.landscape().quality_map();
    auto probability_map = instance_case.landscape().probability_map();
    auto vertex_options_map = instance_case.vertex_options_map();
    auto arc_options_map = instance_case.arc_options_map();

    // remove useless arcs
    for(const auto & a : useless_arcs_map[original_t]) graph.remove_arc(a);

    // contract strong arcs
    std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    for(const auto & uv : strong_arcs_map[original_t]) {
        if(!arc_options_map[uv].empty()) continue;
        if(!graph.is_valid_arc(uv)) continue;
        const auto u = graph.source(uv);
        const auto v = graph.target(uv);
        const auto uv_prob = probability_map[uv];

        in_arcs_tmp.clear();
        std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));
        for(const auto & wu : in_arcs_tmp) {
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

    // remove vertices that cannot be traversed by flow
    melon::breadth_first_search bfs(graph);
    for(const auto & v : graph.vertices()) {
        if(quality_map[v] == 0 && vertex_options_map[v].empty()) continue;
        if(bfs.reached(v)) continue;
        bfs.add_source(v).run();
    }
    std::vector<melon::vertex_t<melon::mutable_digraph>> vertices_to_delete_tmp;
    std::ranges::copy(std::ranges::views::filter(
                          graph.vertices(),
                          [&bfs](const auto & v) { return bfs.reached(v); }),
                      std::back_inserter(vertices_to_delete_tmp));
    for(const auto & v : vertices_to_delete_tmp) {
        graph.remove_vertex(v);
    }

    return std::make_tuple(graph, quality_map, vertex_options_map,
                           probability_map, arc_options_map, original_t);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP