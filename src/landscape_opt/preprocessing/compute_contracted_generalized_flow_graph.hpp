#ifndef LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP

#include <optional>
#include <tuple>

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/container/mutable_digraph.hpp"
#include "melon/graph.hpp"

#include "concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_contracted_generalized_flow_graph(const I & instance_case,
                                               const auto & strong_arcs_map,
                                               const auto & useless_arcs_map,
                                               const auto & original_t) {
    using Landscape = typename I::Landscape;
    using OrigGraph = typename Landscape::Graph;
    using Option = typename I::Option;
    using Graph = melon::mutable_digraph;

    const OrigGraph & original_graph = instance_case.landscape().graph();
    Graph graph;
    for([[maybe_unused]] const auto & v : melon::vertices(original_graph)) {
        std::ignore = graph.create_vertex();
    }
    auto quality_map = instance_case.landscape().quality_map();
    auto vertex_options_map = instance_case.vertex_options_map();

    auto original_to_new_arc_map =
        melon::create_arc_map<melon::arc_t<Graph>>(original_graph);
    std::vector<std::tuple<melon::arc_t<Graph>, double, std::optional<Option>>>
        added_arcs;
    const auto & original_probability_map =
        instance_case.landscape().probability_map();
    for(const auto & a : melon::arcs(original_graph)) {
        auto new_arc = graph.create_arc(melon::arc_source(original_graph, a),
                                        melon::arc_target(original_graph, a));
        original_to_new_arc_map[a] = new_arc;
        added_arcs.emplace_back(new_arc, original_probability_map[a],
                                std::nullopt);
    }
    for(auto && a : melon::arcs(original_graph)) {
        for(auto && [enhanced_prob, option] :
            instance_case.arc_options_map()[a]) {
            added_arcs.emplace_back(
                graph.create_arc(melon::arc_source(original_graph, a),
                                 melon::arc_target(original_graph, a)),
                enhanced_prob, std::make_optional(option));
        }
    }
    auto probability_map = melon::create_arc_map<double>(graph);
    auto arc_option_map = melon::create_arc_map<std::optional<Option>>(graph);
    for(auto && [a, prob, option] : added_arcs) {
        probability_map[a] = prob;
        arc_option_map[a] = option;
    }

    // remove useless arcs
    for(const auto & a : useless_arcs_map[original_t])
        graph.remove_arc(original_to_new_arc_map[a]);

    // contract strong arcs
    std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    for(const auto & original_uv : strong_arcs_map[original_t]) {
        if(!instance_case.arc_options_map()[original_uv].empty()) continue;
        auto uv = original_to_new_arc_map[original_uv];
        if(!graph.is_valid_arc(uv)) continue;
        const auto u = melon::arc_source(graph, uv);
        const auto v = melon::arc_target(graph, uv);
        const auto uv_prob = probability_map[uv];

        in_arcs_tmp.clear();
        std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));
        for(const auto & wu : in_arcs_tmp) {
            probability_map[wu] *= uv_prob;
            graph.change_arc_target(wu, v);
        }

        quality_map[v] += quality_map[u];
        for(const auto & [quality_gain, option] : vertex_options_map[u])
            vertex_options_map[v].emplace_back(uv_prob * quality_gain, option);
        graph.remove_vertex(u);
    }

    // remove vertices that cannot be traversed by flow
    melon::breadth_first_search bfs(graph);
    for(const auto & v : melon::vertices(graph)) {
        if(quality_map[v] == 0 && vertex_options_map[v].empty()) continue;
        if(bfs.reached(v)) continue;
        bfs.add_source(v).run();
    }
    std::vector<melon::vertex_t<melon::mutable_digraph>> vertices_to_delete_tmp;
    std::ranges::copy(std::ranges::views::filter(
                          melon::vertices(graph),
                          [&bfs](const auto & v) { return !bfs.reached(v); }),
                      std::back_inserter(vertices_to_delete_tmp));
    for(const auto & v : vertices_to_delete_tmp) {
        graph.remove_vertex(v);
    }

    auto arc_no_map = melon::create_arc_map<std::size_t>(graph);
    std::size_t arc_no = 0;
    for(auto && a : melon::arcs(graph)) {
        arc_no_map[a] = arc_no++;
    }

    return std::make_tuple(graph, quality_map,
                           vertex_options_map, arc_no_map, probability_map, arc_option_map,
                           original_t);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP