#ifndef LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP

#include <optional>
#include <tuple>

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/container/mutable_digraph.hpp"
#include "melon/graph.hpp"

#include "landscape_opt/concepts/instance.hpp"

namespace fhamonic {
namespace landscape_opt {

template <case_c C>
auto compute_contracted_generalized_flow_graph(const C & instance_case,
                                               const auto & strong_arcs,
                                               const auto & useless_arcs,
                                               const auto & original_t) {
    const auto & original_graph = instance_case.graph();
    const auto & original_quality_map = instance_case.vertex_quality_map();
    const auto & original_vertex_options_map =
        instance_case.vertex_options_map();

    melon::mutable_digraph graph;
    auto original_to_new_vertex_map =
        melon::create_vertex_map<melon::vertex_t<melon::mutable_digraph>>(
            original_graph);
    for(const auto & v : melon::vertices(original_graph)) {
        auto new_vertex = graph.create_vertex();
        original_to_new_vertex_map[v] = new_vertex;
    }
    auto quality_map = melon::create_vertex_map<double>(graph);
    auto vertex_options_map =
        melon::create_vertex_map<std::vector<std::pair<double, option_t>>>(
            graph);
    for(const auto & v : melon::vertices(original_graph)) {
        quality_map[original_to_new_vertex_map[v]] = original_quality_map[v];
        vertex_options_map[original_to_new_vertex_map[v]] =
            original_vertex_options_map[v];
    }

    auto original_to_new_arc_map =
        melon::create_arc_map<melon::arc_t<melon::mutable_digraph>>(
            original_graph);
    std::vector<std::tuple<melon::arc_t<melon::mutable_digraph>, double,
                           std::optional<option_t>>>
        added_arcs;
    const auto & original_probability_map = instance_case.arc_probability_map();

    // transform the useless arcs map
    auto arc_uselessness_map =
        melon::create_arc_map<bool>(original_graph, false);
    for(const auto & a : useless_arcs) arc_uselessness_map[a] = true;

    for(const auto & a : melon::arcs(original_graph)) {
        if(arc_uselessness_map[a]) continue;  // filter the useless arcs
        auto new_arc_source =
            original_to_new_vertex_map[melon::arc_source(original_graph, a)];
        auto new_arc_target =
            original_to_new_vertex_map[melon::arc_target(original_graph, a)];
        auto new_arc = graph.create_arc(new_arc_source, new_arc_target);
        original_to_new_arc_map[a] = new_arc;
        added_arcs.emplace_back(new_arc, original_probability_map[a],
                                std::nullopt);
        for(auto && [enhanced_prob, option] :
            instance_case.arc_options_map()[a]) {
            added_arcs.emplace_back(
                graph.create_arc(new_arc_source, new_arc_target), enhanced_prob,
                std::make_optional(option));
        }
    }
    auto probability_map = melon::create_arc_map<double>(graph);
    auto arc_option_map = melon::create_arc_map<std::optional<option_t>>(graph);
    for(auto && [a, prob, option] : added_arcs) {
        probability_map[a] = prob;
        arc_option_map[a] = option;
    }

    // contract strong arcs
    std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    for(const auto & original_uv : strong_arcs) {
        if(!instance_case.arc_options_map()[original_uv].empty()) continue;
        const auto uv = original_to_new_arc_map[original_uv];
        if(!graph.is_valid_arc(uv)) continue;
        const auto u = melon::arc_source(graph, uv);
        const auto v = melon::arc_target(graph, uv);
        const auto uv_prob = probability_map[uv];

        in_arcs_tmp.resize(0);
        std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));
        for(const auto & wu : in_arcs_tmp) {
            probability_map[wu] *= uv_prob;
            graph.change_arc_target(wu, v);
        }

        quality_map[v] += uv_prob * quality_map[u];
        for(const auto & [quality_gain, option] : vertex_options_map[u])
            vertex_options_map[v].emplace_back(uv_prob * quality_gain, option);
        graph.remove_vertex(u);
    }

    // remove vertices that cannot be traversed by flow
    auto bfs = melon::breadth_first_search(graph);
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

    return std::make_tuple(graph, quality_map, vertex_options_map, arc_no_map,
                           probability_map, arc_option_map,
                           original_to_new_vertex_map[original_t]);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_CONTRACTED_GENERALIZED_FLOW_GRAPH_HPP