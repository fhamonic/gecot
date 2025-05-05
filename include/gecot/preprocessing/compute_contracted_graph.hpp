#ifndef GECOT_COMPUTE_CONTRACTED_GRAPH_HPP
#define GECOT_COMPUTE_CONTRACTED_GRAPH_HPP

#include <algorithm>
#include <optional>
#include <ranges>
#include <tuple>

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/container/mutable_digraph.hpp"
#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"

namespace fhamonic {
namespace gecot {

template <case_c C>
auto compute_contracted_graph(const C & instance_case, const auto & strong_arcs,
                              const auto & useless_arcs,
                              const auto & original_t) {
    const auto & original_graph = instance_case.graph();
    const auto & original_quality_map = instance_case.vertex_quality_map();
    const auto & original_vertex_options_map =
        instance_case.vertex_options_map();

    melon::mutable_digraph graph;

    // create new vertices
    auto original_to_new_vertex_map =
        melon::create_vertex_map<melon::vertex_t<melon::mutable_digraph>>(
            original_graph);
    for(const auto & v : melon::vertices(original_graph)) {
        original_to_new_vertex_map[v] = graph.create_vertex();
    }
    auto quality_map = melon::create_vertex_map<double>(graph);
    auto vertex_options_map =
        melon::create_vertex_map<std::vector<std::pair<double, option_t>>>(
            graph);
    for(const auto & v : melon::vertices(original_graph)) {
        auto new_v = original_to_new_vertex_map[v];
        quality_map[new_v] = original_quality_map[v];
        vertex_options_map[new_v] = original_vertex_options_map[v];
    }

    auto original_to_new_arc_map = melon::create_arc_map<
        std::optional<melon::arc_t<melon::mutable_digraph>>>(original_graph);
    std::vector<std::tuple<melon::arc_t<melon::mutable_digraph>, double,
                           std::optional<option_t>>>
        added_arcs;
    const auto & original_probability_map = instance_case.arc_probability_map();
    const auto & original_arc_options_map = instance_case.arc_options_map();

    // transform the useless arcs map
    auto arc_uselessness_map =
        melon::create_arc_map<bool>(original_graph, false);
    for(const auto & a : useless_arcs) arc_uselessness_map[a] = true;
    for(const auto & a : strong_arcs) arc_uselessness_map[a] = false;

    // create new arcs
    for(const auto & a : melon::arcs(original_graph)) {
        if(arc_uselessness_map[a]) continue;  // filter the useless arcs
        auto new_source =
            original_to_new_vertex_map[melon::arc_source(original_graph, a)];
        auto new_target =
            original_to_new_vertex_map[melon::arc_target(original_graph, a)];
        auto new_arc = graph.create_arc(new_source, new_target);
        original_to_new_arc_map[a].emplace(new_arc);

        added_arcs.emplace_back(new_arc, original_probability_map[a],
                                std::nullopt);
        for(auto && [enhanced_prob, option] : original_arc_options_map[a]) {
            added_arcs.emplace_back(new_arc, enhanced_prob,
                                    std::make_optional(option));
        }
    }
    auto probability_map = melon::create_arc_map<double>(graph);
    auto arc_options_map =
        melon::create_arc_map<std::vector<std::pair<double, option_t>>>(graph,
                                                                        {});
    for(auto && [a, prob, option] : added_arcs) {
        if(!option.has_value()) {
            probability_map[a] = prob;
            continue;
        }
        arc_options_map[a].emplace_back(prob, option.value());
    }

    // contract strong arcs
    std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    for(const auto & original_uv : strong_arcs) {
        if(!original_arc_options_map[original_uv].empty())
            continue;  // if the arc can be improved
        if(!original_to_new_arc_map[original_uv].has_value())
            continue;  // should not be needed (no useless arcs are strong)
        const auto uv = original_to_new_arc_map[original_uv].value();
        if(!graph.is_valid_arc(uv))
            continue;  // if many strong arcs shared the same source
        const auto u = melon::arc_source(graph, uv);
        const auto v = melon::arc_target(graph, uv);
        const auto uv_prob = probability_map[uv];

        in_arcs_tmp.resize(0);
        std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));
        for(const auto & wu : in_arcs_tmp) {
            probability_map[wu] *= uv_prob;
            for(auto & [enhanced_prob, option] : arc_options_map[wu]) {
                enhanced_prob *= uv_prob;
            }
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

    auto improved_probability_map = probability_map;
    for(const auto & a : melon::arcs(graph)) {
        for(const auto & [improved_prob, option] : arc_options_map[a])
            improved_probability_map[a] =
                std::max(improved_probability_map[a], improved_prob);
    }

    auto t = original_to_new_vertex_map[original_t];

    auto big_M_map = compute_big_M_map(
        graph, quality_map, vertex_options_map, improved_probability_map,
        std::views::filter(melon::vertices(graph), [&](auto && u) {
            return u == t || std::ranges::any_of(
                                 melon::out_arcs(graph, u), [&](auto && a) {
                                     return arc_options_map[a].size() > 0;
                                 });
        }));

    return std::make_tuple(graph, quality_map, vertex_options_map,
                           probability_map, arc_options_map, big_M_map, t, original_t);
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_CONTRACTED_GRAPH_HPP