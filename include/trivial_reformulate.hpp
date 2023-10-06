#ifndef TRIVIAL_REFORMULATE_HPP
#define TRIVIAL_REFORMULATE_HPP

#include <utility>

#include "melon/algorithm/strongly_connected_components.hpp"
#include "melon/utility/static_digraph_builder.hpp"
#include "melon/views/subgraph.hpp"

#include "landscape_opt/concepts/instance.hpp"

namespace fhamonic {
namespace landscape_opt {

template <instance_c I>
auto trivial_reformulate_instance(
    const I & instance, double budget = std::numeric_limits<double>::max()) {}

template <case_c C>
auto trivial_reformulate_case(const C & instance_case,
                              const bool parallel = false) {
    using graph_t = case_graph_t<C>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    using probability_map_t = case_probability_map_t<C>;

    const auto & original_graph = instance_case.graph();
    const auto & original_quality_map = instance_case.vertex_quality_map();
    const auto & original_probability_map = instance_case.arc_probability_map();

    instance_case.vertex_options_map();
    instance_case.arc_options_map();

    std::vector<double> quality_map;
    std::vector<std::string> names_map;
    std::vector<std::vector<vertex_t>> components;
    auto component_num_map =
        melon::create_vertex_map<std::size_t>(original_graph);

    for(auto component :
        melon::strongly_connected_components(melon::views::subgraph(
            original_graph, melon::views::true_map{},
            melon::views::map([&original_probability_map](const arc_t & a) {
                return original_probability_map[a] == 1.0;
            })))) {
        quality_map.push_back(0.0);
        names_map.emplace_back();
        components.emplace_back();
        for(const vertex_t & v : component) {
            quality_map.back() += original_quality_map[v];
            if(!names_map.back().empty()) names_map.back().append('+');
            names_map.back().append(instance_case.vertex_name(v));
            components.back().push_back(v);
            component_num_map[v] = components.size() - 1;
        }
    }

    melon::static_digraph_builder<melon::static_digraph, double, std::string>
        builder(components.size());

    for(std::size_t i = 0; i < components.size(); ++i) {
        for(const vertex_t & v : components[i]) {
            for(const arc_t & a : melon::out_arcs(original_graph, v)) {
                const vertex_t & w = melon::arc_target(original_graph, a);
                builder.add_arc(i, component_num_map[w],
                                original_probability_map[a],
                                instance_case.arc_name(a));
            }
        }
    }

    auto [graph, arc_probability_map, arc_names] = builder.build();

    return;
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // TRIVIAL_REFORMULATE_HPP