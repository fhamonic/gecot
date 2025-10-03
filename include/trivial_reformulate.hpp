#ifndef TRIVIAL_REFORMULATE_HPP
#define TRIVIAL_REFORMULATE_HPP

#include <optional>
#include <utility>

#include "gecot/concepts/instance.hpp"
#include "melon/algorithm/strongly_connected_components.hpp"
#include "melon/utility/static_digraph_builder.hpp"
#include "melon/views/subgraph.hpp"

#include "instance.hpp"

namespace fhamonic {

template <gecot::instance_c I>
auto trivial_reformulate_instance(
    const I & instance, double budget = std::numeric_limits<double>::max()) {
    Instance reformulated_instance;
    reformulated_instance.set_criterion(instance.criterion());

    auto reformulated_option_map =
        instance.template create_option_map<std::optional<gecot::option_t>>();
    for(auto && original_option : instance.options()) {
        const double cost = instance.option_cost(original_option);
        if(cost > budget) continue;
        auto option = reformulated_instance.add_option(
            instance.option_name(original_option), cost);
        reformulated_option_map[original_option].emplace(option);
    }

    for(auto && instance_case : instance.cases()) {
        trivial_reformulate_case(instance_case, reformulated_instance,
                                 reformulated_option_map);
    }

    return reformulated_instance;
}

template <gecot::case_c C>
void trivial_reformulate_case(const C & instance_case, Instance & instance,
                              auto && reformulated_option_map) {
    using graph_t = gecot::case_graph_t<C>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    const auto & original_graph = instance_case.graph();
    const auto & original_source_quality_map =
        instance_case.source_quality_map();
    const auto & original_target_quality_map =
        instance_case.target_quality_map();
    const auto original_probability_map = instance_case.arc_probability_map();

    const auto & original_vertex_options_map =
        instance_case.vertex_options_map();
    const auto & original_arc_options_map = instance_case.arc_options_map();

    std::vector<double> source_quality_map;
    std::vector<double> target_quality_map;
    std::vector<std::string> vertex_names_map;
    std::vector<std::vector<vertex_t>> components;
    auto component_num_map =
        melon::create_vertex_map<std::size_t>(original_graph);

    for(auto component :
        melon::strongly_connected_components(melon::views::subgraph(
            original_graph, {}, [&original_probability_map](const arc_t & a) {
                return original_probability_map[a] == 1;
            }))) {
        source_quality_map.push_back(0.0);
        target_quality_map.push_back(0.0);
        vertex_names_map.emplace_back();
        const std::size_t component_num = components.size();
        components.emplace_back();
        for(const vertex_t & v : component) {
            source_quality_map.back() += original_source_quality_map[v];
            target_quality_map.back() += original_target_quality_map[v];
            if(!vertex_names_map.back().empty())
                vertex_names_map.back().append("+");
            vertex_names_map.back().append(instance_case.vertex_name(v));
            components.back().push_back(v);
            component_num_map[v] = component_num;
        }
    }

    std::size_t num_components = components.size();
    melon::static_digraph_builder<melon::static_digraph, double, std::string,
                                  arc_t>
        builder(num_components);
    for(std::size_t i = 0; i < num_components; ++i) {
        for(const vertex_t & v : components[i]) {
            for(const arc_t & a : melon::out_arcs(original_graph, v)) {
                const vertex_t & w = melon::arc_target(original_graph, a);
                if(i == component_num_map[w]) continue;
                builder.add_arc(static_cast<vertex_t>(i),
                                static_cast<vertex_t>(component_num_map[w]),
                                original_probability_map[a],
                                instance_case.arc_name(a), a);
            }
        }
    }

    auto [graph, arc_probability_map, arc_names, original_arc_map] =
        builder.build();

    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        vertex_name_to_id_map;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        arc_name_to_id_map;

    for(vertex_t v : melon::vertices(graph))
        vertex_name_to_id_map[vertex_names_map[v]] = v;
    for(arc_t a : melon::arcs(graph)) arc_name_to_id_map[arc_names[a]] = a;

    auto & reformulated_case = instance.emplace_case(
        instance_case.name(), std::move(graph), std::move(source_quality_map),
        std::move(target_quality_map), std::move(arc_probability_map),
        std::move(vertex_names_map), std::move(vertex_name_to_id_map),
        std::move(arc_names), std::move(arc_name_to_id_map));

    auto vertex_options = melon::create_vertex_map<
        std::vector<std::tuple<double, double, gecot::option_t>>>(graph, {});
    for(vertex_t v : melon::vertices(graph)) {
        for(vertex_t original_v : components[v]) {
            for(const auto & [source_quality_gain, target_quality_gain,
                              original_option] :
                original_vertex_options_map[original_v]) {
                if(!reformulated_option_map[original_option].has_value())
                    continue;
                vertex_options[v].emplace_back(
                    source_quality_gain, target_quality_gain,
                    reformulated_option_map[original_option].value());
            }
        }
    }
    reformulated_case.set_vertex_options_map(std::move(vertex_options));

    auto arc_options =
        melon::create_arc_map<std::vector<std::pair<double, gecot::option_t>>>(
            graph, {});
    for(arc_t a : melon::arcs(graph)) {
        for(const auto & [improved_prob, original_option] :
            original_arc_options_map[original_arc_map[a]]) {
            if(!reformulated_option_map[original_option].has_value()) continue;
            arc_options[a].emplace_back(
                improved_prob,
                reformulated_option_map[original_option].value());
        }
    }
    reformulated_case.set_arc_options_map(std::move(arc_options));
}

}  // namespace fhamonic

#endif  // TRIVIAL_REFORMULATE_HPP