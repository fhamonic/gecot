#ifndef GECOT_TEST_INSTANCE_BUILDER_HPP
#define GECOT_TEST_INSTANCE_BUILDER_HPP

#include <cstddef>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "melon/container/static_digraph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

#include "instance.hpp"

// a case in plain vectors; arc options are indexed like arcs
struct case_spec {
    std::size_t num_vertices;
    std::vector<std::pair<unsigned int, unsigned int>> arcs;
    std::vector<double> probabilities;
    std::vector<double> source_qualities, target_qualities;
    // per vertex: (source quality gain, target quality gain, option)
    std::vector<std::vector<std::tuple<double, double, gecot::option_t>>>
        vertex_options;
    // per arc: (improved probability, option)
    std::vector<std::vector<std::pair<double, gecot::option_t>>> arc_options;
};

// options of several costs, named after the dimensions, and conflict sets
inline Instance build_instance(
    const std::vector<std::string> & cost_dimensions,
    const std::vector<std::vector<double>> & option_costs,
    const std::vector<std::vector<gecot::option_t>> & conflict_sets,
    const std::vector<case_spec> & cases, criterion_formula criterion) {
    using graph_t = melon::static_digraph;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;
    Instance instance;
    instance.set_cost_dimensions(cost_dimensions);
    for(std::size_t o = 0; o < option_costs.size(); ++o)
        instance.add_option("o" + std::to_string(o), option_costs[o]);
    for(std::size_t l = 0; l < conflict_sets.size(); ++l)
        instance.add_conflict_set("s" + std::to_string(l), conflict_sets[l]);
    for(std::size_t k = 0; k < cases.size(); ++k) {
        const case_spec & spec = cases[k];
        melon::static_digraph_builder<graph_t, double, std::size_t> builder(
            spec.num_vertices);
        for(std::size_t i = 0; i < spec.arcs.size(); ++i)
            builder.add_arc({spec.arcs[i].first, spec.arcs[i].second},
                            spec.probabilities[i], i);
        auto [graph, probabilities, input_index] = builder.build();
        std::vector<std::string> vertex_names, arc_names;
        phmap::node_hash_map<std::string, vertex_t> vertex_ids;
        phmap::node_hash_map<std::string, arc_t> arc_ids;
        for(auto && v : melon::vertices(graph)) {
            vertex_names.push_back("v" + std::to_string(v));
            vertex_ids[vertex_names.back()] = v;
        }
        for(auto && a : melon::arcs(graph)) {
            arc_names.push_back("a" + std::to_string(input_index[a]));
            arc_ids[arc_names.back()] = a;
        }
        auto vertex_options = melon::create_vertex_map<
            std::vector<std::tuple<double, double, gecot::option_t>>>(graph,
                                                                      {});
        for(auto && v : melon::vertices(graph))
            if(v < spec.vertex_options.size())
                vertex_options[v] = spec.vertex_options[v];
        auto arc_options = melon::create_arc_map<
            std::vector<std::pair<double, gecot::option_t>>>(graph, {});
        for(auto && a : melon::arcs(graph))
            if(input_index[a] < spec.arc_options.size())
                arc_options[a] = spec.arc_options[input_index[a]];
        auto & instance_case = instance.emplace_case(
            "c" + std::to_string(k), std::move(graph),
            std::vector<double>(spec.source_qualities),
            std::vector<double>(spec.target_qualities),
            std::vector<double>(probabilities.begin(), probabilities.end()),
            std::move(vertex_names), std::move(vertex_ids),
            std::move(arc_names), std::move(arc_ids));
        instance_case.set_vertex_options_map(std::move(vertex_options));
        instance_case.set_arc_options_map(std::move(arc_options));
    }
    instance.set_criterion(std::move(criterion));
    return instance;
}

inline Instance build_instance(const std::vector<double> & option_costs,
                               const std::vector<case_spec> & cases,
                               criterion_formula criterion) {
    std::vector<std::vector<double>> costs;
    for(const double cost : option_costs) costs.push_back({cost});
    return build_instance({"cost"}, costs, {}, cases, std::move(criterion));
}

#endif  // GECOT_TEST_INSTANCE_BUILDER_HPP
