#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/adaptor/reverse.hpp"
#include "melon/concepts/graph.hpp"
#include "melon/static_digraph.hpp"
#include "melon/static_digraph_builder.hpp"

#include "concepts/instance.hpp"
#include "indices/eca.hpp"
#include "indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {

template <concepts::Instance I>
std::vector<std::vector<
    std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
computeOptionsForNodes(const I & instance) noexcept {
    std::vector<std::vector<
        std::pair<melon::vertex_t<typename I::Landscape::Graph>, double>>>
        nodeOptionsMap(instance.options().size());
    for(auto && u : instance.landscape().graph().vertices()) {
        for(auto && [quality_gain, option] : instance.node_options_map()[u]) {
            nodeOptionsMap[option].emplace_back(u, quality_gain);
        }
    }
    return nodeOptionsMap;
}

template <concepts::Instance I>
std::vector<
    std::vector<std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
computeOptionsForArcs(const I & instance) noexcept {
    std::vector<std::vector<
        std::pair<melon::arc_t<typename I::Landscape::Graph>, double>>>
        arcOptionsMap(instance.options().size());
    for(auto && a : instance.landscape().graph().arcs())
        for(auto && [enhanced_prob, option] : instance.arc_options_map()[a])
            arcOptionsMap[option].emplace_back(a, enhanced_prob);
    return arcOptionsMap;
}

}  // namespace detail

template <concepts::InstanceCase I>
auto generalized_flow_graph(const I & instance_case) {
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

    for(auto && a : original_graph.arcs()) {
        builder.add_arc(original_graph.source(a), original_graph.target(a),
                        original_probability_map[a], std::nullopt);
        for(auto && [enhanced_prob, option] :
            instance_case.arc_options_map()[a]) {
            builder.add_arc(original_graph.source(a), original_graph.target(a),
                            enhanced_prob, std::make_optional(option));
        }
    }

    auto [graph, probability_map, arc_options_map] = builder.build();

    return std::make_tuple(graph, instance_case.landscape().quality_map(),
                           instance_case.node_options_map(), probability_map,
                           arc_options_map);
}

template <typename GR, typename QM, typename VOM, typename PM>
auto compute_big_M_map(const GR & graph, const QM & quality_map,
                       const VOM & vertex_options_map,
                       const PM & probability_map,
                       const bool parallel = false) {
    auto big_M_map = melon::create_vertex_map<double>(graph);
    auto improved_quality_map = quality_map;
    for(const auto & v : melon::vertices(graph)) {
        for(const auto & [quality_gain, option] : vertex_options_map[v])
            improved_quality_map[v] += quality_gain;
    }

    auto compute_big_Ms = [&](auto && vertices_subrange) {
        using ReverseGraph = melon::adaptors::reverse<GR>;
        melon::dijkstra<ReverseGraph, PM,
                        detail::parallel_eca_dijkstra_traits<ReverseGraph, PM>>
            algo(ReverseGraph(graph), probability_map);

        for(auto && s : vertices_subrange) {
            if(quality_map[s] == 0) continue;
            double sum = 0.0;
            algo.reset();
            algo.add_source(s);
            for(const auto & [u, prob] : algo) {
                sum += prob * improved_quality_map[u];
            }
            big_M_map[s] = sum;
        }
    };

    auto vertices_range = melon::vertices(graph);
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(vertices_range.begin(), vertices_range.end()),
            compute_big_Ms);
    } else {
        compute_big_Ms(
            tbb::blocked_range(vertices_range.begin(), vertices_range.end()));
    }
    return big_M_map;
}

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP