#ifndef GECOT_COMPUTE_BIG_M_MAP_HPP
#define GECOT_COMPUTE_BIG_M_MAP_HPP

#include <limits>
#include <optional>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/knapsack_bnb.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/parallel_pc_num.hpp"

namespace fhamonic {
namespace gecot {

template <typename GR, typename QM, typename VOM, typename PM>
auto compute_big_M_map(const GR & graph, const QM & source_quality_map,
                       const VOM & source_options_map,
                       const PM & probability_map, auto && vertices) {
    auto big_M_map = melon::create_vertex_map<double>(
        graph, std::numeric_limits<double>::max());
    auto improved_source_quality_map = source_quality_map;
    for(const auto & v : melon::vertices(graph)) {
        for(const auto & [source_quality_gain, option] : source_options_map[v])
            improved_source_quality_map[v] += source_quality_gain;
    }

    std::vector<melon::vertex_t<GR>> vertices_vector;
    std::ranges::copy(vertices, std::back_inserter(vertices_vector));

    tbb::parallel_for(
        tbb::blocked_range(vertices_vector.begin(), vertices_vector.end()),
        [&](auto && vertices_subrange) {
            auto reversed_graph = melon::views::reverse(graph);
            auto algo = melon::dijkstra(
                detail::parallel_pc_num_dijkstra_traits<
                    decltype(reversed_graph),
                    melon::mapped_value_t<
                        PM, melon::arc_t<decltype(reversed_graph)>>>{},
                reversed_graph, probability_map);

            for(auto && s : vertices_subrange) {
                double M = 0;
                algo.reset().add_source(s);
                for(const auto & [t, t_prob] : algo) {
                    M += t_prob * improved_source_quality_map[t];
                }
                big_M_map[s] = M;
            }
        });

    return big_M_map;
}

template <instance_c I, typename GR, typename QM, typename VOM, typename PM>
auto compute_knapsack_big_M_map(const I & instance, const double budget,
                                const GR & graph, const QM & source_quality_map,
                                const VOM & source_options_map,
                                const PM & probability_map, auto && vertices) {
    auto big_M_map = melon::create_vertex_map<double>(
        graph, std::numeric_limits<double>::max());

    std::vector<melon::vertex_t<GR>> vertices_vector;
    std::ranges::copy(vertices, std::back_inserter(vertices_vector));

    tbb::parallel_for(
        tbb::blocked_range(vertices_vector.begin(), vertices_vector.end()),
        [&](auto && vertices_subrange) {
            auto reversed_graph = melon::views::reverse(graph);
            auto algo = melon::dijkstra(
                detail::parallel_pc_num_dijkstra_traits<
                    decltype(reversed_graph),
                    melon::mapped_value_t<
                        PM, melon::arc_t<decltype(reversed_graph)>>>{},
                reversed_graph, probability_map);

            auto option_gains = instance.template create_option_map<double>();

            for(auto && s : vertices_subrange) {
                double M = 0;
                option_gains.fill(0);
                algo.reset().add_source(s);
                for(const auto & [t, t_prob] : algo) {
                    M += t_prob * source_quality_map[t];
                    for(const auto & [source_quality_gain, option] :
                        source_options_map[t])
                        option_gains[option] += t_prob * source_quality_gain;
                }
                melon::knapsack_bnb knapsack(
                    instance.options(), option_gains,
                    [&instance](auto && o) { return instance.option_cost(o); },
                    budget);
                big_M_map[s] = M + knapsack.run().solution_value();
            }
        });

    return big_M_map;
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_BIG_M_MAP_HPP