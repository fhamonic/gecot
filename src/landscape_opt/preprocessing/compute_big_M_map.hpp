#ifndef LANDSCAPE_OPT_COMPUTE_BIG_M_MAP_HPP
#define LANDSCAPE_OPT_COMPUTE_BIG_M_MAP_HPP

#include "melon/algorithm/dijkstra.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"

#include "concepts/instance_case.hpp"
#include "indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {

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
        using ReverseGraph = melon::views::reverse<GR>;
        // melon::dijkstra<ReverseGraph, PM,
        //                 detail::parallel_eca_dijkstra_traits<ReverseGraph, PM>>
        //     algo(ReverseGraph(graph), probability_map);
        melon::dijkstra<GR, PM,
                        detail::parallel_eca_dijkstra_traits<GR, PM>>
            algo(graph, probability_map);

        for(auto && s : vertices_subrange) {
            big_M_map[s] = 0;
            algo.reset().add_source(s);
            for(const auto & [t, t_prob] : algo) {
                big_M_map[s] += t_prob * improved_quality_map[t];
            }
        }
    };

    auto do_compute = [&](auto && vertices_range) {
        if(parallel) {
            tbb::parallel_for(tbb::blocked_range(vertices_range.begin(),
                                                 vertices_range.end()),
                              compute_big_Ms);
        } else {
            compute_big_Ms(tbb::blocked_range(vertices_range.begin(),
                                              vertices_range.end()));
        }
    };

    if constexpr(std::ranges::random_access_range<
                     melon::vertices_range_t<GR>>) {
        do_compute(std::views::common(melon::vertices(graph)));

    } else {
        auto vertices_range = std::views::common(melon::vertices(graph));
        std::vector<melon::vertex_t<GR>> vertices_vector(vertices_range.begin(),
                                                         vertices_range.end());
        do_compute(vertices_vector);
    }
    return big_M_map;
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_BIG_M_MAP_HPP