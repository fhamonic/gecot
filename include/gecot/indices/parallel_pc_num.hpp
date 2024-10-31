#ifndef GECOT_INDICES_PARALLEL_PC_NUM_HPP
#define GECOT_INDICES_PARALLEL_PC_NUM_HPP

#include <cmath>
#include <concepts>

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/container/d_ary_heap.hpp"

namespace fhamonic {
namespace gecot {

namespace detail {
template <typename GR, typename V>
struct parallel_pc_num_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<V>;
    using heap = melon::updatable_d_ary_heap<
        4, std::pair<melon::vertex_t<GR>, V>, typename semiring::less_t,
        melon::vertex_map_t<GR, std::size_t>, melon::views::get_map<1>,
        melon::views::get_map<0>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <typename GR, typename QM, typename PM>
double parallel_pc_num(const GR & graph, const QM & quality_map,
                       const PM & probability_map) {
    auto vertices_range = melon::vertices(graph);

    double pc_num_sum = tbb::parallel_reduce(
        tbb::blocked_range(vertices_range.begin(), vertices_range.end()), 0.0,
        [&](auto && vertices_subrange, double init) {
            auto algo = melon::dijkstra(
                detail::parallel_pc_num_dijkstra_traits<
                    GR, melon::mapped_value_t<PM, melon::arc_t<GR>>>{},
                graph, probability_map);

            for(auto && s : vertices_subrange) {
                if(quality_map[s] == 0) continue;
                double sum = 0.0;
                algo.reset();
                algo.add_source(s);
                for(const auto & [u, prob] : algo) {
                    sum += quality_map[u] * prob;
                }
                init += quality_map[s] * sum;
            }
            return init;
        },
        std::plus<double>());

    return pc_num_sum;
};

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_INDICES_PARALLEL_PC_NUM_HPP