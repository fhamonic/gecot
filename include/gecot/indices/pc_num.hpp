#ifndef GECOT_INDICES_PC_NUM_HPP
#define GECOT_INDICES_PC_NUM_HPP

#include <cmath>
#include <concepts>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/views/reverse.hpp"

namespace fhamonic {
namespace gecot {

namespace detail {
template <typename GR, typename V>
struct pc_num_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<V>;
    using heap = melon::updatable_d_ary_heap<
        4, std::pair<melon::vertex_t<GR>, V>, typename semiring::less_t,
        melon::vertex_map_t<GR, std::size_t>, melon::views::element_map<1>,
        melon::views::element_map<0>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <typename GR, typename SQM, typename TQM, typename PM>
double pc_num(const GR & graph, const SQM & source_quality_map,
              const TQM & target_quality_map, const PM & probability_map) {
    using V = melon::mapped_value_t<PM, melon::arc_t<GR>>;
    auto algo = melon::dijkstra(detail::pc_num_dijkstra_traits<GR, V>{}, graph,
                                probability_map);

    V pc_num_sum = 0.0;
    for(const auto & s : melon::vertices(graph)) {
        if(source_quality_map[s] == 0) continue;
        V sum = 0.0;
        algo.reset();
        algo.add_source(s);
        for(const auto & [u, prob] : algo) {
            sum += target_quality_map[u] * prob;
        }
        pc_num_sum += source_quality_map[s] * sum;
    }

    return pc_num_sum;
};

template <typename GR, typename SQM, typename PM>
double pc_num_vertex_in_flow(const GR & graph, const SQM & source_quality_map,
                             const PM & probability_map,
                             const melon::vertex_t<GR> & t) {
    using V = melon::mapped_value_t<PM, melon::arc_t<GR>>;
    V sum = 0.0;
    auto && rgraph = melon::views::reverse(graph);
    std::cout << t << " " << graph.is_valid_vertex(t) << std::endl;
    for(const auto & [u, prob] :
        melon::dijkstra(detail::pc_num_dijkstra_traits<decltype(rgraph), V>{},
                        rgraph, probability_map, t)) {
        sum += source_quality_map[u] * prob;
    }
    return sum;
};

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_INDICES_PC_NUM_HPP