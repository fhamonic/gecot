#ifndef GECOT_INDICES_PC_NUM_HPP
#define GECOT_INDICES_PC_NUM_HPP

#include <cmath>
#include <concepts>

#include "melon/algorithm/dijkstra.hpp"

namespace fhamonic {
namespace gecot {

namespace detail {
template <typename GR, typename V>
struct pc_num_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<V>;
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return semiring::less(e1.second, e2.second);
        }
    };
    using heap = melon::d_ary_heap<4, melon::vertex_t<GR>, V, entry_cmp,
                                   melon::vertex_map_t<GR, std::size_t>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <typename GR, typename QM, typename PM>
double pc_num(const GR & graph, const QM & quality_map,
              const PM & probability_map) {
    using V = melon::mapped_value_t<PM, melon::arc_t<GR>>;
    auto algo = melon::dijkstra(detail::pc_num_dijkstra_traits<GR, V>{}, graph,
                                probability_map);

    V pc_num_sum = 0.0;
    for(const auto & s : melon::vertices(graph)) {
        if(quality_map[s] == 0) continue;
        V sum = 0.0;
        algo.reset();
        algo.add_source(s);
        for(const auto & [u, prob] : algo) {
            sum += quality_map[u] * prob;
        }
        pc_num_sum += quality_map[s] * sum;
    }

    return pc_num_sum;
};

template <typename GR, typename QM, typename PM>
double pc_num_vertex_contribution(const GR & graph, const QM & quality_map,
                                  const PM & probability_map,
                                  const melon::vertex_t<GR> & t) {
    using V = melon::mapped_value_t<PM, melon::arc_t<GR>>;
    auto algo = melon::dijkstra(detail::pc_num_dijkstra_traits<GR, V>{}, graph,
                                probability_map);

    V sum = 0.0;
    algo.reset();
    algo.add_source(t);
    for(const auto & [u, prob] : algo) {
        sum += quality_map[u] * prob;
    }

    return sum;
};

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_INDICES_PC_NUM_HPP