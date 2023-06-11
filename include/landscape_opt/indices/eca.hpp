#ifndef LANDSCAPE_OPT_INDICES_ECA_HPP
#define LANDSCAPE_OPT_INDICES_ECA_HPP

#include <cmath>
#include <concepts>

#include "melon/algorithm/dijkstra.hpp"

namespace fhamonic {
namespace landscape_opt {

namespace detail {
template <typename GR, typename PM>
struct eca_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<
        melon::mapped_value_t<PM, melon::arc_t<GR>>>;
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return semiring::less(e1.second, e2.second);
        }
    };
    using heap =
        melon::d_ary_heap<4, melon::vertex_t<GR>,
                          melon::mapped_value_t<PM, melon::arc_t<GR>>,
                          entry_cmp, melon::vertex_map_t<GR, std::size_t>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <typename GR, typename QM, typename PM>
double eca(const GR & graph, const QM & quality_map,
           const PM & probability_map) {
    melon::dijkstra<GR, PM, detail::eca_dijkstra_traits<GR, PM>> algo(
        graph, probability_map);

    double eca_sum = 0.0;
    for(const auto & s : melon::vertices(graph)) {
        if(quality_map[s] == 0) continue;
        double sum = 0.0;
        algo.reset();
        algo.add_source(s);
        for(const auto & [u, prob] : algo) {
            sum += quality_map[u] * prob;
        }
        eca_sum += quality_map[s] * sum;
    }

    return std::sqrt(eca_sum);
};

template <typename GR, typename QM, typename PM>
double eca_vertex_contribution(const GR & graph, const QM & quality_map,
           const PM & probability_map, const melon::vertex_t<GR> & t) {
    melon::dijkstra<GR, PM, detail::eca_dijkstra_traits<GR, PM>> algo(
        graph, probability_map);
    
    double sum = 0.0;
    algo.reset();
    algo.add_source(t);
    for(const auto & [u, prob] : algo) {
        sum += quality_map[u] * prob;
    }
    
    return sum;
};

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_INDICES_ECA_HPP