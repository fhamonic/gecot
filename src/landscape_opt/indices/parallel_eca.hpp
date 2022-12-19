#ifndef LANDSCAPE_OPT_INDICES_PARALLEL_ECA_HPP
#define LANDSCAPE_OPT_INDICES_PARALLEL_ECA_HPP

#include <cmath>
#include <concepts>

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include "melon/all.hpp"

#include "concepts/landscape.hpp"

namespace fhamonic {
namespace landscape_opt {

namespace detail {
template <typename GR, typename PM>
struct parallel_eca_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<
        melon::mapped_value_t<PM, melon::arc_t<GR>>>;
    using heap =
        melon::d_ary_heap<4, melon::vertex_t<GR>,
                          melon::mapped_value_t<PM, melon::arc_t<GR>>,
                          decltype([](const auto & e1, const auto & e2) {
                              return semiring::less(e1.second, e2.second);
                          }),
                          melon::vertex_map_t<GR, std::size_t>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <typename GR, typename QM, typename PM>
double parallel_eca(const GR & graph, const QM & quality_map,
                    const PM & probability_map) {
    auto vertices_range = melon::vertices(graph);

    double eca_sum = tbb::parallel_reduce(
        tbb::blocked_range(vertices_range.begin(), vertices_range.end()), 0.0,
        [&](auto && nodes_subrange, double init) {
            melon::dijkstra<GR, PM,
                            detail::parallel_eca_dijkstra_traits<GR, PM>>
                algo(graph, probability_map);

            for(auto && s : nodes_subrange) {
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

    return std::sqrt(eca_sum);
};

template <concepts::Landscape LS>
double parallel_eca(const LS l) {
    return parallel_eca(l.graph(), l.quality_map(), l.probability_map());
};

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_INDICES_PARALLEL_ECA_HPP