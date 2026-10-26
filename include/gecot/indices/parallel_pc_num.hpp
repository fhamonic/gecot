#ifndef GECOT_INDICES_PARALLEL_PC_NUM_HPP
#define GECOT_INDICES_PARALLEL_PC_NUM_HPP

#include <cmath>
#include <concepts>

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include "melon/algorithm/dijkstra.hpp"

#include "gecot/indices/pc_num.hpp"

namespace gecot {

template <typename GR, typename SQM, typename TQM, typename PM>
double parallel_pc_num(const GR & graph, const SQM & source_quality_map,
                       const TQM & target_quality_map,
                       const PM & probability_map) {
    auto vertices_range = melon::vertices(graph);

    double pc_num_sum = tbb::parallel_reduce(
        tbb::blocked_range(vertices_range.begin(), vertices_range.end()), 0.0,
        [&](auto && vertices_subrange, double init) {
            auto algo = melon::dijkstra(
                detail::pc_num_dijkstra_traits<
                    GR, melon::mapped_value_t<PM, melon::arc_t<GR>>>{},
                graph, probability_map);

            for(auto && s : vertices_subrange) {
                if(source_quality_map[s] == 0) continue;
                double sum = 0.0;
                algo.reset();
                algo.add_source(s);
                for(const auto & [u, prob] : algo) {
                    sum += target_quality_map[u] * prob;
                }
                init += source_quality_map[s] * sum;
            }
            return init;
        },
        std::plus<double>());

    return pc_num_sum;
};

}  // namespace gecot

#endif  // GECOT_INDICES_PARALLEL_PC_NUM_HPP