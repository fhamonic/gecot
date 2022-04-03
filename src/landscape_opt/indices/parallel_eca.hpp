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

template <typename GR, typename QM, typename PM>
double parallel_eca(const GR & graph, const QM & quality_map,
           const PM & probability_map) {
    auto nodes_range = graph.nodes();

    double eca_sum = tbb::parallel_reduce(
        tbb::blocked_range(nodes_range.begin(), nodes_range.end()), 0.0,
        [&](auto && nodes_subrange, double init) {
            melon::Dijkstra<GR, PM, melon::TRACK_NONE,
                            melon::DijkstraMostProbablePathSemiring<double>>
                dijkstra(graph, probability_map);

            for(auto && s : nodes_subrange) {
                double sum = 0.0;
                dijkstra.reset(s);
                dijkstra.add_source(s);
                for(const auto & [u, prob] : dijkstra) {
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