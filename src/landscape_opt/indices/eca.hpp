#ifndef LANDSCAPE_OPT_INDICES_ECA_HPP
#define LANDSCAPE_OPT_INDICES_ECA_HPP

#include <cmath>
#include <concepts>

#include "melon/all.hpp"

#include "concepts/landscape.hpp"

namespace fhamonic {
namespace landscape_opt {

template <typename GR, typename QM, typename PM>
double eca(const GR & graph, const QM & quality_map,
           const PM & probability_map) {

    melon::Dijkstra<GR, PM, melon::TRACK_NONE,
                    melon::DijkstraMostProbablePathSemiring<double>>
        dijkstra(graph, probability_map);

    double eca_sum = 0.0;
    for(auto && s : graph.nodes()) {
        double sum = 0.0;
        dijkstra.reset(s);
        dijkstra.add_source(s);
        for(const auto & [u, prob] : dijkstra) {
            sum += quality_map[u] * prob;
        }
        eca_sum += quality_map[s] * sum;
    }

    return std::sqrt(eca_sum);
};

template <concepts::Landscape LS>
double eca(const LS l) {
    return eca(l.graph(), l.quality_map(), l.probability_map());
};


}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_INDICES_ECA_HPP