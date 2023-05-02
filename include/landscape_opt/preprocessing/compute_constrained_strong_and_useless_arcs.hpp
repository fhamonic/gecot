#ifndef LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"
#include "melon/graph.hpp"

#include "landscape_opt/helper.hpp"
#include "landscape_opt/preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_constrained_strong_and_useless_arcs(const I & instance_case,
                                                 const bool parallel = false) {
    using LS = typename I::Landscape;
    using GR = typename LS::Graph;
    using PM = typename LS::ProbabilityMap;
    using arc_t = melon::arc_t<GR>;

    const LS & landscape = instance_case.landscape();
    const GR & graph = landscape.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const PM & upper_length_map = landscape.probability_map();

    const auto arcOptions = detail::computeOptionsForArcs(instance_case);

    auto cost_map = melon::create_arc_map < double >> (graph);
    const int nb_mus = 5;
    double max_mu = 0;
    for(auto && i : instance_case.options()) {
        const double arc_cost = instance_case.option_cost(i) / arcOptions[i].size();
        for(auto && [a, enhanced_prob] : arcOptions[i]) {
            max_mu =
                std::max(max_mu, std::log(enhanced_prob / upper_length_map[a]) /
                                     arc_cost);
        }
    }

    std::vector<std::pair<double, PM>> modified_lower_length_maps;
    for(int i = 0; i < nb_mus; ++i) {
        const double mu = max_mu * i / nb_mus;
        modified_lower_length_maps.emplace_back(mu, upper_length_map);
        auto & modified_lower_length_map =
            modified_lower_length_maps.back().second;
        for(auto && i : instance_case.options()) {
            const double modifier = std::exp(
                -mu * instance_case.option_cost(i) / arcOptions[i].size());
            for(auto && [a, enhanced_prob] : arcOptions[i]) {
                modified_lower_length_maps[a] =
                    std::max(mu_lower_length_map[a], modifier * enhanced_prob);
            }
        }
    }

    auto compute_strong_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            melon::strong_fiber<GR, PM, PM,
                                detail::strong_arc_default_traits<GR, double>>
                algo(graph, lower_length_map, upper_length_map);
            for(const auto & uv : arcs_block) {
                algo.reset().add_strong_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    strong_arcs_map[w].push_back(uv);
                }
            }
        };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_strong_arcs);
    } else {
        compute_strong_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    auto compute_useless_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            melon::strong_fiber<GR, PM, PM,
                                detail::useless_arc_default_traits<GR, double>>
                algo(graph, lower_length_map, upper_length_map);
            for(const auto & uv : arcs_block) {
                useless_arcs_map[graph.arc_source(uv)].push_back(uv);
                algo.reset().add_useless_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    useless_arcs_map[w].push_back(uv);
                }
            }
        };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_useless_arcs);
    } else {
        compute_useless_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP