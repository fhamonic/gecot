#ifndef LANDSCAPE_OPT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"
#include "melon/graph.hpp"

#include "landscape_opt/helper.hpp"
#include "landscape_opt/preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace landscape_opt {

template <instance_c I, case_c C>
auto compute_constrained_strong_and_useless_arcs(
    const I & instance, const C & instance_case, const double budget,
    const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; }) {
    using graph_t = case_graph_t<C>;
    using probability_map_t = case_probability_map_t<C>;
    using arc_t = melon::arc_t<graph_t>;

    const auto & graph = instance_case.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();

    const auto arcs_options = instance.create_option_map(
        std::vector<std::pair<melon::arc_t<graph_t>, double>>{});
    compute_case_arc_options(instance_case, arcs_options);

    auto cost_map = melon::create_arc_map<double>(graph);
    const int nb_mus = 5;
    double max_mu = 0;
    for(auto && i : instance.options()) {
        const double arc_cost = instance.option_cost(i) /
                                static_cast<double>(arcs_options[i].size());
        for(auto && [a, enhanced_prob] : arcs_options[i]) {
            max_mu = std::max(
                max_mu,
                std::log(enhanced_prob / base_probability_map[a]) / arc_cost);
        }
    }

    std::vector<std::pair<double, probability_map_t>>
        lagrange_improved_probability_maps;
    for(int mu_id = 0; mu_id < nb_mus; ++mu_id) {
        const double mu = max_mu * mu_id / nb_mus;
        lagrange_improved_probability_maps.emplace_back(mu,
                                                        base_probability_map);
        auto & mu_length_map = lagrange_improved_probability_maps.back().second;
        for(auto && i : instance.options()) {
            const double modifier =
                std::exp(-mu * instance.option_cost(i) /
                         static_cast<double>(arcs_options[i].size()));
            for(auto && [a, enhanced_prob] : arcs_options[i]) {
                mu_length_map[a] =
                    std::max(mu_length_map[a], modifier * enhanced_prob);
            }
        }
    }

    auto compute_strong_arcs = [&](const tbb::blocked_range<
                                   decltype(arcs_range.begin())> & arcs_block) {
        melon::strong_fiber<graph_t, probability_map_t, probability_map_t,
                            detail::strong_arc_default_traits<graph_t, double>>
            algo(graph, base_probability_map, base_probability_map);
        for(const auto & uv : arcs_block) {
            for(auto && [mu, mu_length_map] :
                lagrange_improved_probability_maps) {
                algo.reset()
                    .set_lower_length_map(mu_length_map)
                    .add_strong_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    strong_arcs_map[w].push_back(uv);
                }
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

    auto compute_useless_arcs = [&](const tbb::blocked_range<
                                    decltype(arcs_range.begin())> &
                                        arcs_block) {
        melon::strong_fiber<graph_t, probability_map_t, probability_map_t,
                            detail::useless_arc_default_traits<graph_t, double>>
            algo(graph, base_probability_map, base_probability_map);
        for(const auto & uv : arcs_block) {
            for(auto && [mu, mu_length_map] :
                lagrange_improved_probability_maps) {
                useless_arcs_map[graph.arc_source(uv)].push_back(uv);
                algo.reset()
                    .set_lower_length_map(mu_length_map)
                    .add_useless_arc_source(uv);
                for(const auto & [w, w_dist] : algo) {
                    useless_arcs_map[w].push_back(uv);
                }
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

#endif  // LANDSCAPE_OPT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP