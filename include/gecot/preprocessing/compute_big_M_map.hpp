#ifndef GECOT_COMPUTE_BIG_M_MAP_HPP
#define GECOT_COMPUTE_BIG_M_MAP_HPP

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/knapsack_bnb.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/bounds/lp_knapsack.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"

namespace gecot {

// plain: the quality that reaches a vertex with every quality gain;
// knapsack_bnb: with the gains of its best options within the budget;
// knapsack_dantzig: with the linear relaxation of that knapsack, looser but
// without the branch and bound's blow-ups; tight: knapsack_bnb, lowered per
// improvement by compute_tight_big_M_maps
enum class big_M_kind { plain, knapsack_bnb, knapsack_dantzig, tight };

template <typename GR, typename QM, typename VOM, typename PM>
auto compute_big_M_map(const GR & graph, const QM & source_quality_map,
                       const VOM & source_options_map,
                       const PM & probability_map, auto && vertices) {
    auto big_M_map = melon::create_vertex_map<double>(
        graph, std::numeric_limits<double>::max());
    auto improved_source_quality_map = source_quality_map;
    for(const auto & v : melon::vertices(graph)) {
        for(const auto & [source_quality_gain, option] : source_options_map[v])
            improved_source_quality_map[v] += source_quality_gain;
    }

    std::vector<melon::vertex_t<GR>> vertices_vector;
    std::ranges::copy(vertices, std::back_inserter(vertices_vector));

    tbb::parallel_for(
        tbb::blocked_range(vertices_vector.begin(), vertices_vector.end()),
        [&](auto && vertices_subrange) {
            auto reversed_graph = melon::views::reverse(graph);
            auto algo = melon::dijkstra(
                detail::pc_num_dijkstra_traits<
                    decltype(reversed_graph),
                    melon::mapped_value_t<
                        PM, melon::arc_t<decltype(reversed_graph)>>>{},
                reversed_graph, probability_map);

            for(auto && s : vertices_subrange) {
                double M = 0;
                algo.reset().add_source(s);
                for(const auto & [t, t_prob] : algo) {
                    M += t_prob * improved_source_quality_map[t];
                }
                big_M_map[s] = M;
            }
        });

    return big_M_map;
}

template <instance_c I, typename GR, typename QM, typename VOM, typename PM>
auto compute_knapsack_big_M_map(const I & instance, const budget_t & budget,
                                const bool dantzig_bound, const GR & graph,
                                const QM & source_quality_map,
                                const VOM & source_options_map,
                                const PM & probability_map, auto && vertices) {
    auto big_M_map = melon::create_vertex_map<double>(
        graph, std::numeric_limits<double>::max());

    std::vector<melon::vertex_t<GR>> vertices_vector;
    std::ranges::copy(vertices, std::back_inserter(vertices_vector));

    tbb::parallel_for(
        tbb::blocked_range(vertices_vector.begin(), vertices_vector.end()),
        [&](auto && vertices_subrange) {
            auto reversed_graph = melon::views::reverse(graph);
            auto algo = melon::dijkstra(
                detail::pc_num_dijkstra_traits<
                    decltype(reversed_graph),
                    melon::mapped_value_t<
                        PM, melon::arc_t<decltype(reversed_graph)>>>{},
                reversed_graph, probability_map);

            auto option_gains = instance.template create_option_map<double>();
            auto dimension_gains =
                instance.template create_option_map<double>();
            std::vector<double> gains;
            std::vector<std::vector<double>> costs(budget.size());
            for(std::size_t r = 0; r < budget.size(); ++r)
                for(auto && o : instance.options())
                    costs[r].push_back(instance.option_cost(o, r));
            auto fits = [&](const option_t o) {
                for(std::size_t r = 0; r < budget.size(); ++r)
                    if(costs[r][o] > budget[r]) return false;
                return true;
            };

            for(auto && s : vertices_subrange) {
                double M = 0;
                option_gains.fill(0);
                algo.reset().add_source(s);
                for(const auto & [t, t_prob] : algo) {
                    M += t_prob * source_quality_map[t];
                    for(const auto & [source_quality_gain, option] :
                        source_options_map[t])
                        option_gains[option] += t_prob * source_quality_gain;
                }
                // each cost dimension alone is a relaxation, and so is
                // ignoring the conflict sets: the least of the knapsacks
                double knapsack_value = std::numeric_limits<double>::infinity();
                for(std::size_t r = 0; r < budget.size(); ++r) {
                    if(dantzig_bound) {
                        // the options that do not fit would enter as fractions
                        gains.clear();
                        for(auto && o : instance.options())
                            gains.push_back(fits(o) ? option_gains[o] : 0.0);
                        knapsack_value =
                            std::min(knapsack_value,
                                     lp_knapsack(gains, costs[r], budget[r]));
                        continue;
                    }
                    // the options free in r are in every solution: in the
                    // branch and bound, they make it explode. Their gains are
                    // zeroed, not their items removed, for the same order.
                    double free_gains = 0.0;
                    for(auto && o : instance.options()) {
                        dimension_gains[o] = option_gains[o];
                        if(costs[r][o] > 0) continue;
                        free_gains += option_gains[o];
                        dimension_gains[o] = 0.0;
                    }
                    melon::knapsack_bnb knapsack(
                        instance.options(), dimension_gains,
                        [&](auto && o) { return costs[r][o]; }, budget[r]);
                    knapsack_value =
                        std::min(knapsack_value,
                                 free_gains + knapsack.run().solution_value());
                }
                big_M_map[s] = M + knapsack_value;
            }
        });

    return big_M_map;
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_BIG_M_MAP_HPP