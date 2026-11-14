#ifndef GECOT_COMPUTE_TIGHT_BIG_M_MAPS_HPP
#define GECOT_COMPUTE_TIGHT_BIG_M_MAPS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/knapsack_bnb.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"

namespace gecot {

// The tight big-Ms of a flow toward t, of docs/formulations/preprocessing.md
// ("The tight constants"): one per arc a and option i improving it, bounding
// the flow that leaves the tail of a on a improved by i, and one per option
// of target_options, bounding the flow that reaches t when it is bought.
// The graph gives each arc a base probability and the (improved probability,
// option) pairs of its improvements; flow_mip's graph has an arc of base
// probability 0 per improvement. The costs and the budget are those of the
// cost dimension 0.
template <instance_c I, typename GR, typename QM, typename SOM, typename BPM,
          typename IM>
auto compute_tight_big_M_maps(const I & instance, const double budget,
                              const double tolerance, const GR & graph,
                              const QM & source_quality_map,
                              const SOM & source_options_map,
                              const BPM & base_probability_map,
                              const IM & improvements_map,
                              const melon::vertex_t<GR> & t,
                              const auto & target_options) {
    using vertex_t = melon::vertex_t<GR>;
    using arc_t = melon::arc_t<GR>;
    const auto reversed_graph = melon::views::reverse(graph);
    using reversed_traits =
        detail::pc_num_dijkstra_traits<std::decay_t<decltype(reversed_graph)>,
                                       double>;
    const std::size_t num_options = std::ranges::size(instance.options());

    // per option its improved arcs, and the share 1 / k_i of its cost that
    // each pays
    std::vector<std::vector<arc_t>> option_arcs_of(num_options);
    auto improved_probability_map = melon::create_arc_map<double>(graph, 0.0);
    for(auto && a : melon::arcs(graph)) {
        improved_probability_map[a] = base_probability_map[a];
        for(auto && [p, i] : improvements_map[a]) {
            if(instance.option_cost(i, 0) > budget) continue;
            option_arcs_of[i].push_back(a);
            improved_probability_map[a] =
                std::max(improved_probability_map[a], p);
        }
    }
    std::vector<double> share(num_options, 0.0);
    for(std::size_t i = 0; i < num_options; ++i) {
        if(option_arcs_of[i].empty()) continue;
        std::vector<vertex_t> heads, tails;
        for(auto && a : option_arcs_of[i]) {
            heads.push_back(melon::arc_target(graph, a));
            tails.push_back(melon::arc_source(graph, a));
        }
        for(auto * v : {&heads, &tails}) {
            std::ranges::sort(*v);
            v->erase(std::ranges::unique(*v).begin(), v->end());
        }
        share[i] =
            1.0 / static_cast<double>(std::min(heads.size(), tails.size()));
    }

    // the probabilities of connection to t, with every improvement and with
    // none
    auto improved_to_t = melon::create_vertex_map<double>(graph, 0.0);
    for(const auto & [v, p] : melon::dijkstra(reversed_traits{}, reversed_graph,
                                              improved_probability_map, t))
        improved_to_t[v] = p;
    auto base_to_t = melon::create_vertex_map<double>(graph, 0.0);
    for(const auto & [v, p] : melon::dijkstra(reversed_traits{}, reversed_graph,
                                              base_probability_map, t))
        base_to_t[v] = p;

    // The multipliers μ, times the budget; any μ >= 0 is valid. 0 first: its
    // pass chooses the sources kept, which the others only lower.
    static constexpr std::array<double, 6> multiples = {0.0, 0.5, 1.0,
                                                        2.0, 4.0, 8.0};
    static_assert(multiples.front() == 0.0);
    struct reach_scratch {
        option_t free_option = 0;
        double mu = 0.0;
        std::vector<double> best;
        std::vector<vertex_t> touched;
    };
    auto make_reach = [&](reach_scratch & sc) {
        auto tolled = melon::maps::function([&](const arc_t & e) {
            if(sc.mu == 0.0) return improved_probability_map[e];
            double p = base_probability_map[e];
            for(auto && [improved_probability, i] : improvements_map[e]) {
                if(instance.option_cost(i, 0) > budget) continue;
                p = std::max(
                    p, i == sc.free_option
                           ? improved_probability
                           : improved_probability *
                                 std::exp(-sc.mu * instance.option_cost(i, 0) *
                                          share[i]));
            }
            return p;
        });
        return melon::dijkstra(reversed_traits{}, reversed_graph,
                               std::move(tolled));
    };
    std::size_t vertex_id_bound = 0;
    for(auto && v : melon::vertices(graph))
        vertex_id_bound = std::max(vertex_id_bound, std::size_t{v} + 1);
    // for every source s that reaches from and is kept: the least over μ of
    // e^{μ (β - c_i)} Π^μ(s, from), the improvements of i free
    auto budget_reach = [&](auto & algo, reach_scratch & sc,
                            const vertex_t from, const option_t i, auto && keep,
                            std::vector<std::pair<vertex_t, double>> & reach) {
        const double left =
            std::max(0.0, budget - instance.option_cost(i, 0) + tolerance);
        sc.free_option = i;
        if(sc.best.size() < vertex_id_bound)
            sc.best.assign(vertex_id_bound, 0.0);
        reach.clear();
        for(const double multiple : multiples) {
            if(multiple > 0 && budget <= 0) break;
            sc.mu = multiple > 0 ? multiple / budget : 0.0;
            const double factor = std::exp(sc.mu * left);
            algo.reset().add_source(from);
            for(const auto & [s, p] : algo) {
                if(multiple == 0.0) {
                    if(!keep(s, p)) continue;
                    sc.touched.push_back(s);
                    sc.best[s] = p;
                } else if(sc.best[s] > 0)
                    sc.best[s] = std::min(sc.best[s], factor * p);
            }
        }
        for(const vertex_t s : sc.touched) {
            reach.emplace_back(s, sc.best[s]);
            sc.best[s] = 0.0;
        }
        sc.touched.clear();
    };

    // the knapsack of the gains of the other options, those of i outside it
    auto bound = [&](const std::vector<std::pair<vertex_t, double>> & reach,
                     const option_t i) {
        auto option_gains = instance.template create_option_map<double>();
        option_gains.fill(0.0);
        double M = 0.0;
        for(const auto & [s, p] : reach) {
            M += p * source_quality_map[s];
            for(const auto & [gain, option] : source_options_map[s]) {
                if(option == i)
                    M += p * gain;
                else
                    option_gains[option] += p * gain;
            }
        }
        std::vector<option_t> items;
        for(auto && option : instance.options()) {
            if(!(option_gains[option] > 0)) continue;
            // in every solution: in the branch and bound, they make it explode
            if(instance.option_cost(option, 0) == 0)
                M += option_gains[option];
            else
                items.push_back(option);
        }
        if(!items.empty()) {
            melon::knapsack_bnb knapsack(
                items, option_gains,
                [&instance](auto && option) {
                    return instance.option_cost(option, 0);
                },
                budget - instance.option_cost(i, 0) + tolerance);
            M += knapsack.run().solution_value();
        }
        return M;
    };

    // per arc, the big-Ms of its improvements, in order
    auto arc_big_M_map = melon::create_arc_map<std::vector<double>>(
        graph, std::vector<double>{});
    std::vector<arc_t> improved_arcs;
    for(auto && a : melon::arcs(graph)) {
        arc_big_M_map[a].assign(std::ranges::size(improvements_map[a]), 0.0);
        if(!arc_big_M_map[a].empty()) improved_arcs.push_back(a);
    }
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, improved_arcs.size()),
        [&](const tbb::blocked_range<std::size_t> & r) {
            reach_scratch sc;
            auto algo = make_reach(sc);
            std::vector<std::pair<vertex_t, double>> reach;
            for(std::size_t n = r.begin(); n != r.end(); ++n) {
                const arc_t a = improved_arcs[n];
                std::size_t rank = 0;
                for(auto && [improved_probability, i] : improvements_map[a]) {
                    const double through =
                        improved_probability *
                        improved_to_t[melon::arc_target(graph, a)];
                    // keeps s within a relative 1e-9 of a tie
                    budget_reach(
                        algo, sc, melon::arc_source(graph, a), i,
                        [&](const vertex_t & s, const double p) {
                            return p * through >= base_to_t[s] * (1 - 1e-9);
                        },
                        reach);
                    arc_big_M_map[a][rank++] = bound(reach, i);
                }
            }
        });

    // f_t with i bought, for the options of target_options: every source,
    // the gains of i outside the knapsack
    std::vector<double> target_big_M(num_options, 0.0);
    {
        reach_scratch sc;
        auto algo = make_reach(sc);
        std::vector<std::pair<vertex_t, double>> reach;
        for(const option_t i : target_options) {
            budget_reach(
                algo, sc, t, i,
                [](const vertex_t &, const double) { return true; }, reach);
            target_big_M[i] = bound(reach, i);
        }
    }

    return std::make_pair(std::move(arc_big_M_map), std::move(target_big_M));
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_TIGHT_BIG_M_MAPS_HPP
