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

// The big-Ms of a flow of quality toward t: one per arc a = (u, v) and option
// o improving it, bounding the flow that leaves u on a improved by o, and one
// per option with a target quality gain at t, bounding the flow F_t reaching t
// when the option is bought. The graph gives each arc a base probability and
// the (improved probability, option) pairs of its improvements; flow_mip's
// graph has an arc of base probability 0 per improvement.
//
// A big-M is valid if, for every x within the budget that buys o, some flow
// reaching the true F_t(x) satisfies it. The canonical flow does: each source
// w sends q_w(x) along one most reliable w-t path of the graph improved by x,
// among them one with the fewest improved arcs. So w sends nothing through a
// improved by o when
//   P̄_wu p_ao P̄_vt < P0_wt
// (P̄ with every improvement, P0 with none): every path through it is less
// reliable than w's best path without improvements, which x always leaves; on
// a tie, that path has fewer improved arcs. Hence, over the kept sources W,
//   flow <= Σ_{w ∈ W} q_w(x) P_wu(x),
// q_w(x) being the base quality, plus o's gains, plus the other options' gains
// within the budget B' = B - c_o left by o: a 0/1 knapsack.
//
// For P_wu(x): for any λ >= 0 and any w-u path π improved by the options S,
// all within B',
//   ln P_π <= ln P_π + λ (B' - c(S)) <= λ B' - Σ_{e ∈ π} (ℓ_e + λ c_o(e)
//   s_o(e))
// as c(S) >= Σ_{e ∈ π} c_o(e) s_o(e) when each option o' shares its cost over
// its arcs with s = 1 / k_o', k_o' bounding the arcs of o' a simple path can
// take: a simple path enters a vertex once, so k_o' is the smaller of the
// numbers of heads and of tails of the arcs of o'. o's own improvements are
// free. So P_wu(x) <= e^{λ B'} P_λ(w, u), the most reliable path where each
// improvement's probability is tolled by e^{-λ c_o' s_o'}; λ = 0 gives P̄_wu,
// and the bound takes the least over a few λ.
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

    // per option its improved arcs, and the share s of its cost each pays
    std::vector<std::vector<arc_t>> option_arcs_of(num_options);
    auto improved_probability_map = melon::create_arc_map<double>(graph, 0.0);
    for(auto && a : melon::arcs(graph)) {
        improved_probability_map[a] = base_probability_map[a];
        for(auto && [p, o] : improvements_map[a]) {
            if(instance.option_cost(o) > budget) continue;
            option_arcs_of[o].push_back(a);
            improved_probability_map[a] =
                std::max(improved_probability_map[a], p);
        }
    }
    std::vector<double> share(num_options, 0.0);
    for(std::size_t o = 0; o < num_options; ++o) {
        if(option_arcs_of[o].empty()) continue;
        std::vector<vertex_t> heads, tails;
        for(auto && a : option_arcs_of[o]) {
            heads.push_back(melon::arc_target(graph, a));
            tails.push_back(melon::arc_source(graph, a));
        }
        for(auto * v : {&heads, &tails}) {
            std::ranges::sort(*v);
            v->erase(std::ranges::unique(*v).begin(), v->end());
        }
        share[o] =
            1.0 / static_cast<double>(std::min(heads.size(), tails.size()));
    }

    // P̄_vt and P0_wt for every vertex
    auto improved_to_t = melon::create_vertex_map<double>(graph, 0.0);
    for(const auto & [v, p] : melon::dijkstra(reversed_traits{}, reversed_graph,
                                              improved_probability_map, t))
        improved_to_t[v] = p;
    auto base_to_t = melon::create_vertex_map<double>(graph, 0.0);
    for(const auto & [v, p] : melon::dijkstra(reversed_traits{}, reversed_graph,
                                              base_probability_map, t))
        base_to_t[v] = p;

    // λ = μ / B; any λ >= 0 is valid
    static constexpr std::array<double, 6> mus = {0.0, 0.5, 1.0, 2.0, 4.0, 8.0};
    struct reach_scratch {
        option_t free_option = 0;
        double lambda = 0.0;
        std::vector<double> best;
        std::vector<vertex_t> touched;
    };
    auto make_reach = [&](reach_scratch & sc) {
        auto tolled = melon::maps::function([&](const arc_t & e) {
            if(sc.lambda == 0.0) return improved_probability_map[e];
            double p = base_probability_map[e];
            for(auto && [ip, o] : improvements_map[e]) {
                if(instance.option_cost(o) > budget) continue;
                p = std::max(
                    p, o == sc.free_option
                           ? ip
                           : ip * std::exp(-sc.lambda *
                                           instance.option_cost(o) * share[o]));
            }
            return p;
        });
        return melon::dijkstra(reversed_traits{}, reversed_graph,
                               std::move(tolled));
    };
    std::size_t vertex_id_bound = 0;
    for(auto && v : melon::vertices(graph))
        vertex_id_bound = std::max(vertex_id_bound, std::size_t{v} + 1);
    // for every w reaching from and kept: the least over λ of
    // e^{λ B'} P_λ(w, from), o's improvements free; λ = 0 first, giving P̄
    auto budget_reach = [&](auto & algo, reach_scratch & sc,
                            const vertex_t from, const option_t o, auto && keep,
                            std::vector<std::pair<vertex_t, double>> & reach) {
        const double left =
            std::max(0.0, budget - instance.option_cost(o) + tolerance);
        sc.free_option = o;
        if(sc.best.size() < vertex_id_bound)
            sc.best.assign(vertex_id_bound, 0.0);
        reach.clear();
        for(const double mu : mus) {
            if(mu > 0 && budget <= 0) break;
            sc.lambda = mu > 0 ? mu / budget : 0.0;
            const double factor = std::exp(sc.lambda * left);
            algo.reset().add_source(from);
            for(const auto & [w, p] : algo) {
                if(mu == 0.0) {
                    if(!keep(w, p)) continue;
                    sc.touched.push_back(w);
                    sc.best[w] = p;
                } else if(sc.best[w] > 0)
                    sc.best[w] = std::min(sc.best[w], factor * p);
            }
        }
        for(const vertex_t w : sc.touched) {
            reach.emplace_back(w, sc.best[w]);
            sc.best[w] = 0.0;
        }
        sc.touched.clear();
    };

    // the knapsack of the other options' gains, o's own outside it
    auto bound = [&](const std::vector<std::pair<vertex_t, double>> & reach,
                     const option_t o) {
        auto option_gains = instance.template create_option_map<double>();
        option_gains.fill(0.0);
        double M = 0.0;
        for(const auto & [w, p] : reach) {
            M += p * source_quality_map[w];
            for(const auto & [gain, option] : source_options_map[w]) {
                if(option == o)
                    M += p * gain;
                else
                    option_gains[option] += p * gain;
            }
        }
        std::vector<option_t> items;
        for(auto && option : instance.options())
            if(option_gains[option] > 0) items.push_back(option);
        if(!items.empty()) {
            melon::knapsack_bnb knapsack(
                items, option_gains,
                [&instance](auto && option) {
                    return instance.option_cost(option);
                },
                budget - instance.option_cost(o) + tolerance);
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
            for(std::size_t i = r.begin(); i != r.end(); ++i) {
                const arc_t a = improved_arcs[i];
                std::size_t k = 0;
                for(auto && [ip, o] : improvements_map[a]) {
                    const double through =
                        ip * improved_to_t[melon::arc_target(graph, a)];
                    // keeps w within a relative 1e-9 of a tie
                    budget_reach(
                        algo, sc, melon::arc_source(graph, a), o,
                        [&](const vertex_t & w, const double p) {
                            return p * through >= base_to_t[w] * (1 - 1e-9);
                        },
                        reach);
                    arc_big_M_map[a][k++] = bound(reach, o);
                }
            }
        });

    // F_t with o bought, for the options of target_options: every source,
    // o's gains outside the knapsack
    std::vector<double> target_big_M(num_options, 0.0);
    {
        reach_scratch sc;
        auto algo = make_reach(sc);
        std::vector<std::pair<vertex_t, double>> reach;
        for(const option_t o : target_options) {
            budget_reach(
                algo, sc, t, o,
                [](const vertex_t &, const double) { return true; }, reach);
            target_big_M[o] = bound(reach, o);
        }
    }

    return std::make_pair(std::move(arc_big_M_map), std::move(target_big_M));
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_TIGHT_BIG_M_MAPS_HPP
