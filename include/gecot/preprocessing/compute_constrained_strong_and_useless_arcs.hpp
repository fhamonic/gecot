#ifndef GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/concurrent_dijkstras.hpp"
#include "melon/graph.hpp"

#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace gecot {

template <instance_c I, case_c C>
auto compute_constrained_strong_and_useless_arcs(
    const I & instance, const C & instance_case, const double budget,
    const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; },
    double probability_resolution = 0.00000001) {
    using graph_t = case_graph_t<C>;
    using log_probability_t = uint64_t;
    using log_probability_map_t = melon::arc_map_t<graph_t, log_probability_t>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    auto prob_to_length =
        [probability_resolution](const double p) -> log_probability_t {
        assert(p != 0);
        return static_cast<log_probability_t>(
            -std::log(std::max(p, probability_resolution)) /
            std::log1p(probability_resolution));
    };

    const auto & graph = instance_case.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();

    auto base_length_map = melon::create_arc_map<log_probability_t>(graph);
    for(const arc_t & a : arcs_range) {
        base_length_map[a] = prob_to_length(base_probability_map[a]);
    }

    auto arcs_options = instance.create_option_map(
        std::vector<std::pair<melon::arc_t<graph_t>, double>>{});
    compute_case_arc_options(instance_case, arcs_options);

    auto cost_map = melon::create_arc_map<double>(graph);
    const int nb_mus = 10;
    double max_mu = 0;
    for(auto && i : instance.options()) {
        const double arc_cost = instance.option_cost(i) /
                                static_cast<double>(arcs_options[i].size());
        for(auto && [a, enhanced_prob] : arcs_options[i]) {
            max_mu = std::max(
                max_mu, (base_length_map[a] - prob_to_length(enhanced_prob)) /
                            arc_cost);
        }
    }
    max_mu = 1;

    std::cout << "max_mu: " << max_mu << "\tbudget penalty: " << max_mu * budget
              << std::endl;

    std::vector<std::pair<double, log_probability_map_t>>
        lagrange_improved_length_maps;
    for(double mu = 0; mu <= max_mu;
        mu += max_mu / static_cast<double>(nb_mus - 1)) {
        // std::cout << "mu: " << mu << std::endl;
        auto mu_length_map = base_length_map;
        for(auto && i : instance.options()) {
            const double arc_cost = instance.option_cost(i) /
                                    static_cast<double>(arcs_options[i].size());
            const log_probability_t penalty = mu * arc_cost;
            for(auto && [a, enhanced_prob] : arcs_options[i]) {
                mu_length_map[a] = std::min(
                    mu_length_map[a], prob_to_length(enhanced_prob) + penalty);
            }
        }
        lagrange_improved_length_maps.emplace_back(mu,
                                                   std::move(mu_length_map));
    }

    auto compute_strong_arcs = [&](const tbb::blocked_range<
                                   decltype(arcs_range.begin())> & arcs_block) {
        arc_t uv;
        auto fiber_map = melon::create_arc_map<bool>(graph, false);
        auto sgraph =
            melon::views::subgraph(graph, {}, [&](const arc_t & a) -> bool {
                return !fiber_map[melon::arc_target(graph, a)] && a != uv;
            });
        auto algo = melon::concurrent_dijkstras(
            detail::useless_arc_default_traits<graph_t, log_probability_t>{},
            sgraph, base_length_map, base_length_map);
        for(const auto & a : arcs_block) {
            uv = a;
            fiber_map.fill(false);
            auto && u = melon::arc_source(graph, uv);
            auto && v = melon::arc_target(graph, uv);
            std::vector<std::pair<vertex_t, log_probability_t>> mu_fiber;
            for(auto && [mu, mu_length_map] : lagrange_improved_length_maps) {
                const log_probability_t budget_penalty = mu * budget;
                algo.reset();
                algo.set_red_length_map(mu_length_map);
                algo.add_red_source(u);
                algo.add_blue_source(v, base_length_map[uv] + budget_penalty);
                for(auto && [w, w_dist] : mu_fiber)
                    algo.add_blue_source(w, w_dist + budget_penalty);
                for(const auto & [w, w_dist] : algo) {
                    if(fiber_map[w]) continue;
                    fiber_map[w] = true;
                    strong_arcs_map[w].push_back(uv);
                    mu_fiber.emplace_back(w, w_dist.first - budget_penalty);
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
        arc_t uv;
        auto fiber_map = melon::create_arc_map<bool>(graph, false);
        auto sgraph =
            melon::views::subgraph(graph, {}, [&](const arc_t & a) -> bool {
                return !fiber_map[melon::arc_target(graph, a)] && a != uv;
            });
        auto algo = melon::concurrent_dijkstras(
            detail::useless_arc_default_traits<graph_t, log_probability_t>{},
            sgraph, base_length_map, base_length_map);
        for(const auto & a : arcs_block) {
            uv = a;
            fiber_map.fill(false);
            auto && u = melon::arc_source(graph, uv);
            auto && v = melon::arc_target(graph, uv);
            std::vector<std::pair<vertex_t, log_probability_t>> mu_fiber;
            for(auto && [mu, mu_length_map] : lagrange_improved_length_maps) {
                const log_probability_t budget_penalty = mu * budget;
                algo.reset();
                algo.set_red_length_map(mu_length_map);
                algo.add_blue_source(u, budget_penalty);
                algo.add_red_source(v, mu_length_map[uv]);
                for(auto && [w, w_dist] : mu_fiber)
                    algo.add_blue_source(w, w_dist + budget_penalty);
                for(const auto & [w, w_dist] : algo) {
                    if(fiber_map[w]) continue;
                    fiber_map[w] = true;
                    useless_arcs_map[w].push_back(uv);
                    mu_fiber.emplace_back(w, w_dist.first - budget_penalty);
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

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP