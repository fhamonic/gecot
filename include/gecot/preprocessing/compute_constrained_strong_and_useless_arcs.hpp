#ifndef GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"
#include "melon/graph.hpp"

#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

namespace fhamonic {
namespace gecot {

template <instance_c I, case_c C>
auto compute_constrained_strong_and_useless_arcs(
    const I & instance, const C & instance_case, const double budget,
    const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; }) {
    using graph_t = case_graph_t<C>;
    using probability_map_t = case_probability_map_t<C>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    auto get_modifier = [](const double & mu, const double & cost) -> double {
        return std::exp(-mu * cost);
    };

    const auto & graph = instance_case.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();

    auto arcs_options = instance.create_option_map(
        std::vector<std::pair<melon::arc_t<graph_t>, double>>{});
    compute_case_arc_options(instance_case, arcs_options);

    auto cost_map = melon::create_arc_map<double>(graph);
    const int nb_mus = 10;
    // double max_mu = 30 / budget;
    double max_mu = 0;
    std::cout << "budget: " << budget << std::endl;
    for(auto && i : instance.options()) {
        const double arc_cost = instance.option_cost(i) /
                                static_cast<double>(arcs_options[i].size());
        for(auto && [a, enhanced_prob] : arcs_options[i]) {
            max_mu = std::max(
                max_mu, std::log(enhanced_prob /
                                 std::max(base_probability_map[a], 0.01)) /
                            arc_cost);
        }
    }

    std::cout << "max_mu: " << max_mu
              << "\tbudget modifier: " << get_modifier(max_mu, budget)
              << std::endl;

    std::vector<std::pair<double, probability_map_t>>
        lagrange_improved_probability_maps;
    for(double mu = 0; mu <= max_mu;
        mu += max_mu / static_cast<double>(nb_mus - 1)) {
        // std::cout << "mu: " << mu << std::endl;
        auto mu_length_map = base_probability_map;
        for(auto && i : instance.options()) {
            const double arc_cost = instance.option_cost(i) /
                                    static_cast<double>(arcs_options[i].size());
            const double modifier = get_modifier(mu, arc_cost);
            for(auto && [a, enhanced_prob] : arcs_options[i]) {
                mu_length_map[a] =
                    std::max(mu_length_map[a], modifier * enhanced_prob);
            }
        }
        lagrange_improved_probability_maps.emplace_back(
            mu, std::move(mu_length_map));
    }
    // for(double mu = max_mu;;) {
    //     mu -= max_mu / static_cast<double>(nb_mus - 1);
    //     if(mu <= 0) break;
    //     std::cout << "mu: " << mu << std::endl;
    //     auto mu_length_map = base_probability_map;
    //     for(auto && i : instance.options()) {
    //         const double arc_cost = instance.option_cost(i) /
    //                                 static_cast<double>(arcs_options[i].size());
    //         const double modifier = get_modifier(mu, arc_cost);
    //         for(auto && [a, enhanced_prob] : arcs_options[i]) {
    //             mu_length_map[a] =
    //                 std::max(mu_length_map[a], modifier * enhanced_prob);
    //         }
    //     }
    //     lagrange_improved_probability_maps.emplace_back(
    //         mu, std::move(mu_length_map));
    // }

    // for(const auto & a : arcs_range) {
    //     auto && mu = lagrange_improved_probability_maps[90].first;
    //     const auto & mu_length_map =
    //         lagrange_improved_probability_maps[90].second;
    //     std::cout << "mu! : " << mu << std::endl;
    //     if(mu_length_map[a] > base_probability_map[a])
    //         std::cout << "caca! : " << mu_length_map[a] << " vs "
    //                   << base_probability_map[a] << std::endl;
    // }

    auto compute_strong_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            arc_t uv;
            auto fiber_map = melon::create_arc_map<bool>(graph, false);
            auto sgraph =
                melon::views::subgraph(graph, {}, [&](const arc_t & a) -> bool {
                    return !fiber_map[melon::arc_target(graph, a)] && a != uv;
                });
            auto algo = melon::strong_fiber(
                detail::useless_arc_default_traits<graph_t, double>{}, sgraph,
                base_probability_map, base_probability_map);
            for(const auto & a : arcs_block) {
                uv = a;
                fiber_map.fill(false);
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                std::vector<std::pair<vertex_t, double>> mu_fiber;
                for(auto && [mu, mu_length_map] :
                    lagrange_improved_probability_maps) {
                    const double budget_modifier = get_modifier(mu, budget);
                    algo.reset();
                    algo.set_reduced_length_map(mu_length_map);
                    algo.relax_useless_vertex(u);
                    algo.relax_strong_vertex(
                        v, base_probability_map[uv] * budget_modifier);
                    for(auto && [w, w_prob] : mu_fiber)
                        algo.relax_strong_vertex(w, w_prob * budget_modifier);
                    for(const auto & [w, w_prob] : algo) {
                        if(fiber_map[w]) continue;
                        if(mu > 0)
                            std::cout << mu << "\t" << budget_modifier << "\n";
                        fiber_map[w] = true;
                        strong_arcs_map[w].push_back(uv);
                        mu_fiber.emplace_back(w, w_prob / budget_modifier);
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

    auto compute_useless_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            arc_t uv;
            auto fiber_map = melon::create_arc_map<bool>(graph, false);
            auto sgraph =
                melon::views::subgraph(graph, {}, [&](const arc_t & a) -> bool {
                    return !fiber_map[melon::arc_target(graph, a)] && a != uv;
                });
            auto algo = melon::strong_fiber(
                detail::useless_arc_default_traits<graph_t, double>{}, sgraph,
                base_probability_map, base_probability_map);
            for(const auto & a : arcs_block) {
                uv = a;
                fiber_map.fill(false);
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                useless_arcs_map[u].push_back(uv);
                std::vector<std::pair<vertex_t, double>> mu_fiber;
                for(auto && [mu, mu_length_map] :
                    lagrange_improved_probability_maps) {
                    const double budget_modifier = get_modifier(mu, budget);
                    algo.reset();
                    algo.set_reduced_length_map(mu_length_map);
                    algo.relax_strong_vertex(u, budget_modifier);
                    algo.relax_useless_vertex(v, mu_length_map[uv]);
                    for(auto && [w, w_prob] : mu_fiber)
                        algo.relax_strong_vertex(w, w_prob * budget_modifier);
                    for(const auto & [w, w_prob] : algo) {
                        if(fiber_map[w]) continue;
                        if(mu > 0)
                            std::cout << mu << "\t" << budget_modifier << "\n";
                        fiber_map[w] = true;
                        useless_arcs_map[w].push_back(uv);
                        mu_fiber.emplace_back(w, w_prob / budget_modifier);
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