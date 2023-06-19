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
    const int nb_mus = 5;
    double max_mu = 2/budget;
    // double max_mu = 0;
    //     std::cout << "budget: " << budget << std::endl;
    // for(auto && i : instance.options()) {
    //     const double arc_cost = instance.option_cost(i) /
    //                             static_cast<double>(arcs_options[i].size());
    //     for(auto && [a, enhanced_prob] : arcs_options[i]) {
    //     std::cout << (std::log(enhanced_prob /
    //                              std::max(base_probability_map[a], 0.5)) /
    //                         arc_cost) << std::endl;
    //         max_mu = std::max(
    //             max_mu, std::log(enhanced_prob /
    //                              std::max(base_probability_map[a], 0.6)) /
    //                         arc_cost);
    //     }
    // }

    // std::cout << "max_mu: " << max_mu << std::endl;

    std::vector<std::pair<double, probability_map_t>>
        lagrange_improved_probability_maps;
    for(double mu = 0; mu <= max_mu;
        mu += max_mu / static_cast<double>(nb_mus - 1)) {
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

    // for(const auto & a : arcs_range) {
    //     auto && mu = lagrange_improved_probability_maps[90].first;
    //     const auto & mu_length_map =
    //         lagrange_improved_probability_maps[90].second;
    //     std::cout << "mu! : " << mu << std::endl;
    //     if(mu_length_map[a] > base_probability_map[a])
    //         std::cout << "caca! : " << mu_length_map[a] << " vs "
    //                   << base_probability_map[a] << std::endl;
    // }

    auto compute_strong_arcs = [&](const tbb::blocked_range<
                                   decltype(arcs_range.begin())> & arcs_block) {
        melon::strong_fiber<graph_t, probability_map_t, probability_map_t,
                            detail::strong_arc_default_traits<graph_t, double>>
            algo(graph, base_probability_map, base_probability_map);
        for(const auto & uv : arcs_block) {
            std::vector<std::pair<vertex_t, double>> mu_fiber;
            for(auto && [mu, mu_length_map] :
                lagrange_improved_probability_maps) {
                const double budget_modifier = get_modifier(mu, budget);
                std::cout << mu << " " << budget_modifier << std::endl;
                auto u = melon::arc_source(graph, uv);
                algo.reset()
                    .set_lower_length_map(mu_length_map)
                    .add_source(
                        u, melon::views::map([&uv](const arc_t & a) -> bool {
                            return a == uv;
                        }),
                        1.0, mu_length_map,
                        melon::views::map([&](const arc_t & a) {
                            return base_probability_map[a] * budget_modifier;
                        }));
                for(auto && [w, w_prob] : mu_fiber)
                    algo.add_source(w, melon::views::map([](auto && v) -> bool {
                                        return true;
                                    }),
                                    w_prob * budget_modifier);
                for(const auto & [w, w_prob] : algo) {
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

    auto compute_useless_arcs = [&](const tbb::blocked_range<
                                    decltype(arcs_range.begin())> &
                                        arcs_block) {
        melon::strong_fiber<graph_t, probability_map_t, probability_map_t,
                            detail::useless_arc_default_traits<graph_t, double>>
            algo(graph, base_probability_map, base_probability_map);
        for(const auto & uv : arcs_block) {
            std::vector<std::pair<vertex_t, double>> mu_fiber;
            for(auto && [mu, mu_length_map] :
                lagrange_improved_probability_maps) {
                const double budget_modifier = get_modifier(mu, budget);
                auto u = melon::arc_source(graph, uv);
                useless_arcs_map[u].push_back(uv);
                algo.reset()
                    .set_lower_length_map(mu_length_map)
                    .add_source(
                        u, melon::views::map([&uv](const arc_t & a) -> bool {
                            return a != uv;
                        }),
                        1.0, mu_length_map,
                        melon::views::map([&](const arc_t & a) {
                            return base_probability_map[a] * budget_modifier;
                        }));
                for(auto && [w, w_prob] : mu_fiber)
                    algo.add_source(w, melon::views::map([](auto && v) -> bool {
                                        return true;
                                    }),
                                    w_prob * budget_modifier);
                for(const auto & [w, w_prob] : algo) {
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

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP