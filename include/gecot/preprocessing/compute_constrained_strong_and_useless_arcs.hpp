#ifndef GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP

#include <atomic>
#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/competing_dijkstras.hpp"
#include "melon/graph.hpp"

#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

#include "io_helper.hpp"

namespace fhamonic {
namespace gecot {

template <instance_c I, case_c C>
auto compute_constrained_strong_and_useless_arcs(
    const I & instance, const C & instance_case, const double budget,
    const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; },
    double probability_resolution = 0.00000001, int num_mus = 10) {
    using graph_t = case_graph_t<C>;
    using log_probability_t = uint64_t;
    using log_probability_map_t = melon::arc_map_t<graph_t, log_probability_t>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    spdlog::stopwatch prep_sw;
    spdlog::trace("Lagrangian preprocessing of the '{}' graph:",
                  instance_case.name());

    auto prob_to_length =
        [probability_resolution](const double p) -> log_probability_t {
        assert(p != 0);
        return static_cast<log_probability_t>(
            -std::log(std::max(p, probability_resolution)) /
            std::log1p(probability_resolution));
    };
    auto cost_to_penatlty = [](const double mu,
                               const double cost) -> log_probability_t {
        return static_cast<log_probability_t>(mu * cost);
    };

    const auto & graph = instance_case.graph();
    const auto & quality_map = instance_case.vertex_quality_map();

    auto useless_vertices_map = melon::create_vertex_map<bool>(graph);
    for(const auto & v : melon::vertices(graph)) {
        useless_vertices_map[v] =
            (quality_map[v] == 0 &&
             instance_case.vertex_options_map()[v].empty());
    }

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
    double min_mu = std::numeric_limits<double>::max();
    double max_mu = 0;
    for(auto && i : instance.options()) {
        const double arc_cost = instance.option_cost(i) /
                                static_cast<double>(arcs_options[i].size());
        for(auto && [a, enhanced_prob] : arcs_options[i]) {
            min_mu = std::min(min_mu, 1 / arc_cost);
            if(base_length_map[a] > prob_to_length(enhanced_prob))
                max_mu = std::max(
                    max_mu, static_cast<double>(base_length_map[a] -
                                                prob_to_length(enhanced_prob)) /
                                arc_cost);
        }
    }

    double geometrical_ratio = std::pow(max_mu / min_mu, 1.0 / num_mus);
    spdlog::trace("Min µ: {:.4e}, Max µ: {:.4e}, common ratio: {:.6g}", min_mu,
                  max_mu, geometrical_ratio);

    std::vector<double> mus;
    mus.push_back(0.0);
    for(double mu = min_mu; mu <= max_mu; mu *= geometrical_ratio)
        mus.push_back(mu);

    std::vector<std::atomic<std::size_t>> stong_counters(mus.size());
    std::vector<std::atomic<std::size_t>> useless_counters(mus.size());
    std::vector<std::pair<double, log_probability_map_t>>
        lagrange_improved_length_maps;
    for(double mu : mus) {
        auto mu_length_map = base_length_map;
        for(auto && i : instance.options()) {
            const double arc_cost = instance.option_cost(i) /
                                    static_cast<double>(arcs_options[i].size());
            const log_probability_t penalty = cost_to_penatlty(mu, arc_cost);
            for(auto && [a, enhanced_prob] : arcs_options[i]) {
                mu_length_map[a] = std::min(
                    mu_length_map[a], prob_to_length(enhanced_prob) + penalty);
            }
        }
        lagrange_improved_length_maps.emplace_back(mu,
                                                   std::move(mu_length_map));
    }

    {
        progress_bar<spdlog::level::trace, 50> pb(2 * arcs_range.size());

        auto compute_strong_arcs = [&](const tbb::blocked_range<
                                       decltype(arcs_range.begin())> &
                                           arcs_block) {
            arc_t uv;
            auto fiber_map = melon::create_arc_map<bool>(graph, false);
            auto sgraph =
                melon::views::subgraph(graph, {}, [&](const arc_t & a) -> bool {
                    return !fiber_map[melon::arc_target(graph, a)] && a != uv;
                });
            auto algo = melon::competing_dijkstras(
                detail::useless_arc_default_traits<graph_t,
                                                   log_probability_t>{},
                sgraph, base_length_map, base_length_map);
            for(const auto & a : arcs_block) {
                uv = a;
                fiber_map.fill(false);
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                std::size_t previous_fiber_size = 0;
                std::vector<std::pair<vertex_t, log_probability_t>> mu_fiber;
                for(auto i : std::views::iota(std::size_t(0), mus.size())) {
                    auto && [mu, mu_length_map] =
                        lagrange_improved_length_maps[i];
                    const log_probability_t budget_penalty =
                        cost_to_penatlty(mu, budget);
                    algo.reset();
                    algo.set_red_length_map(mu_length_map);
                    algo.add_red_source(u);
                    algo.add_blue_source(v,
                                         base_length_map[uv] + budget_penalty);
                    for(auto && [w, w_dist] : mu_fiber)
                        algo.add_blue_source(w, w_dist + budget_penalty);
                    for(const auto & [w, w_dist] : algo) {
                        if(fiber_map[w]) continue;
                        fiber_map[w] = true;
                        mu_fiber.emplace_back(w, w_dist.first - budget_penalty);
                        if(useless_vertices_map[w]) continue;
                        strong_arcs_map[w].push_back(uv);
                    }
                    if(mu_fiber.size() == previous_fiber_size) continue;
                    stong_counters[i] += mu_fiber.size() - previous_fiber_size;
                    previous_fiber_size = mu_fiber.size();
                }
                pb.tick();
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
            auto algo = melon::competing_dijkstras(
                detail::useless_arc_default_traits<graph_t,
                                                   log_probability_t>{},
                sgraph, base_length_map, base_length_map);
            for(const auto & a : arcs_block) {
                uv = a;
                fiber_map.fill(false);
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                std::size_t previous_fiber_size = 0;
                std::vector<std::pair<vertex_t, log_probability_t>> mu_fiber;
                for(auto i : std::views::iota(std::size_t(0), mus.size())) {
                    auto && [mu, mu_length_map] =
                        lagrange_improved_length_maps[i];
                    const log_probability_t budget_penalty =
                        cost_to_penatlty(mu, budget);
                    algo.reset();
                    algo.set_red_length_map(mu_length_map);
                    algo.add_blue_source(u, budget_penalty);
                    algo.add_red_source(v, mu_length_map[uv]);
                    for(auto && [w, w_dist] : mu_fiber)
                        algo.add_blue_source(w, w_dist + budget_penalty);
                    for(const auto & [w, w_dist] : algo) {
                        if(fiber_map[w]) continue;
                        fiber_map[w] = true;
                        mu_fiber.emplace_back(w, w_dist.first - budget_penalty);
                        if(useless_vertices_map[w]) continue;
                        useless_arcs_map[w].push_back(uv);
                    }
                    if(mu_fiber.size() == previous_fiber_size) continue;
                    useless_counters[i] +=
                        mu_fiber.size() - previous_fiber_size;
                    previous_fiber_size = mu_fiber.size();
                }

                pb.tick();
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
    }
    if(spdlog::get_level() == spdlog::level::trace) {
        for(auto i : std::views::iota(std::size_t(1), mus.size())) {
            stong_counters[i] += stong_counters[i - 1];
            useless_counters[i] += useless_counters[i - 1];
        }
        spdlog::trace("------------------------------------------");
        spdlog::trace("  µ value  | strong fiber | useless fiber ");
        spdlog::trace("           |  (avg size)  |  (avg size)");
        spdlog::trace("------------------------------------------");
        const auto num_arcs = melon::num_arcs(graph);
        for(auto i : std::views::iota(std::size_t(0), mus.size())) {
            if(i > 0 &&
               stong_counters[i].load() == stong_counters[i - 1].load() &&
               useless_counters[i].load() == useless_counters[i - 1].load())
                continue;
            spdlog::trace(" {:>.3e} | {:>12.3f} | {:>13.3f}  ", mus[i],
                          static_cast<double>(stong_counters[i]) /
                              static_cast<double>(num_arcs),
                          static_cast<double>(useless_counters[i]) /
                              static_cast<double>(num_arcs));
        }
        spdlog::trace("------------------------------------------");

        std::size_t num_strong, num_useless, num_sinks;
        num_strong = num_useless = num_sinks = 0;
        for(auto && v : melon::vertices(graph)) {
            if(useless_vertices_map[v]) continue;
            num_strong += strong_arcs_map[v].size();
            num_useless += useless_arcs_map[v].size();
            ++num_sinks;
        }
        spdlog::trace(
            "  {:>10.2f} strong arcs on average",
            static_cast<double>(num_strong) / static_cast<double>(num_sinks));
        spdlog::trace(
            "  {:>10.2f} useless arcs on average",
            static_cast<double>(num_useless) / static_cast<double>(num_sinks));
        spdlog::trace("          (took {} ms)",
                      std::chrono::duration_cast<std::chrono::milliseconds>(
                          prep_sw.elapsed())
                          .count());
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP