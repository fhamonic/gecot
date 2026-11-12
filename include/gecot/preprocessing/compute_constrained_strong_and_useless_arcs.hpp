#ifndef GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <utility>

#include <fmt/ranges.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/competing_dijkstras.hpp"
#include "melon/graph.hpp"

#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

#include "gecot/utils/log_table.hpp"
#include "gecot/utils/progress_bar.hpp"

namespace gecot {

template <instance_c I, case_c C>
auto compute_constrained_strong_and_useless_arcs(
    const I & instance, const C & instance_case, const double budget,
    const auto & option_predicate = [](const option_t & o) { return true; },
    double probability_resolution = 1e-8, int num_mus = 10,
    bool decades_sweep = true, const bool all_targets = false) {
    using graph_t = case_graph_t<C>;
    using log_probability_t = uint64_t;
    using log_probability_map_t = melon::arc_map_t<graph_t, log_probability_t>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    detail::check_resolution(instance_case.graph(), probability_resolution);
    spdlog::stopwatch prep_sw;
    spdlog::info("Preprocessing the '{}' graph, budget-aware",
                 instance_case.name());

    auto prob_to_length =
        [probability_resolution](const double p) -> log_probability_t {
        return static_cast<log_probability_t>(
            -std::log(std::max(p, probability_resolution)) /
            std::log1p(probability_resolution));
    };
    auto cost_to_penatlty = [](const double mu,
                               const double cost) -> log_probability_t {
        return static_cast<log_probability_t>(mu * cost);
    };
    // two steps down, which outweigh the rounding of mu * budget: along a path,
    // the shares of a feasible set must not sum past the budget penalty. One
    // unit over lets blue win a strong tie it should lose, and the useless loop
    // label an arc on a shortest path
    auto arc_cost_to_penalty = [](const double mu,
                                  const double arc_cost) -> log_probability_t {
        return static_cast<log_probability_t>(
            std::nextafter(mu * std::nextafter(arc_cost, 0.0), 0.0));
    };

    const auto & graph = instance_case.graph();
    const auto & target_quality_map = instance_case.target_quality_map();

    auto useless_target_vertices_map = melon::create_vertex_map<bool>(graph);
    for(const auto & v : melon::vertices(graph)) {
        useless_target_vertices_map[v] =
            !all_targets &&
            (target_quality_map[v] == 0 &&
             std::ranges::none_of(instance_case.vertex_options_map()[v],
                                  [](auto && e) {
                                      auto && [sqg, tqg, o] = e;
                                      return tqg > 0;
                                  }));
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
    const auto zero_cycle_arcs_map =
        detail::zero_cycle_arcs_map(graph, base_length_map);

    auto arcs_options = instance.create_option_map(
        std::vector<std::pair<melon::arc_t<graph_t>, double>>{});
    compute_case_arc_options(instance_case, arcs_options);

    auto cost_map = melon::create_arc_map<double>(graph);
    double min_mu = std::numeric_limits<double>::max();
    double max_mu = 0;
    double max_arc_cost = 0;
    for(auto && i : instance.options()) {
        if(!option_predicate(i) || arcs_options[i].empty()) continue;
        const double arc_cost = instance.option_cost(i) /
                                static_cast<double>(arcs_options[i].size());
        max_arc_cost = std::max(max_arc_cost, arc_cost);
        for(auto && [a, enhanced_prob] : arcs_options[i]) {
            min_mu = std::min(min_mu, 1 / arc_cost);
            if(base_length_map[a] > prob_to_length(enhanced_prob))
                max_mu = std::max(
                    max_mu, static_cast<double>(base_length_map[a] -
                                                prob_to_length(enhanced_prob)) /
                                arc_cost);
        }
    }

    // a blue seed of 2^62 outweighs any path, so larger multipliers add no
    // label; past 2^64 the uint64 casts of mu * budget and of mu * arc_cost
    // are undefined, and a predicate may admit arc_cost > budget
    max_mu = std::min(max_mu, 0x1p62 / std::max({budget, max_arc_cost, 1.0}));
    if(decades_sweep) min_mu = std::max(min_mu, max_mu / 1000.0);
    double geometrical_ratio = std::pow(max_mu / min_mu, 1.0 / num_mus);
    spdlog::debug("Min µ: {:.4e}, Max µ: {:.4e}, common ratio: {:.6g}", min_mu,
                  max_mu, geometrical_ratio);

    std::vector<double> mus;
    mus.push_back(0.0);
    // by index: repeated products overshoot max_mu by an ulp and skip it, and
    // never end when the ratio rounds to 1. The exponent k / num_mus, unlike
    // pow(geometrical_ratio, k), makes the grid a bitwise subset of the grid
    // of 2 * num_mus, so doubling num_mus never loses a label
    for(int k = 0; k <= num_mus; ++k) {
        const double mu =
            k < num_mus ? min_mu * std::pow(max_mu / min_mu,
                                            static_cast<double>(k) / num_mus)
                        : max_mu;
        if(mus.back() < mu && mu <= max_mu) mus.push_back(mu);
    }
    // µ = 0 again, seeded by the fiber of the positive multipliers: mus is
    // the test order, not sorted
    if(decades_sweep && mus.size() > 1) mus.push_back(0.0);
    spdlog::debug("µ sequence: {:.3e}", fmt::join(mus, " "));

    std::vector<std::atomic<std::size_t>> stong_counters(mus.size());
    std::vector<std::atomic<std::size_t>> useless_counters(mus.size());
    std::vector<std::pair<double, log_probability_map_t>>
        lagrange_improved_length_maps;
    for(double mu : mus) {
        auto mu_length_map = base_length_map;
        for(auto && i : instance.options()) {
            if(!option_predicate(i) || arcs_options[i].empty()) continue;
            const double arc_cost = instance.option_cost(i) /
                                    static_cast<double>(arcs_options[i].size());
            const log_probability_t penalty = arc_cost_to_penalty(mu, arc_cost);
            for(auto && [a, enhanced_prob] : arcs_options[i]) {
                mu_length_map[a] = std::min(
                    mu_length_map[a], prob_to_length(enhanced_prob) + penalty);
            }
        }
        lagrange_improved_length_maps.emplace_back(mu,
                                                   std::move(mu_length_map));
    }

    // every out-arc of w but uv enters the fiber: as a blue seed w relaxes
    // nothing, for good since the fiber only grows. Dropping such seeds makes
    // mu_fiber smaller than the fiber
    auto dead_seed = [&graph](const arc_t & uv, const auto & fiber_map,
                              const vertex_t & w) {
        return std::ranges::all_of(
            melon::out_arcs(graph, w), [&](const arc_t & b) {
                return b == uv || fiber_map[melon::arc_target(graph, b)];
            });
    };

    {
        progress_bar<spdlog::level::info, 64> pb(2 * arcs_range.size());

        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                    arcs_block) {
                arc_t uv;
                auto fiber_map = melon::create_vertex_map<bool>(graph, false);
                auto sgraph = melon::views::subgraph(
                    graph, {}, [&](const arc_t & a) -> bool {
                        return !fiber_map[melon::arc_target(graph, a)] &&
                               a != uv;
                    });
                auto algo = melon::competing_dijkstras(
                    detail::strong_arc_default_traits<graph_t,
                                                      log_probability_t>{},
                    sgraph, base_length_map, base_length_map);
                for(const auto & a : arcs_block) {
                    uv = a;
                    fiber_map.fill(false);
                    auto && u = melon::arc_source(graph, uv);
                    auto && v = melon::arc_target(graph, uv);
                    std::vector<std::pair<vertex_t, log_probability_t>>
                        mu_fiber;
                    for(auto i : std::views::iota(std::size_t(0), mus.size())) {
                        auto && [mu, mu_length_map] =
                            lagrange_improved_length_maps[i];
                        const log_probability_t budget_penalty =
                            cost_to_penatlty(mu, budget);
                        algo.reset();
                        algo.set_red_length_map(mu_length_map);
                        algo.add_red_source(u);
                        // before the fiber seeds: each add_*_source settles
                        // the red vertices closer than every blue seed pushed
                        // so far, and this seed is the closest. Red wins the
                        // ties of zero-length cycles, as in
                        // compute_strong_and_useless_arcs; the fiber seeds
                        // inherit the unit
                        algo.add_blue_source(v, base_length_map[uv] +
                                                    zero_cycle_arcs_map[uv] +
                                                    budget_penalty);
                        std::erase_if(mu_fiber, [&](const auto & p) {
                            return dead_seed(uv, fiber_map, p.first);
                        });
                        for(auto && [w, w_dist] : mu_fiber)
                            if(w != v)
                                algo.add_blue_source(w,
                                                     w_dist + budget_penalty);
                        std::size_t fiber_gain = 0;
                        for(const auto & [w, w_dist] : algo) {
                            if(fiber_map[w]) continue;
                            fiber_map[w] = true;
                            ++fiber_gain;
                            mu_fiber.emplace_back(
                                w, w_dist.first - budget_penalty);
                            if(useless_target_vertices_map[w]) continue;
                            strong_arcs_map[w].push_back(uv);
                        }
                        if(fiber_gain > 0) stong_counters[i] += fiber_gain;
                    }
                    pb.tick();
                }
            });

        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                    arcs_block) {
                arc_t uv;
                auto fiber_map = melon::create_vertex_map<bool>(graph, false);
                auto sgraph = melon::views::subgraph(
                    graph, {}, [&](const arc_t & a) -> bool {
                        return !fiber_map[melon::arc_target(graph, a)] &&
                               a != uv;
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
                    std::vector<std::pair<vertex_t, log_probability_t>>
                        mu_fiber;
                    for(auto i : std::views::iota(std::size_t(0), mus.size())) {
                        // v blue means red lost its only source: the fiber
                        // already holds every vertex u reaches
                        if(fiber_map[v]) break;
                        auto && [mu, mu_length_map] =
                            lagrange_improved_length_maps[i];
                        const log_probability_t budget_penalty =
                            cost_to_penatlty(mu, budget);
                        algo.reset();
                        algo.set_red_length_map(mu_length_map);
                        // before the fiber seeds, as in the strong loop
                        algo.add_blue_source(u, budget_penalty);
                        algo.add_red_source(v, mu_length_map[uv]);
                        std::erase_if(mu_fiber, [&](const auto & p) {
                            return dead_seed(uv, fiber_map, p.first);
                        });
                        for(auto && [w, w_dist] : mu_fiber)
                            if(w != u)
                                algo.add_blue_source(w,
                                                     w_dist + budget_penalty);
                        std::size_t fiber_gain = 0;
                        for(const auto & [w, w_dist] : algo) {
                            if(fiber_map[w]) continue;
                            fiber_map[w] = true;
                            ++fiber_gain;
                            mu_fiber.emplace_back(
                                w, w_dist.first - budget_penalty);
                            if(useless_target_vertices_map[w]) continue;
                            useless_arcs_map[w].push_back(uv);
                        }
                        if(fiber_gain > 0) useless_counters[i] += fiber_gain;
                    }

                    pb.tick();
                }
            });
    }
    // the parallel loops push in thread order: sorted, the contracted graphs
    // and the models built on them are the same from run to run
    for(auto && v : melon::vertices(graph)) {
        std::sort(strong_arcs_map[v].begin(), strong_arcs_map[v].end());
        std::sort(useless_arcs_map[v].begin(), useless_arcs_map[v].end());
    }
    if(spdlog::should_log(spdlog::level::debug)) {
        for(auto i : std::views::iota(std::size_t(1), mus.size())) {
            stong_counters[i] += stong_counters[i - 1];
            useless_counters[i] += useless_counters[i - 1];
        }
        log_table<spdlog::level::debug, 0, 9, 12, 13> fibers(
            {"µ value", log_format::scientific(3)},
            {"strong fiber (avg. size)", log_format::fixed(3)},
            {"useless fiber (avg. size)", log_format::fixed(3)});
        const auto num_arcs = melon::num_arcs(graph);
        for(auto i : std::views::iota(std::size_t(0), mus.size())) {
            if(i > 0 &&
               stong_counters[i].load() == stong_counters[i - 1].load() &&
               useless_counters[i].load() == useless_counters[i - 1].load())
                continue;
            fibers.row(mus[i],
                       static_cast<double>(stong_counters[i]) /
                           static_cast<double>(num_arcs),
                       static_cast<double>(useless_counters[i]) /
                           static_cast<double>(num_arcs));
        }

        std::size_t num_strong, num_useless, num_sinks;
        num_strong = num_useless = num_sinks = 0;
        for(auto && v : melon::vertices(graph)) {
            if(useless_target_vertices_map[v]) continue;
            num_strong += strong_arcs_map[v].size();
            num_useless += useless_arcs_map[v].size();
            ++num_sinks;
        }
        spdlog::debug(
            "  {:>10.2f} strong arcs on average",
            static_cast<double>(num_strong) / static_cast<double>(num_sinks));
        spdlog::debug(
            "  {:>10.2f} useless arcs on average",
            static_cast<double>(num_useless) / static_cast<double>(num_sinks));
        spdlog::debug("          (took {} ms)",
                      std::chrono::duration_cast<std::chrono::milliseconds>(
                          prep_sw.elapsed())
                          .count());
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_CONSTRAINED_STRONG_AND_USELESS_ARCS_HPP