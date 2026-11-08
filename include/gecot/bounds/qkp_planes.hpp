#ifndef GECOT_BOUNDS_QKP_PLANES_HPP
#define GECOT_BOUNDS_QKP_PLANES_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/bounds/bound_report.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/bounds/lp_knapsack.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/indices/pc_num.hpp"

namespace gecot {

// The case probabilities with every option bought, of cost at most budget.
template <case_c C, instance_c I>
std::vector<double> improved_probability_map(const C & instance_case,
                                             const I & instance,
                                             double budget) {
    std::vector<double> probabilities(
        instance_case.arc_probability_map().begin(),
        instance_case.arc_probability_map().end());
    const auto & arc_options_map = instance_case.arc_options_map();
    for(auto && a : melon::arcs(instance_case.graph()))
        for(auto && [improved_probability, option] : arc_options_map[a])
            if(instance.option_cost(option) <= budget)
                probabilities[a] =
                    std::max(probabilities[a], improved_probability);
    return probabilities;
}

// Upper planes of the quadratic knapsack that bounds the weighted case
// scores. Every probability P(X) is at most P̄, its value with every option
// bought, and the qualities are the base ones q plus the gains G x, so
//   Σ_k w_k pc_k(X) <= H + l·x + xᵀQx,
//   H = Σ_k w_k q^s P̄ q^t,  l_o = Σ_k w_k (g^s_o P̄ q^t + q^s P̄ g^t_o),
//   Q_oo' = Σ_k w_k g^s_o P̄ g^t_o'.
// Splitting Q_oo' + Q_o'o into θ_oo' + θ_o'o with θ >= 0 bounds row o by
//   u_o = l_o + Q_oo + max{Σ_o' θ_oo' y_o' : c·y <= B - c_o, y_o = 0}
// (Caprara, Pisinger and Toth 1999), and the whole by H + max{u·x : c·x <= B},
// both knapsacks by their LP relaxations.
struct qkp_planes {
    enum class theta_start { best, half, target, source };
    theta_start start = theta_start::best;
    // projected subgradient steps on θ after the start
    std::size_t iterations = 0;
    // the weighted score the steps aim at, 0.9 times the bound if not given
    std::optional<double> target;

    struct weighted_result {
        double bound;
        // per option: the bound u_o on the gains of buying it, 0 without gains
        std::vector<double> option_bounds;
    };

    template <instance_c I>
    weighted_result weighted(
        const I & instance, const double budget,
        std::span<const double> case_weights,
        std::optional<double> weighted_target = std::nullopt) const {
        using graph_t = instance_graph_t<I>;
        using vertex_t = melon::vertex_t<graph_t>;
        const std::size_t num_options = std::ranges::size(instance.options());
        constexpr std::size_t no_item = std::numeric_limits<std::size_t>::max();

        // the items: options with a quality gain in a weighted case
        std::vector<std::size_t> item_of(num_options, no_item);
        std::vector<option_t> items;
        for(auto && instance_case : instance.cases()) {
            if(case_weights[instance_case.id()] <= 0) continue;
            const auto & vertex_options_map =
                instance_case.vertex_options_map();
            for(auto && v : melon::vertices(instance_case.graph()))
                for(auto && [source_gain, target_gain, option] :
                    vertex_options_map[v])
                    if((source_gain > 0 || target_gain > 0) &&
                       instance.option_cost(option) <= budget &&
                       item_of[option] == no_item) {
                        item_of[option] = items.size();
                        items.push_back(option);
                    }
        }
        const std::size_t n = items.size();
        std::vector<double> costs(n);
        for(std::size_t i = 0; i < n; ++i)
            costs[i] = instance.option_cost(items[i]);
        spdlog::debug(
            "qkp_planes: {} options with quality gains, {:.0f} MB of dense "
            "matrices",
            n,
            static_cast<double>(n * n) * sizeof(double) *
                (iterations > 0 ? 3 : 1) / 1e6);

        double H = 0.0;
        std::vector<double> l(n, 0.0), Q(n * n, 0.0);
        for(auto && instance_case : instance.cases()) {
            const double w = case_weights[instance_case.id()];
            if(w <= 0) continue;
            const auto & graph = instance_case.graph();
            const auto & source_quality_map =
                instance_case.source_quality_map();
            const auto & target_quality_map =
                instance_case.target_quality_map();
            const auto probabilities =
                improved_probability_map(instance_case, instance, budget);
            H += w * parallel_pc_num(graph, source_quality_map,
                                     target_quality_map, probabilities);

            // per item its (vertex, gain) and per vertex its (item, gain),
            // the gains of a vertex merged by SCC contraction summed
            std::vector<std::vector<std::pair<vertex_t, double>>> item_sources(
                n),
                item_targets(n);
            auto vertex_targets = melon::create_vertex_map<
                std::vector<std::pair<std::size_t, double>>>(graph, {});
            auto add = []<typename K>(std::vector<std::pair<K, double>> & list,
                                      K key, double gain) {
                for(auto & [k, g] : list)
                    if(k == key) {
                        g += gain;
                        return;
                    }
                list.emplace_back(key, gain);
            };
            const auto & vertex_options_map =
                instance_case.vertex_options_map();
            for(auto && v : melon::vertices(graph))
                for(auto && [source_gain, target_gain, option] :
                    vertex_options_map[v]) {
                    const std::size_t i = item_of[option];
                    if(i == no_item) continue;
                    if(source_gain > 0) add(item_sources[i], v, source_gain);
                    if(target_gain > 0) {
                        add(item_targets[i], v, target_gain);
                        add(vertex_targets[v], i, target_gain);
                    }
                }

            // each row by one task: no write is shared
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, n),
                [&](const tbb::blocked_range<std::size_t> & rows) {
                    auto forward = melon::dijkstra(
                        detail::pc_num_dijkstra_traits<graph_t, double>{},
                        graph, probabilities);
                    auto reversed_graph = melon::views::reverse(graph);
                    auto backward = melon::dijkstra(
                        detail::pc_num_dijkstra_traits<decltype(reversed_graph),
                                                       double>{},
                        reversed_graph, probabilities);
                    for(std::size_t i = rows.begin(); i != rows.end(); ++i) {
                        double * Q_row = Q.data() + i * n;
                        for(auto && [s, source_gain] : item_sources[i]) {
                            double flow = 0.0;
                            forward.reset().add_source(s);
                            for(const auto & [t, probability] : forward) {
                                flow += target_quality_map[t] * probability;
                                for(auto && [j, target_gain] :
                                    vertex_targets[t])
                                    Q_row[j] += w * source_gain * probability *
                                                target_gain;
                            }
                            l[i] += w * source_gain * flow;
                        }
                        for(auto && [t, target_gain] : item_targets[i]) {
                            double flow = 0.0;
                            backward.reset().add_source(t);
                            for(const auto & [s, probability] : backward)
                                flow += source_quality_map[s] * probability;
                            l[i] += w * target_gain * flow;
                        }
                    }
                });
        }

        weighted_result result{H, std::vector<double>(num_options, 0.0)};
        if(n == 0) {
            spdlog::trace("qkp_planes: no quality gains, the bound is H = {}",
                          H);
            return result;
        }

        // θ by an accessor theta(i, j); u, x and y, if given, receive the row
        // bounds and the LP solutions of the outer and the row knapsacks
        auto evaluate = [&](auto && theta, std::vector<double> & u,
                            std::vector<double> * x, std::vector<double> * y) {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, n),
                [&](const tbb::blocked_range<std::size_t> & rows) {
                    std::vector<double> row(n);
                    for(std::size_t i = rows.begin(); i != rows.end(); ++i) {
                        for(std::size_t j = 0; j < n; ++j)
                            row[j] = (j == i) ? 0.0 : theta(i, j);
                        u[i] = l[i] + Q[i * n + i] +
                               lp_knapsack(
                                   row, costs, budget - costs[i],
                                   y ? std::span<double>(y->data() + i * n, n)
                                     : std::span<double>{});
                    }
                });
            return H +
                   lp_knapsack(u, costs, budget,
                               x ? std::span<double>(*x) : std::span<double>{});
        };
        auto half = [&](std::size_t i, std::size_t j) {
            return (Q[i * n + j] + Q[j * n + i]) / 2;
        };
        auto as_target = [&](std::size_t i, std::size_t j) {
            return Q[j * n + i];
        };
        auto as_source = [&](std::size_t i, std::size_t j) {
            return Q[i * n + j];
        };

        std::vector<double> u(n), best_u;
        double best = std::numeric_limits<double>::infinity();
        theta_start best_start = theta_start::half;
        auto try_start = [&](theta_start s, auto && theta, const char * name) {
            const double b = evaluate(theta, u, nullptr, nullptr);
            spdlog::trace("qkp_planes: θ {}: bound {}", name, b);
            if(b < best) {
                best = b;
                best_u = u;
                best_start = s;
            }
        };
        if(start == theta_start::best || start == theta_start::half)
            try_start(theta_start::half, half, "half");
        if(start == theta_start::best || start == theta_start::target)
            try_start(theta_start::target, as_target, "as target");
        if(start == theta_start::best || start == theta_start::source)
            try_start(theta_start::source, as_source, "as source");

        if(iterations > 0) {
            std::vector<double> theta(n * n);
            for(std::size_t i = 0; i < n; ++i)
                for(std::size_t j = 0; j < n; ++j)
                    theta[i * n + j] =
                        best_start == theta_start::half     ? half(i, j)
                        : best_start == theta_start::target ? as_target(i, j)
                                                            : as_source(i, j);
            auto stored = [&](std::size_t i, std::size_t j) {
                return theta[i * n + j];
            };
            std::vector<double> x(n), y(n * n);
            const double aim = weighted_target.value_or(0.9 * best);
            double step = 2.0;
            std::size_t stall = 0;
            for(std::size_t iteration = 0; iteration < iterations;
                ++iteration) {
                const double b = evaluate(stored, u, &x, &y);
                if(b < best - 1e-9 * std::abs(best)) {
                    best = b;
                    best_u = u;
                    stall = 0;
                } else if(++stall >= 10) {
                    step /= 2;
                    stall = 0;
                }
                // dbound/dθ_ij on the split θ_ij + θ_ji = M_ij
                double norm = 0.0;
                for(std::size_t i = 0; i < n; ++i)
                    for(std::size_t j = 0; j < n; ++j)
                        if(i != j) {
                            const double g =
                                x[i] * y[i * n + j] - x[j] * y[j * n + i];
                            norm += g * g;
                        }
                const double t = step * (b - aim) / norm;
                // NaN θ would drop pair terms from the rows
                if(norm == 0 || !std::isfinite(t)) break;
                for(std::size_t i = 0; i < n; ++i)
                    for(std::size_t j = i + 1; j < n; ++j) {
                        const double M = Q[i * n + j] + Q[j * n + i];
                        const double g =
                            x[i] * y[i * n + j] - x[j] * y[j * n + i];
                        const double a =
                            std::clamp(theta[i * n + j] - t * g, 0.0, M);
                        const double c =
                            std::clamp(theta[j * n + i] + t * g, 0.0, M);
                        theta[i * n + j] = (a + M - c) / 2;
                        theta[j * n + i] = (c + M - a) / 2;
                    }
                spdlog::trace("qkp_planes: iteration {}: bound {}, best {}",
                              iteration, b, best);
            }
        }

        result.bound = best;
        for(std::size_t i = 0; i < n; ++i)
            result.option_bounds[items[i]] = best_u[i];
        return result;
    }

    template <instance_c I>
    bound_report bound(const I & instance, const double budget) const {
        // the target is the criterion's: the weighted part of a linear one
        std::optional<double> weighted_target;
        if(target) {
            const auto form = linear_form(instance.criterion(),
                                          std::ranges::size(instance.cases()));
            if(form)
                weighted_target = target.value() - form->constant;
            else
                spdlog::warn(
                    "qkp_planes: --target ignored, the criterion is not "
                    "linear");
        }
        const double b = bound_criterion(
            instance, "qkp_planes", [&](std::span<const double> w) {
                return weighted(instance, budget, w, weighted_target).bound;
            });
        return {b};
    }
};

}  // namespace gecot

#endif  // GECOT_BOUNDS_QKP_PLANES_HPP
