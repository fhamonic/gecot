#ifndef GECOT_SOLVERS_GREEDY_HPP
#define GECOT_SOLVERS_GREEDY_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/utils/log_table.hpp"

namespace gecot {
namespace solvers {

struct Greedy {
    double feasibility_tol = 0.0;
    // every option within the budget is bought first, then removed one by
    // one until the budget is met; with a single cost and no conflict set
    bool decremental = false;
    bool only_dec = false;  // decremental, without the final purchases
    bool worst = false;     // worst choice in both phases

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const budget_t & budget) const {
        if(decremental && !has_plain_budget(instance))
            throw std::invalid_argument(
                "greedy --decremental: conflict sets and several cost "
                "dimensions are not supported, run greedy without "
                "--decremental");
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options, free_options;
        double purchased = 0.0;
        const auto fits_alone = [&](const option_t & option) {
            for(std::size_t r = 0; r < budget.size(); ++r)
                if(instance.option_cost(option, r) >
                   budget[r] + feasibility_tol)
                    return false;
            return true;
        };
        for(const option_t & option : instance.options()) {
            if(!fits_alone(option)) continue;
            if(!decremental) {
                free_options.emplace_back(option);
                continue;
            }
            options.emplace_back(option);
            solution[option] = true;
            purchased += instance.option_cost(option);
        }

        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        auto cases_current_sqm =
            instance
                .template create_case_map<instance_source_quality_map_t<I>>();
        auto cases_current_tqm =
            instance
                .template create_case_map<instance_target_quality_map_t<I>>();
        auto cases_current_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);
        auto options_ratios = instance.create_option_map(0.0);
        // divides the gains: the cost with a single dimension
        const auto costs = compute_heuristic_costs(instance, budget);
        const std::size_t r0 = reference_dimension(budget);
        const auto options_sets = compute_options_conflict_sets(instance);
        // a conflict set of the option holds a bought option
        const auto blocked = [&](const option_t & option) {
            return std::ranges::any_of(options_sets[option], [&](auto && l) {
                return std::ranges::any_of(
                    instance.conflict_sets()[l], [&](auto && other) {
                        return other != option && solution[other];
                    });
            });
        };

        // the landscape with the given options bought
        const auto set_current_maps = [&](auto && bought_options) {
            for(const auto & instance_case : cases) {
                auto & current_sqm = (cases_current_sqm[instance_case.id()] =
                                          instance_case.source_quality_map());
                auto & current_tqm = (cases_current_tqm[instance_case.id()] =
                                          instance_case.target_quality_map());
                auto & current_pm = (cases_current_pm[instance_case.id()] =
                                         instance_case.arc_probability_map());
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];
                for(auto && option : bought_options) {
                    for(auto && [u, source_quality_gain, target_quality_gain] :
                        vertex_options[option]) {
                        current_sqm[u] += source_quality_gain;
                        current_tqm[u] += target_quality_gain;
                    }
                    for(auto && [a, enhanced_prob] : arc_options[option])
                        current_pm[a] = std::max(current_pm[a], enhanced_prob);
                }
            }
        };
        set_current_maps(options);

        log_table<spdlog::level::trace, 50, 40, 12, 11> removed(
            {"removed option id"}, {"score loss", log_format::scientific(4)},
            {"solution cost", log_format::scientific(5)});
        double previous_score = compute_score(
            instance, cases_current_sqm, cases_current_tqm, cases_current_pm);
        // The scores are sums whose order changes from run to run, and
        // their last digits with it: among options of equal ratios, these
        // digits would choose. The option of lowest id is chosen among those
        // whose ratio is within 1e-12 of the score, per unit of cost, of the
        // best one. Over the incremental steps of the benchmark, the best
        // ratio and the next were within 3e-15, tied, or 6e-9 apart at
        // least; with --worst, where the ratios near zero compete, real
        // differences fall inside the window and are ties too.
        //
        // A free option, whose ratio would be infinite or undefined, comes
        // first if it gains, last if it loses, and otherwise counts as a gain
        // of 0.
        const auto tier = [&](const option_t & o) {
            if(costs[o] != 0) return 0;
            const double eps = 1e-12 * std::abs(previous_score);
            return (options_ratios[o] > eps) - (options_ratios[o] < -eps);
        };
        const auto key = [&](const option_t & o) {
            return costs[o] == 0 && tier(o) == 0 ? 0.0 : options_ratios[o];
        };
        const auto by_ratio = [&](auto && o1, auto && o2) {
            const int t1 = tier(o1), t2 = tier(o2);
            return t1 != t2 ? t1 < t2 : key(o1) < key(o2);
        };
        const auto weight = [&](const option_t & o) {
            return costs[o] == 0 ? 1.0 : 1 / costs[o];
        };
        const auto first_tied = [&](std::vector<option_t> & candidates,
                                    const auto best_it) {
            auto first_it = best_it;
            for(auto it = candidates.begin(); it != candidates.end(); ++it)
                if(*it < *first_it && tier(*it) == tier(*best_it) &&
                   (costs[*it] == 0) == (costs[*best_it] == 0) &&
                   std::abs(key(*it) - key(*best_it)) <=
                       1e-12 * std::abs(previous_score) *
                           (weight(*it) + weight(*best_it)))
                    first_it = it;
            return first_it;
        };
        while(!options.empty() &&
              purchased > budget.front() + feasibility_tol) {
            compute_options_cases_decr_pc_num(
                instance, solution, options, cases_current_sqm,
                cases_current_tqm, cases_current_pm, cases_vertex_options,
                cases_arc_options, options_cases_pc_num);
            for(const option_t & option : options) {
                options_ratios[option] =
                    (previous_score -
                     instance.eval_criterion(options_cases_pc_num[option])) /
                    instance.option_cost(option);
            }
            auto worst_option_it = first_tied(
                options, worst ? std::ranges::max_element(options, by_ratio)
                               : std::ranges::min_element(options, by_ratio));
            option_t worst_option = *worst_option_it;
            const double worst_option_price =
                instance.option_cost(worst_option);
            previous_score -= options_ratios[worst_option] * worst_option_price;
            purchased -= worst_option_price;
            solution[worst_option] = false;
            for(const auto & instance_case : cases) {
                auto & current_sqm = cases_current_sqm[instance_case.id()];
                auto & current_tqm = cases_current_tqm[instance_case.id()];
                auto & current_pm = cases_current_pm[instance_case.id()];
                const auto & original_pm = instance_case.arc_probability_map();
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];
                for(auto && [u, source_quality_gain, target_quality_gain] :
                    vertex_options[worst_option]) {
                    current_sqm[u] -= source_quality_gain;
                    current_tqm[u] -= target_quality_gain;
                }
                for(auto && [a, _] : arc_options[worst_option]) {
                    current_pm[a] = original_pm[a];
                    for(auto && [current_prob, i] :
                        instance_case.arc_options_map()[a]) {
                        if(!solution[i] || worst_option == i) continue;
                        current_pm[a] = std::max(current_pm[a], current_prob);
                    }
                }
            }
            options.erase(worst_option_it);
            free_options.emplace_back(worst_option);
            removed.row(instance.option_name(worst_option),
                        options_ratios[worst_option] * worst_option_price,
                        purchased);
        }
        removed.end();

        if(!decremental || !only_dec) {
            // the removals left their rounding errors in previous_score
            if(decremental)
                previous_score =
                    compute_score(instance, cases_current_sqm,
                                  cases_current_tqm, cases_current_pm);
            std::vector<double> budget_left;
            for(std::size_t r = 0; r < budget.size(); ++r)
                budget_left.push_back(r == 0 ? budget[r] - purchased
                                             : budget[r]);
            // over the budget left in a dimension, or excluded by a bought
            // option
            const auto unavailable = [&](const option_t & o) {
                for(std::size_t r = 0; r < budget.size(); ++r)
                    if(instance.option_cost(o, r) >
                       budget_left[r] + feasibility_tol)
                        return true;
                return blocked(o);
            };
            {
                const auto [first, last] =
                    std::ranges::remove_if(free_options, unavailable);
                free_options.erase(first, last);
            }
            log_table<spdlog::level::trace, 50, 40, 12, 11> added(
                {"added option id"}, {"score gain", log_format::scientific(4)},
                {"budget left", log_format::scientific(5)});
            // When nothing more fits, a bought option is swapped for another
            // of its conflict sets that fits and scores more, while such a
            // swap exists: buying a free option first must not lock out a
            // better one. The purchases then go on.
            const bool upgrades =
                !worst && !std::ranges::empty(instance.conflict_sets());
            const std::vector<option_t> candidates = free_options;
            const auto upgrade = [&] {
                for(;;) {
                    double best_score =
                        previous_score + 1e-12 * std::abs(previous_score);
                    std::optional<std::pair<option_t, option_t>> best_swap;
                    for(auto && set : instance.conflict_sets()) {
                        const auto bought = std::ranges::find_if(
                            set, [&](auto && o) { return solution[o]; });
                        if(bought == std::ranges::end(set)) continue;
                        const option_t o = *bought;
                        for(auto && m : set) {
                            if(solution[m]) continue;
                            bool fits = true;
                            for(std::size_t r = 0; r < budget.size(); ++r)
                                fits =
                                    fits && instance.option_cost(m, r) <=
                                                budget_left[r] +
                                                    instance.option_cost(o, r) +
                                                    feasibility_tol;
                            if(!fits) continue;
                            solution[o] = false;
                            solution[m] = true;
                            if(!blocked(m)) {
                                const double score = instance.eval_criterion(
                                    compute_solution_cases_pc_num(
                                        instance, solution,
                                        cases_vertex_options,
                                        cases_arc_options));
                                if(score > best_score) {
                                    best_score = score;
                                    best_swap.emplace(o, m);
                                }
                            }
                            solution[o] = true;
                            solution[m] = false;
                        }
                    }
                    if(!best_swap) return false;
                    const auto [o, m] = best_swap.value();
                    solution[o] = false;
                    solution[m] = true;
                    for(std::size_t r = 0; r < budget.size(); ++r)
                        budget_left[r] += instance.option_cost(o, r) -
                                          instance.option_cost(m, r);
                    set_current_maps(
                        std::views::filter(instance.options(), [&](auto && i) {
                            return static_cast<bool>(solution[i]);
                        }));
                    const double score =
                        compute_score(instance, cases_current_sqm,
                                      cases_current_tqm, cases_current_pm);
                    added.row(instance.option_name(o) + " -> " +
                                  instance.option_name(m),
                              score - previous_score, budget_left[r0]);
                    previous_score = score;
                    free_options.clear();
                    for(auto && c : candidates)
                        if(!solution[c] && !unavailable(c))
                            free_options.push_back(c);
                    if(!free_options.empty()) return true;
                }
            };
            while(!free_options.empty() || (upgrades && upgrade())) {
                compute_options_cases_incr_pc_num(
                    instance, free_options, cases_current_sqm,
                    cases_current_tqm, cases_current_pm, cases_vertex_options,
                    cases_arc_options, options_cases_pc_num);
                for(const option_t & option : free_options) {
                    const double gain =
                        instance.eval_criterion(options_cases_pc_num[option]) -
                        previous_score;
                    options_ratios[option] =
                        costs[option] == 0 ? gain : gain / costs[option];
                }
                auto best_option_it = first_tied(
                    free_options,
                    worst ? std::ranges::min_element(free_options, by_ratio)
                          : std::ranges::max_element(free_options, by_ratio));
                option_t best_option = *best_option_it;
                // as the ratio times the cost, as before the cost dimensions
                if(costs[best_option] == 0)
                    previous_score += options_ratios[best_option];
                else
                    previous_score +=
                        options_ratios[best_option] * costs[best_option];
                for(std::size_t r = 0; r < budget.size(); ++r)
                    budget_left[r] -= instance.option_cost(best_option, r);
                solution[best_option] = true;
                for(const auto & instance_case : cases) {
                    auto & current_sqm = cases_current_sqm[instance_case.id()];
                    auto & current_tqm = cases_current_tqm[instance_case.id()];
                    auto & current_pm = cases_current_pm[instance_case.id()];
                    for(auto && [u, source_quality_gain, target_quality_gain] :
                        cases_vertex_options[instance_case.id()][best_option]) {
                        current_sqm[u] += source_quality_gain;
                        current_tqm[u] += target_quality_gain;
                    }
                    for(auto && [a, enhanced_prob] :
                        cases_arc_options[instance_case.id()][best_option])
                        current_pm[a] = std::max(current_pm[a], enhanced_prob);
                }
                added.row(
                    instance.option_name(best_option),
                    costs[best_option] == 0
                        ? options_ratios[best_option]
                        : options_ratios[best_option] * costs[best_option],
                    budget_left[r0]);

                free_options.erase(best_option_it);
                const auto [first, last] =
                    std::ranges::remove_if(free_options, unavailable);
                free_options.erase(first, last);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_GREEDY_HPP