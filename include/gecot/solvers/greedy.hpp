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

#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/solvers/heuristics_common.hpp"
#include "gecot/utils/log_table.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

struct Greedy {
    double feasibility_tol = default_feasibility_tol;
    // every option within the budget is bought first, then removed one by
    // one until the budget is met; with a single cost and no conflict set
    bool decremental = false;
    bool only_dec = false;  // decremental, without the final purchases
    bool worst = false;     // worst choice in both phases

    // PURCHASES: the level of the table of the purchases, lower as a MIP start
    template <spdlog::level::level_enum PURCHASES = spdlog::level::debug,
              instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const budget_t & budget) const {
        if(decremental) check_decremental(instance, "greedy");
        // Ctrl-C between two steps once within the budget: the options bought
        // so far. The report without values is what logs the line on Ctrl-C.
        const auto stopped = [] {
            if(stop_requested) return true;
            report_progress({});
            return false;
        };
        auto [solution, options, free_options, purchased] =
            heuristic_start(instance, budget, feasibility_tol, decremental);

        heuristic_landscape landscape(instance);
        landscape.set(options);
        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        auto options_ratios = instance.create_option_map(0.0);
        // divides the gains: the cost with a single dimension
        const auto costs = compute_heuristic_costs(instance, budget);
        const auto options_sets = compute_options_conflict_sets(instance);
        const auto blocked = [&](const option_t & i) {
            return is_blocked(instance, options_sets, solution, i);
        };

        log_table<spdlog::level::trace, 50, 40, 12, 11> removed(
            {"removed option id"}, {"score loss", log_format::scientific(4)},
            {"solution cost", log_format::scientific(5)});
        double previous_score = landscape.score();
        // Among the options tied for the best ratio, the one of lowest id.
        // Over the incremental steps of the benchmark, the best ratio and
        // the next were within 3e-15, tied, or 6e-9 apart at least; with
        // --worst, where the ratios near zero compete, real differences fall
        // inside the window and are ties too.
        const option_ranking by_ratio{costs, options_ratios, previous_score};
        const auto weight = [&](const option_t & i) {
            return costs[i] == 0 ? 1.0 : 1 / costs[i];
        };
        // excluded: the options that cannot be taken, even tied
        const auto first_tied = [&](std::vector<option_t> & candidates,
                                    const auto best_it, auto && excluded) {
            auto first_it = best_it;
            for(auto it = candidates.begin(); it != candidates.end(); ++it)
                if(*it < *first_it && !excluded(*it) &&
                   by_ratio.tier(*it) == by_ratio.tier(*best_it) &&
                   (costs[*it] == 0) == (costs[*best_it] == 0) &&
                   std::abs(by_ratio.key(*it) - by_ratio.key(*best_it)) <=
                       tie_window * std::abs(previous_score) *
                           (weight(*it) + weight(*best_it)))
                    first_it = it;
            return first_it;
        };
        // --decremental: the option of smallest loss per unit of cost
        // (largest with --worst) is removed, until the cost fits the budget;
        // then, without --worst, while removing one raises the score, which
        // only a criterion that can decrease allows: for the others, the
        // test would cost a last evaluation of every option
        const bool can_decrease =
            !worst && !has_non_decreasing_criterion(instance);
        while(!options.empty() &&
              (purchased > budget.front() + feasibility_tol ||
               (can_decrease && !stopped()))) {
            landscape.scores_without(solution, options, options_cases_pc_num);
            for(const option_t & i : options) {
                options_ratios[i] =
                    (previous_score -
                     instance.eval_criterion(options_cases_pc_num[i])) /
                    instance.option_cost(i);
            }
            // once the cost fits, an option whose removal does not raise the
            // score is kept: the smallest ratio can be that of such an option
            // of small cost, beside a removal that raises the score
            const bool fits = purchased <= budget.front() + feasibility_tol;
            const auto kept = [&](const option_t & i) {
                return fits && options_ratios[i] * instance.option_cost(i) >=
                                   -tie_window * std::abs(previous_score);
            };
            auto worst_option_it = options.end();
            for(auto it = options.begin(); it != options.end(); ++it)
                if(!kept(*it) && (worst_option_it == options.end() ||
                                  (worst ? by_ratio(*worst_option_it, *it)
                                         : by_ratio(*it, *worst_option_it))))
                    worst_option_it = it;
            if(worst_option_it == options.end()) break;
            worst_option_it = first_tied(options, worst_option_it, kept);
            option_t worst_option = *worst_option_it;
            const double worst_option_price =
                instance.option_cost(worst_option);
            previous_score -= options_ratios[worst_option] * worst_option_price;
            purchased -= worst_option_price;
            solution[worst_option] = false;
            landscape.remove(worst_option, solution);
            options.erase(worst_option_it);
            free_options.emplace_back(worst_option);
            removed.row(instance.option_name(worst_option),
                        options_ratios[worst_option] * worst_option_price,
                        purchased);
        }
        removed.end();

        if(!decremental || !only_dec) {
            // the removals left their rounding errors in previous_score
            if(decremental) previous_score = landscape.score();
            std::vector<double> budget_left;
            for(std::size_t r = 0; r < budget.size(); ++r)
                budget_left.push_back(r == 0 ? budget[r] - purchased
                                             : budget[r]);
            // over the budget left in a dimension, or excluded by a bought
            // option
            const auto unavailable = [&](const option_t & i) {
                for(std::size_t r = 0; r < budget.size(); ++r)
                    if(instance.option_cost(i, r) >
                       budget_left[r] + feasibility_tol)
                        return true;
                return blocked(i);
            };
            std::erase_if(free_options, unavailable);
            log_table<PURCHASES, 50, 40, 11, 12> added(
                {"option bought"}, {"gain", change_text},
                {"total gain", gain_text});
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
                        previous_score + tie_window * std::abs(previous_score);
                    std::optional<std::pair<option_t, option_t>> best_swap;
                    for(auto && set : instance.conflict_sets()) {
                        const auto bought = std::ranges::find_if(
                            set, [&](auto && i) { return solution[i]; });
                        if(bought == std::ranges::end(set)) continue;
                        const option_t i = *bought;
                        for(auto && j : set) {
                            if(solution[j]) continue;
                            bool fits = true;
                            for(std::size_t r = 0; r < budget.size(); ++r)
                                fits =
                                    fits && instance.option_cost(j, r) <=
                                                budget_left[r] +
                                                    instance.option_cost(i, r) +
                                                    feasibility_tol;
                            if(!fits) continue;
                            solution[i] = false;
                            solution[j] = true;
                            if(!blocked(j)) {
                                const double score = instance.eval_criterion(
                                    compute_solution_cases_pc_num(
                                        instance, solution,
                                        landscape.cases_vertex_options,
                                        landscape.cases_arc_options));
                                if(score > best_score) {
                                    best_score = score;
                                    best_swap.emplace(i, j);
                                }
                            }
                            solution[i] = true;
                            solution[j] = false;
                        }
                    }
                    if(!best_swap) return false;
                    const auto [i, j] = best_swap.value();
                    solution[i] = false;
                    solution[j] = true;
                    for(std::size_t r = 0; r < budget.size(); ++r)
                        budget_left[r] += instance.option_cost(i, r) -
                                          instance.option_cost(j, r);
                    landscape.set(std::views::filter(
                        instance.options(), [&](auto && bought) {
                            return static_cast<bool>(solution[bought]);
                        }));
                    const double score = landscape.score();
                    added.row(instance.option_name(i) + " -> " +
                                  instance.option_name(j),
                              score - previous_score, score);
                    previous_score = score;
                    free_options.clear();
                    for(auto && c : candidates)
                        if(!solution[c] && !unavailable(c))
                            free_options.push_back(c);
                    if(!free_options.empty()) return true;
                }
            };
            // the option of largest gain per unit of cost (smallest with
            // --worst) is bought, as long as one fits
            while(!stopped() &&
                  (!free_options.empty() || (upgrades && upgrade()))) {
                landscape.scores_with(free_options, options_cases_pc_num);
                for(const option_t & i : free_options) {
                    const double gain =
                        instance.eval_criterion(options_cases_pc_num[i]) -
                        previous_score;
                    options_ratios[i] = costs[i] == 0 ? gain : gain / costs[i];
                }
                // Without --worst, no option that lowers the score is bought,
                // which only a criterion that can decrease allows, not even
                // one tied with the best. If the best does, all the others
                // do: only the swaps are left.
                const auto lowers = [&](const option_t & i) {
                    return !worst &&
                           (costs[i] == 0 ? options_ratios[i]
                                          : options_ratios[i] * costs[i]) <
                               -tie_window * std::abs(previous_score);
                };
                const auto best_it =
                    worst ? std::ranges::min_element(free_options, by_ratio)
                          : std::ranges::max_element(free_options, by_ratio);
                if(lowers(*best_it)) {
                    free_options.clear();
                    continue;
                }
                auto best_option_it = first_tied(free_options, best_it, lowers);
                option_t best_option = *best_option_it;
                // as the ratio times the cost, as before the cost dimensions
                const double gain =
                    costs[best_option] == 0
                        ? options_ratios[best_option]
                        : options_ratios[best_option] * costs[best_option];
                previous_score += gain;
                for(std::size_t r = 0; r < budget.size(); ++r)
                    budget_left[r] -= instance.option_cost(best_option, r);
                solution[best_option] = true;
                landscape.buy(best_option);
                added.row(instance.option_name(best_option), gain,
                          previous_score);

                free_options.erase(best_option_it);
                std::erase_if(free_options, unavailable);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_GREEDY_HPP