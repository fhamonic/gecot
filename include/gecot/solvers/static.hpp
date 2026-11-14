#ifndef GECOT_SOLVERS_STATIC_HPP
#define GECOT_SOLVERS_STATIC_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <vector>

#include <spdlog/spdlog.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/solvers/heuristics_common.hpp"
#include "gecot/utils/log_table.hpp"

namespace gecot {
namespace solvers {

struct Static {
    double feasibility_tol = default_feasibility_tol;
    // every option within the budget is bought first, then the worst ones
    // are removed until the budget is met; with a single cost and no
    // conflict set
    bool decremental = false;
    bool only_dec = false;  // decremental, without the final purchases

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const budget_t & budget) const {
        if(decremental) check_decremental(instance, "static");
        auto solution = instance.create_option_map(false);

        // the options that fit alone: bought with --decremental, to buy
        // otherwise
        std::vector<option_t> options, free_options;
        for(const option_t & i : instance.options())
            if(fits_alone(instance, i, budget, feasibility_tol))
                (decremental ? options : free_options).emplace_back(i);
        double purchased = 0.0;
        for(const option_t & i : options) {
            solution[i] = true;
            purchased += instance.option_cost(i);
        }

        heuristic_landscape landscape(instance);
        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        auto options_ratios = instance.create_option_map(0.0);

        // --decremental: the options by increasing loss per unit of cost in
        // the scenario that buys them all, removed until the cost fits the
        // budget
        if(decremental) {
            landscape.set(options);
            log_table<spdlog::level::trace, 50, 40, 12, 11> removed(
                {"removed option id"},
                {"loss/cost ratio", log_format::scientific(5)},
                {"solution cost", log_format::scientific(5)});
            const double max_score = landscape.score();
            landscape.scores_without(solution, options, options_cases_pc_num);
            for(const option_t & i : options) {
                options_ratios[i] = (max_score - instance.eval_criterion(
                                                     options_cases_pc_num[i])) /
                                    instance.option_cost(i);
            }
            std::ranges::sort(
                options, [&options_ratios](auto && i1, auto && i2) {
                    return options_ratios[i1] < options_ratios[i2];
                });

            for(const option_t & i : options) {
                if(purchased <= budget.front() + feasibility_tol) break;
                purchased -= instance.option_cost(i);
                solution[i] = false;
                free_options.emplace_back(i);
                removed.row(instance.option_name(i), options_ratios[i],
                            purchased);
            }
            if(only_dec) return solution;
        }

        // The cost spent in each dimension. The removals leave it in the
        // first one alone, which is the only one: --decremental refuses
        // several.
        std::vector<double> spent;
        for(std::size_t r = 0; r < budget.size(); ++r)
            spent.push_back(r == 0 ? purchased : 0.0);
        const auto fits = [&](const option_t & i) {
            for(std::size_t r = 0; r < budget.size(); ++r)
                if(!(instance.option_cost(i, r) <=
                     budget[r] - spent[r] + feasibility_tol))
                    return false;
            return true;
        };
        // the options by decreasing gain per unit of cost in the scenario
        // reached, each bought if it fits in what is left of the budget
        if(std::ranges::any_of(free_options, fits)) {
            log_table<spdlog::level::trace, 50, 40, 12, 11> added(
                {"added option id"},
                {"gain/cost ratio", log_format::scientific(5)},
                {"budget left", log_format::scientific(5)});

            landscape.set(std::views::filter(options, [&](auto && i) {
                return static_cast<bool>(solution[i]);
            }));
            const double decremental_score = landscape.score();
            landscape.scores_with(free_options, options_cases_pc_num);

            // divides the gains: the cost with a single dimension
            const auto costs = compute_heuristic_costs(instance, budget);
            const std::size_t r0 = reference_dimension(budget);
            for(const option_t & i : free_options) {
                const double gain =
                    instance.eval_criterion(options_cases_pc_num[i]) -
                    decremental_score;
                options_ratios[i] = costs[i] == 0 ? gain : gain / costs[i];
            }
            const auto tier = [&](const option_t & i) {
                return free_option_tier(costs[i], options_ratios[i],
                                        decremental_score);
            };
            const auto key = [&](const option_t & i) {
                return ratio_key(costs[i], options_ratios[i],
                                 decremental_score);
            };
            std::ranges::sort(free_options, [&](auto && i1, auto && i2) {
                const int t1 = tier(i1), t2 = tier(i2);
                return t1 != t2 ? t1 > t2 : key(i1) > key(i2);
            });

            const auto options_sets = compute_options_conflict_sets(instance);
            for(const option_t & i : free_options) {
                bool over = false;
                for(std::size_t r = 0; r < budget.size(); ++r)
                    over = over || spent[r] + instance.option_cost(i, r) >
                                       budget[r] + feasibility_tol;
                if(over || is_blocked(instance, options_sets, solution, i))
                    continue;
                for(std::size_t r = 0; r < budget.size(); ++r)
                    spent[r] += instance.option_cost(i, r);
                solution[i] = true;
                added.row(instance.option_name(i), options_ratios[i],
                          budget[r0] - spent[r0]);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_STATIC_HPP