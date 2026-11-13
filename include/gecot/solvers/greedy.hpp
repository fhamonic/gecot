#ifndef GECOT_SOLVERS_GREEDY_HPP
#define GECOT_SOLVERS_GREEDY_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
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
    // one until the budget is met
    bool decremental = false;
    bool only_dec = false;  // decremental, without the final purchases
    bool worst = false;     // worst choice in both phases

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        check_plain_budget(instance, "greedy");
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options, free_options;
        double purchased = 0.0;
        for(const option_t & option : instance.options()) {
            if(instance.option_cost(option) > budget + feasibility_tol)
                continue;
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

        for(const auto & instance_case : cases) {
            auto & current_sqm = (cases_current_sqm[instance_case.id()] =
                                      instance_case.source_quality_map());
            auto & current_tqm = (cases_current_tqm[instance_case.id()] =
                                      instance_case.target_quality_map());
            auto & current_pm = (cases_current_pm[instance_case.id()] =
                                     instance_case.arc_probability_map());
            const auto & vertex_options =
                cases_vertex_options[instance_case.id()];
            const auto & arc_options = cases_arc_options[instance_case.id()];
            for(auto && option : options) {
                for(auto && [u, source_quality_gain, target_quality_gain] :
                    vertex_options[option]) {
                    current_sqm[u] += source_quality_gain;
                    current_tqm[u] += target_quality_gain;
                }
                for(auto && [a, enhanced_prob] : arc_options[option])
                    current_pm[a] = std::max(current_pm[a], enhanced_prob);
            }
        }

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
        const auto first_tied = [&](std::vector<option_t> & candidates,
                                    const auto best_it) {
            auto first_it = best_it;
            for(auto it = candidates.begin(); it != candidates.end(); ++it)
                if(*it < *first_it &&
                   std::abs(options_ratios[*it] - options_ratios[*best_it]) <=
                       1e-12 * std::abs(previous_score) *
                           (1 / instance.option_cost(*it) +
                            1 / instance.option_cost(*best_it)))
                    first_it = it;
            return first_it;
        };
        while(!options.empty() && purchased > budget + feasibility_tol) {
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
            const auto by_ratio = [&options_ratios](auto && o1, auto && o2) {
                return options_ratios[o1] < options_ratios[o2];
            };
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
            double budget_left = budget - purchased;
            {
                const auto [first, last] = std::ranges::remove_if(
                    free_options,
                    [&instance,
                     b = budget_left + feasibility_tol](const option_t & o) {
                        return instance.option_cost(o) > b;
                    });
                free_options.erase(first, last);
            }
            log_table<spdlog::level::trace, 50, 40, 12, 11> added(
                {"added option id"}, {"score gain", log_format::scientific(4)},
                {"budget left", log_format::scientific(5)});
            while(free_options.size() > 0) {
                compute_options_cases_incr_pc_num(
                    instance, free_options, cases_current_sqm,
                    cases_current_tqm, cases_current_pm, cases_vertex_options,
                    cases_arc_options, options_cases_pc_num);
                for(const option_t & option : free_options) {
                    options_ratios[option] =
                        (instance.eval_criterion(options_cases_pc_num[option]) -
                         previous_score) /
                        instance.option_cost(option);
                }
                const auto by_ratio = [&options_ratios](auto && o1,
                                                        auto && o2) {
                    return options_ratios[o1] < options_ratios[o2];
                };
                auto best_option_it = first_tied(
                    free_options,
                    worst ? std::ranges::min_element(free_options, by_ratio)
                          : std::ranges::max_element(free_options, by_ratio));
                option_t best_option = *best_option_it;
                const double best_option_price =
                    instance.option_cost(best_option);
                previous_score +=
                    options_ratios[best_option] * best_option_price;
                budget_left -= best_option_price;
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
                added.row(instance.option_name(best_option),
                          options_ratios[best_option] * best_option_price,
                          budget_left);

                free_options.erase(best_option_it);
                const auto [first, last] = std::ranges::remove_if(
                    free_options,
                    [&instance,
                     b = budget_left + feasibility_tol](const option_t & o) {
                        return instance.option_cost(o) > b;
                    });
                free_options.erase(first, last);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_GREEDY_HPP