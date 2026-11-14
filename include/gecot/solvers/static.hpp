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
#include "gecot/utils/log_table.hpp"

namespace gecot {
namespace solvers {

struct Static {
    double feasibility_tol = 0.0;
    // every option within the budget is bought first, then the worst ones
    // are removed until the budget is met; with a single cost and no
    // conflict set
    bool decremental = false;
    bool only_dec = false;  // decremental, without the final purchases

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const budget_t & budget) const {
        if(decremental && !has_plain_budget(instance))
            throw std::invalid_argument(
                "static --decremental: conflict sets and several cost "
                "dimensions are not supported, run static without "
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

        if(decremental) {
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
                {"removed option id"},
                {"loss/cost ratio", log_format::scientific(5)},
                {"solution cost", log_format::scientific(5)});

            const double max_score =
                compute_score(instance, cases_current_sqm, cases_current_tqm,
                              cases_current_pm);
            compute_options_cases_decr_pc_num(
                instance, solution, options, cases_current_sqm,
                cases_current_tqm, cases_current_pm, cases_vertex_options,
                cases_arc_options, options_cases_pc_num);

            for(auto && option : options) {
                options_ratios[option] =
                    (max_score -
                     instance.eval_criterion(options_cases_pc_num[option])) /
                    instance.option_cost(option);
            }

            std::ranges::sort(
                options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] < options_ratios[o2];
                });

            for(auto && option : options) {
                if(purchased <= budget.front() + feasibility_tol) break;
                const double price = instance.option_cost(option);
                purchased -= price;
                solution[option] = false;
                free_options.emplace_back(option);
                removed.row(instance.option_name(option),
                            options_ratios[option], purchased);
            }
            if(only_dec) return solution;
        }

        // the cost spent in each dimension
        std::vector<double> spent;
        for(std::size_t r = 0; r < budget.size(); ++r)
            spent.push_back(r == 0 ? purchased : 0.0);
        const auto fits = [&](const option_t & o) {
            for(std::size_t r = 0; r < budget.size(); ++r)
                if(!(instance.option_cost(o, r) <=
                     budget[r] - spent[r] + feasibility_tol))
                    return false;
            return true;
        };
        if(std::ranges::any_of(free_options, fits)) {
            log_table<spdlog::level::trace, 50, 40, 12, 11> added(
                {"added option id"},
                {"gain/cost ratio", log_format::scientific(5)},
                {"budget left", log_format::scientific(5)});

            for(const auto & instance_case : cases) {
                auto & current_sqm = (cases_current_sqm[instance_case.id()] =
                                          instance_case.source_quality_map());
                auto & current_tqm = (cases_current_tqm[instance_case.id()] =
                                          instance_case.target_quality_map());
                auto & current_pm = (cases_current_pm[instance_case.id()] =
                                         instance_case.arc_probability_map());
                auto && vertex_options =
                    cases_vertex_options[instance_case.id()];
                auto && arc_options = cases_arc_options[instance_case.id()];
                for(auto && option : options) {
                    if(!solution[option]) continue;
                    for(auto && [u, source_quality_gain, target_quality_gain] :
                        vertex_options[option]) {
                        current_sqm[u] += source_quality_gain;
                        current_tqm[u] += target_quality_gain;
                    }
                    for(auto && [a, enhanced_prob] : arc_options[option])
                        current_pm[a] = std::max(current_pm[a], enhanced_prob);
                }
            }
            const double decremental_score =
                compute_score(instance, cases_current_sqm, cases_current_tqm,
                              cases_current_pm);
            compute_options_cases_incr_pc_num(
                instance, free_options, cases_current_sqm, cases_current_tqm,
                cases_current_pm, cases_vertex_options, cases_arc_options,
                options_cases_pc_num);

            // divides the gains: the cost with a single dimension
            const auto costs = compute_heuristic_costs(instance, budget);
            const std::size_t r0 = reference_dimension(budget);
            for(const option_t & option : free_options) {
                const double gain =
                    instance.eval_criterion(options_cases_pc_num[option]) -
                    decremental_score;
                options_ratios[option] =
                    costs[option] == 0 ? gain : gain / costs[option];
            }
            // a free option, whose ratio would be infinite or undefined,
            // comes first if it gains, last if it loses, and otherwise
            // counts as a gain of 0
            const auto tier = [&](const option_t & o) {
                if(costs[o] != 0) return 0;
                const double eps = 1e-12 * std::abs(decremental_score);
                return (options_ratios[o] > eps) - (options_ratios[o] < -eps);
            };
            const auto key = [&](const option_t & o) {
                return costs[o] == 0 && tier(o) == 0 ? 0.0 : options_ratios[o];
            };
            std::ranges::sort(free_options, [&](auto && o1, auto && o2) {
                const int t1 = tier(o1), t2 = tier(o2);
                return t1 != t2 ? t1 > t2 : key(o1) > key(o2);
            });

            const auto options_sets = compute_options_conflict_sets(instance);
            // a conflict set of the option holds a bought option
            const auto blocked = [&](const option_t & option) {
                return std::ranges::any_of(
                    options_sets[option], [&](auto && l) {
                        return std::ranges::any_of(
                            instance.conflict_sets()[l], [&](auto && other) {
                                return other != option && solution[other];
                            });
                    });
            };
            for(const option_t & option : free_options) {
                bool over = false;
                for(std::size_t r = 0; r < budget.size(); ++r)
                    over = over || spent[r] + instance.option_cost(option, r) >
                                       budget[r] + feasibility_tol;
                if(over || blocked(option)) continue;
                for(std::size_t r = 0; r < budget.size(); ++r)
                    spent[r] += instance.option_cost(option, r);
                solution[option] = true;
                added.row(instance.option_name(option), options_ratios[option],
                          budget[r0] - spent[r0]);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_STATIC_HPP