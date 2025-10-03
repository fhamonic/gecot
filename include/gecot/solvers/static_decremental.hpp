#ifndef GECOT_SOLVERS_STATIC_DECREMENTAL_HPP
#define GECOT_SOLVERS_STATIC_DECREMENTAL_HPP

#include <algorithm>
#include <limits>
#include <ranges>
#include <vector>

#include <spdlog/spdlog.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct StaticDecremental {
    double feasibility_tol = 0.0;
    bool only_dec = false;

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        double purchased = 0.0;
        for(const option_t & option : instance.options()) {
            if(instance.option_cost(option) > budget + feasibility_tol)
                continue;
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

        spdlog::trace(
            "--------------------------------------------------------");
        spdlog::trace("  removed option id  | loss/cost ratio | solution cost");
        spdlog::trace(
            "--------------------------------------------------------");

        const double max_score = compute_score(
            instance, cases_current_sqm, cases_current_tqm, cases_current_pm);
        compute_options_cases_decr_pc_num(
            instance, solution, options, cases_current_sqm, cases_current_tqm,
            cases_current_pm, cases_vertex_options, cases_arc_options,
            options_cases_pc_num);

        auto options_ratios = instance.create_option_map(0.0);
        for(auto && option : options) {
            options_ratios[option] =
                (max_score -
                 instance.eval_criterion(options_cases_pc_num[option])) /
                instance.option_cost(option);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] < options_ratios[o2];
        });

        std::vector<option_t> free_options;
        for(auto && option : options) {
            const double price = instance.option_cost(option);
            purchased -= price;
            solution[option] = false;
            free_options.emplace_back(option);
            spdlog::trace("{:>20} |  {: #.6e}  | {: #.5e}",
                          instance.option_name(option), options_ratios[option],
                          purchased);
            if(purchased <= budget + feasibility_tol) break;
        }

        if(!only_dec &&
           std::ranges::any_of(
               free_options,
               [&instance, budget_left = budget - purchased +
                                         feasibility_tol](const option_t & o) {
                   return instance.option_cost(o) <= budget_left;
               })) {
            spdlog::trace(
                "--------------------------------------------------------");
            spdlog::trace(
                "   added option id   | gain/cost ratio | solution cost");
            spdlog::trace(
                "--------------------------------------------------------");

            for(auto instance_case : cases) {
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

            for(const option_t & option : free_options) {
                options_ratios[option] =
                    (instance.eval_criterion(options_cases_pc_num[option]) -
                     decremental_score) /
                    instance.option_cost(option);
            }
            std::ranges::sort(
                free_options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] > options_ratios[o2];
                });

            for(const option_t & option : free_options) {
                const double price = instance.option_cost(option);
                if(purchased + price > budget + feasibility_tol) continue;
                purchased += price;
                solution[option] = true;
                spdlog::trace("{:>20} |  {: #.6e}  | {: #.5e}",
                              instance.option_name(option),
                              options_ratios[option], budget - purchased);
            }
        }
        spdlog::trace(
            "--------------------------------------------------------");

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_STATIC_DECREMENTAL_HPP