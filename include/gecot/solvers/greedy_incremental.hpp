#ifndef GECOT_SOLVERS_GREEDY_INCREMENTAL_HPP
#define GECOT_SOLVERS_GREEDY_INCREMENTAL_HPP

#include <algorithm>
#include <ranges>
#include <vector>

#include <spdlog/spdlog.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct GreedyIncremental {
    bool verbose = false;
    bool parallel = false;

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & option : instance.options()) {
            if(instance.option_cost(option) > budget) continue;
            options.emplace_back(option);
        }

        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        auto cases_current_qm =
            instance.template create_case_map<instance_quality_map_t<I>>();
        auto cases_current_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        for(const auto & instance_case : cases) {
            cases_current_qm[instance_case.id()] =
                instance_case.vertex_quality_map();
            cases_current_pm[instance_case.id()] =
                instance_case.arc_probability_map();
        }
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);
        auto options_ratios = instance.create_option_map(0.0);

        spdlog::trace("---------------------------------------------------");
        spdlog::trace("   added option id   | score gain |  budget left");
        spdlog::trace("---------------------------------------------------");

        double previous_score = compute_base_score(instance, parallel);
        double budget_left = budget;
        while(options.size() > 0) {
            compute_options_cases_incr_pc_num(
                instance, options, cases_current_qm, cases_current_pm,
                cases_vertex_options, cases_arc_options, options_cases_pc_num,
                parallel);
            for(const option_t & option : options) {
                options_ratios[option] =
                    (instance.eval_criterion(options_cases_pc_num[option]) -
                     previous_score) /
                    instance.option_cost(option);
            }
            auto best_option_it = std::ranges::max_element(
                options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] < options_ratios[o2];
                });
            option_t best_option = *best_option_it;
            const double best_option_price = instance.option_cost(best_option);
            previous_score += options_ratios[best_option] * best_option_price;
            budget_left -= best_option_price;
            solution[best_option] = true;
            for(const auto & instance_case : cases) {
                auto & current_qm = cases_current_qm[instance_case.id()];
                auto & current_pm = cases_current_pm[instance_case.id()];
                for(auto && [u, quality_gain] :
                    cases_vertex_options[instance_case.id()][best_option])
                    current_qm[u] += quality_gain;
                for(auto && [a, enhanced_prob] :
                    cases_arc_options[instance_case.id()][best_option])
                    current_pm[a] = std::max(current_pm[a], enhanced_prob);
            }
            spdlog::trace("{:>20} |{: #.4e} | {: #.5e}",
                          instance.option_name(best_option),
                          options_ratios[best_option] * best_option_price,
                          budget_left);

            options.erase(best_option_it);
            const auto [first, last] = std::ranges::remove_if(
                options, [&instance, budget_left](const option_t & o) {
                    return instance.option_cost(o) > budget_left;
                });
            options.erase(first, last);
        }
        spdlog::trace("---------------------------------------------------");

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_GREEDY_INCREMENTAL_HPP