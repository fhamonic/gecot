#ifndef GECOT_SOLVERS_STATIC_INCREMENTAL_HPP
#define GECOT_SOLVERS_STATIC_INCREMENTAL_HPP

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

struct StaticIncremental {
    double feasability_tol = 0.0;

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & option : instance.options()) {
            if(instance.option_cost(option) > budget + feasability_tol) continue;
            options.emplace_back(option);
        }

        spdlog::trace(
            "--------------------------------------------------------");
        spdlog::trace("   added option id   | gain/cost ratio | solution cost");
        spdlog::trace(
            "--------------------------------------------------------");

        const double base_score = compute_base_score(instance);
        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        compute_options_cases_incr_pc_num(
            instance, options,
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].vertex_quality_map();
            }),
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].arc_probability_map();
            }),
            cases_vertex_options, cases_arc_options, options_cases_pc_num);

        auto options_ratios = instance.create_option_map(0.0);
        for(const option_t & option : options) {
            options_ratios[option] =
                (instance.eval_criterion(options_cases_pc_num[option]) -
                 base_score) /
                instance.option_cost(option);
        }
        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] > options_ratios[o2];
        });

        double purchased = 0.0;
        for(const option_t & option : options) {
            const double price = instance.option_cost(option);
            if(purchased + price > budget + feasability_tol) continue;
            purchased += price;
            solution[option] = true;
            spdlog::trace("{:>20} |  {: #.6e}  | {: #.5e}",
                          instance.option_name(option), options_ratios[option],
                          budget - purchased);
        }
        spdlog::trace(
            "--------------------------------------------------------");

        return solution;
    }
};
}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_STATIC_INCREMENTAL_HPP