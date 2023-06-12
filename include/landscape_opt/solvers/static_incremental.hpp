#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/zip.hpp>

#include "melon/container/static_map.hpp"

#include "landscape_opt/helper.hpp"
#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/utils/chronometer.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct StaticIncremental {
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

        const double base_score = compute_base_score(instance, parallel);

        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());

        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        compute_options_cases_incr_eca(
            instance, options,
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].vertex_quality_map();
            }),
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].arc_probability_map();
            }),
            cases_vertex_options, cases_arc_options, options_cases_eca,
            parallel);

        auto options_ratios = instance.create_option_map(0.0);
        for(const option_t & option : options) {
            options_ratios[option] =
                (instance.eval_criterion(options_cases_eca[option]) - base_score) /
                instance.option_cost(option);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] > options_ratios[o2];
        });

        double purchased = 0.0;
        for(option_t option : options) {
            const double price = instance.option_cost(option);
            if(purchased + price > budget) continue;
            purchased += price;
            solution[option] = true;
            if(verbose) {
                std::cout << "add option: " << option
                          << "\n\t ratio: " << options_ratios[option]
                          << "\n\t costing: " << price
                          << "\n\t budget left: " << budget - purchased << std::endl;
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP