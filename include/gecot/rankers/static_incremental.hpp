#ifndef GECOT_SOLVERS_STATIC_INCREMENTAL_HPP
#define GECOT_SOLVERS_STATIC_INCREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/utils/chronometer.hpp"

namespace fhamonic {
namespace gecot {
namespace rankers {

struct StaticIncremental {
    bool parallel = false;

    template <instance_c I>
    instance_options_rank_t<I> rank_options(const I & instance) const {
        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & option : instance.options()) {
            options.emplace_back(option);
        }

        const double base_score = compute_base_score(instance, parallel);
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
            cases_vertex_options, cases_arc_options, options_cases_pc_num,
            parallel);

        auto options_ratios = instance.create_option_map(0.0);
        for(const option_t & option : options) {
            options_ratios[option] =
                (instance.eval_criterion(options_cases_pc_num[option]) -
                 base_score) /
                instance.option_cost(option);
        }

        return options_ratios;
    }
};

}  // namespace rankers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_STATIC_INCREMENTAL_HPP