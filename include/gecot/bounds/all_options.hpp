#ifndef GECOT_BOUNDS_ALL_OPTIONS_HPP
#define GECOT_BOUNDS_ALL_OPTIONS_HPP

#include <stdexcept>

#include "gecot/bounds/bound_report.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"

namespace gecot {

// the score with every option bought, whatever its cost: case scores only
// grow with the options bought, so does a non-decreasing criterion
template <instance_c I>
bound_report all_options_bound(const I & instance) {
    if(!is_non_decreasing(instance.criterion()))
        throw std::invalid_argument(
            "all_options: the criterion is not non-decreasing in the case "
            "scores");
    auto all_options = instance.template create_option_map<bool>();
    for(auto && option : instance.options()) all_options[option] = true;
    return {instance.eval_criterion(
        compute_solution_cases_pc_num(instance, all_options))};
}

}  // namespace gecot

#endif  // GECOT_BOUNDS_ALL_OPTIONS_HPP
