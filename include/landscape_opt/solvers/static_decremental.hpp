#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "landscape_opt/concepts/instance.hpp"
#include "landscape_opt/helper.hpp"
#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/utils/chronometer.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct StaticDecremental {
    bool verbose = false;
    bool parallel = false;
    bool only_dec = false;

    template <instance_c I>
    instance_solution_t<I> solve(const I & instance,
                                 const double budget) const {
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        double purchased = 0.0;
        for(const option_t & option : instance.options()) {
            if(instance.option_cost(option) > budget) continue;
            options.emplace_back(option);
            solution[option] = true;
            purchased += instance.option_cost(option);
        }

        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());
        auto cases_current_qm =
            instance.template create_case_map<instance_quality_map_t<I>>();
        auto cases_current_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        for(const auto & instance_case : cases) {
            auto & current_qm = (cases_current_qm[instance_case.id()] =
                                     instance_case.vertex_quality_map());
            auto & current_pm = (cases_current_pm[instance_case.id()] =
                                     instance_case.arc_probability_map());
            const auto & vertex_options =
                cases_vertex_options[instance_case.id()];
            const auto & arc_options = cases_arc_options[instance_case.id()];
            for(auto && option : options) {
                for(auto && [u, quality_gain] : vertex_options[option])
                    current_qm[u] += quality_gain;
                for(auto && [a, enhanced_prob] : arc_options[option])
                    current_pm[a] = std::max(current_pm[a], enhanced_prob);
            }
        }

        const double max_score =
            compute_solution_score(instance, solution, cases_vertex_options,
                                   cases_arc_options, parallel);
        compute_options_cases_decr_eca(instance, solution, options,
                                       cases_current_qm, cases_current_pm,
                                       cases_vertex_options, cases_arc_options,
                                       options_cases_eca, parallel);

        auto options_ratios = instance.create_option_map(0.0);
        for(auto && option : options) {
            options_ratios[option] =
                (max_score -
                 instance.eval_criterion(options_cases_eca[option])) /
                instance.option_cost(option);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] < options_ratios[o2];
        });

        for(auto && option : options) {
            const double price = instance.option_cost(option);
            purchased -= price;
            solution[option] = false;
            if(verbose) {
                std::cout << "removed option: " << option
                          << "\n\t ratio: " << options_ratios[option]
                          << "\n\t costing: " << price
                          << "\n\t purchased: " << purchased << std::endl;
            }
            if(purchased <= budget) break;
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP