#ifndef GECOT_SOLVERS_STATIC_DECREMENTAL_HPP
#define GECOT_SOLVERS_STATIC_DECREMENTAL_HPP

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/eca.hpp"
#include "gecot/utils/chronometer.hpp"

namespace fhamonic {
namespace gecot {
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
            compute_score(instance, cases_current_qm, cases_current_pm);
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

        std::vector<option_t> free_options;
        for(auto && option : options) {
            const double price = instance.option_cost(option);
            purchased -= price;
            solution[option] = false;
            free_options.emplace_back(option);
            if(verbose) {
                std::cout << "removed option: " << option
                          << "\n\t ratio: " << options_ratios[option]
                          << "\n\t costing: " << price
                          << "\n\t purchased: " << purchased << std::endl;
            }
            if(purchased <= budget) break;
        }

        if(!only_dec) {
            for(auto instance_case : cases) {
                auto & current_qm = (cases_current_qm[instance_case.id()] =
                                         instance_case.vertex_quality_map());
                auto & current_pm = (cases_current_pm[instance_case.id()] =
                                         instance_case.arc_probability_map());
                auto && vertex_options =
                    cases_vertex_options[instance_case.id()];
                auto && arc_options = cases_arc_options[instance_case.id()];
                for(auto && option : options) {
                    if(!solution[option]) continue;
                    for(auto && [u, quality_gain] : vertex_options[option])
                        current_qm[u] += quality_gain;
                    for(auto && [a, enhanced_prob] : arc_options[option])
                        current_pm[a] = std::max(current_pm[a], enhanced_prob);
                }
            }
            const double decremental_score =
                compute_score(instance, cases_current_qm, cases_current_pm);
            compute_options_cases_incr_eca(
                instance, free_options, cases_current_qm, cases_current_pm,
                cases_vertex_options, cases_arc_options, options_cases_eca,
                parallel);

            for(const option_t & option : free_options) {
                options_ratios[option] =
                    (instance.eval_criterion(options_cases_eca[option]) -
                     decremental_score) /
                    instance.option_cost(option);
            }
            std::ranges::sort(
                free_options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] > options_ratios[o2];
                });

            for(const option_t & option : free_options) {
                const double price = instance.option_cost(option);
                if(purchased + price > budget) continue;
                purchased += price;
                solution[option] = true;
                if(verbose) {
                    std::cout << "add option: " << option
                              << "\n\t ratio: " << options_ratios[option]
                              << "\n\t costing: " << price
                              << "\n\t budget left: " << budget - purchased
                              << std::endl;
                }
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_STATIC_DECREMENTAL_HPP