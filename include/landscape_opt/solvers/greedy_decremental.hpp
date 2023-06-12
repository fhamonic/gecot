#ifndef LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

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

struct GreedyDecremental {
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
        auto options_ratios = instance.create_option_map(0.0);

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

        std::vector<option_t> free_options;
        double previous_score = compute_score(instance, cases_current_qm,
                                              cases_current_pm, parallel);
        while(purchased > budget) {
            compute_options_cases_decr_eca(
                instance, solution, options, cases_current_qm, cases_current_pm,
                cases_vertex_options, cases_arc_options, options_cases_eca,
                parallel);
            for(const option_t & option : options) {
                options_ratios[option] =
                    (previous_score -
                     instance.eval_criterion(options_cases_eca[option])) /
                    instance.option_cost(option);
            }
            auto worst_option_it = std::ranges::min_element(
                options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] < options_ratios[o2];
                });
            option_t worst_option = *worst_option_it;
            const double worst_option_price =
                instance.option_cost(worst_option);
            previous_score -= options_ratios[worst_option] * worst_option_price;
            purchased -= worst_option_price;
            solution[worst_option] = false;
            for(const auto & instance_case : cases) {
                auto & current_qm = cases_current_qm[instance_case.id()];
                auto & current_pm = cases_current_pm[instance_case.id()];
                const auto & original_pm = instance_case.arc_probability_map();
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];
                for(auto && [u, quality_gain] : vertex_options[worst_option])
                    current_qm[u] -= quality_gain;
                for(auto && [a, _] : arc_options[worst_option]) {
                    current_pm[a] = original_pm[a];
                    for(auto && [current_prob, i] :
                        instance_case.arc_options_map()[a]) {
                        if(solution[i] == 0.0 || worst_option == i) continue;
                        current_pm[a] = std::max(current_pm[a], current_prob);
                    }
                }
            }
            options.erase(worst_option_it);
            free_options.emplace_back(worst_option);

            if(verbose) {
                std::cout << "removed option: " << worst_option
                          << "\n\t ratio: " << options_ratios[worst_option]
                          << "\n\t costing: " << worst_option_price
                          << "\n\t purchased: " << purchased << std::endl;
            }
        }

        if(!only_dec) {
            previous_score = compute_score(instance, cases_current_qm,
                                           cases_current_pm, parallel);
            double budget_left = budget - purchased;
            {
                const auto [first, last] = std::ranges::remove_if(
                    free_options, [&instance, budget_left](const option_t & o) {
                        return instance.option_cost(o) > budget_left;
                    });
                free_options.erase(first, last);
            }
            while(free_options.size() > 0) {
                compute_options_cases_incr_eca(
                    instance, free_options, cases_current_qm, cases_current_pm,
                    cases_vertex_options, cases_arc_options, options_cases_eca,
                    parallel);
                for(const option_t & option : free_options) {
                    options_ratios[option] =
                        (instance.eval_criterion(options_cases_eca[option]) -
                         previous_score) /
                        instance.option_cost(option);
                }
                auto best_option_it = std::ranges::max_element(
                    free_options, [&options_ratios](auto && o1, auto && o2) {
                        return options_ratios[o1] < options_ratios[o2];
                    });
                option_t best_option = *best_option_it;
                const double best_option_price =
                    instance.option_cost(best_option);
                previous_score +=
                    options_ratios[best_option] * best_option_price;
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
                if(verbose) {
                    std::cout << "add option: " << best_option
                              << "\n\t ratio: " << options_ratios[best_option]
                              << "\n\t costing: " << best_option_price
                              << "\n\t budget left: " << budget_left
                              << std::endl;
                }

                free_options.erase(best_option_it);
                const auto [first, last] = std::ranges::remove_if(
                    free_options, [&instance, budget_left](const option_t & o) {
                        return instance.option_cost(o) > budget_left;
                    });
                free_options.erase(first, last);
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP