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
        chronometer chrono;
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        double purchased = 0.0;
        for(const option_t & o : instance.options()) {
            if(instance.option_cost(o) > budget) continue;
            options.emplace_back(o);
            solution[o] = true;
            purchased += instance.option_cost(o);
        }

        const double max_score =
            compute_solution_score(instance, solution, parallel);

        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());

        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        auto cases_max_qm =
            instance.template create_case_map<instance_quality_map_t<I>>();
        auto cases_max_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        for(auto instance_case : cases) {
            auto & max_qm = (cases_max_qm[instance_case.id()] =
                                 instance_case.vertex_quality_map());
            auto & max_pm = (cases_max_pm[instance_case.id()] =
                                 instance_case.arc_probability_map());
            const auto & vertex_options =
                cases_vertex_options[instance_case.id()];
            const auto & arc_options = cases_arc_options[instance_case.id()];
            for(auto && option : options) {
                for(auto && [u, quality_gain] : vertex_options[option])
                    max_qm[u] += quality_gain;
                for(auto && [a, enhanced_prob] : arc_options[option])
                    max_pm[a] = std::max(max_pm[a], enhanced_prob);
            }
        }

        auto compute_options_enhanced_eca =
            [&solution, &options_cases_eca, &cases_max_qm, &cases_max_pm,
             &cases_vertex_options, &cases_arc_options](
                const tbb::blocked_range2d<decltype(cases.begin()),
                                           decltype(options.begin())> &
                    cases_options_block) {
                for(auto instance_case : cases_options_block.rows()) {
                    auto && options_block = cases_options_block.cols();
                    if(options_block.begin() == options_block.end()) continue;
                    const auto & original_pm =
                        instance_case.arc_probability_map();
                    const auto & max_qm = cases_max_qm[instance_case.id()];
                    const auto & max_pm = cases_max_pm[instance_case.id()];
                    auto ablated_qm = max_qm;
                    auto ablated_pm = max_pm;
                    const auto & vertex_options =
                        cases_vertex_options[instance_case.id()];
                    const auto & arc_options =
                        cases_arc_options[instance_case.id()];

                    for(auto it = options_block.begin();;) {
                        option_t option = *it;
                        for(auto && [u, quality_gain] : vertex_options[option])
                            ablated_qm[u] -= quality_gain;
                        for(auto && [a, _] : arc_options[option]) {
                            ablated_pm[a] = original_pm[a];
                            for(auto && [current_prob, i] :
                                instance_case.arc_options_map()[a]) {
                                if(!solution[i] || option == i) continue;
                                ablated_pm[a] =
                                    std::max(ablated_pm[a], current_prob);
                            }
                        }

                        options_cases_eca[option][instance_case.id()] =
                            eca(instance_case.graph(), ablated_qm, ablated_pm);

                        if(++it == options_block.end()) break;

                        for(auto && [u, _] : vertex_options[option])
                            ablated_qm[u] = max_qm[u];
                        for(auto && [a, _] : arc_options[option])
                            ablated_pm[a] = max_pm[a];
                    }
                }
            };

        if(parallel) {
            tbb::parallel_for(
                tbb::blocked_range2d(cases.begin(), cases.end(),
                                     options.begin(), options.end()),
                compute_options_enhanced_eca);
        } else {
            compute_options_enhanced_eca(tbb::blocked_range2d(
                cases.begin(), cases.end(), options.begin(), options.end()));
        }

        auto options_ratios = instance.create_option_map(0.0);
        for(option_t o : options) {
            options_ratios[o] =
                (max_score - instance.eval_criterion(options_cases_eca[o])) /
                instance.option_cost(o);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] < options_ratios[o2];
        });

        std::vector<option_t> free_options;

        for(option_t option : options) {
            if(purchased <= budget) break;
            const double price = instance.option_cost(option);
            purchased -= price;
            solution[option] = false;
            free_options.emplace_back(option);
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

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP