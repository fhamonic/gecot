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
    instance_option_map_t<I, bool> solve(const I & instance,
                                         const double budget) const {
        chronometer chrono;
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & o : instance.options()) {
            if(instance.option_cost(o) > budget) continue;
            options.emplace_back(o);
        }

        auto cases_base_eca = instance.template create_case_map<double>();
        auto compute_base_eca =
            [&cases_base_eca](
                const tbb::blocked_range<decltype(cases.begin())> &
                    cases_block) {
                for(auto instance_case : cases_block)
                    cases_base_eca[instance_case.id()] =
                        eca(instance_case.graph(),
                            instance_case.vertex_quality_map(),
                            instance_case.arc_probability_map());
            };
        if(parallel) {
            tbb::parallel_for(tbb::blocked_range(cases.begin(), cases.end()),
                              compute_base_eca);
        } else {
            compute_base_eca(tbb::blocked_range(cases.begin(), cases.end()));
        }
        const double base_score = instance.eval_criterion(cases_base_eca);

        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());

        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        auto compute_options_enhanced_eca =
            [&options_cases_eca, &cases_vertex_options, &cases_arc_options](
                const tbb::blocked_range2d<decltype(cases.begin()),
                                           decltype(options.begin())> &
                    cases_options_block) {
                for(auto instance_case : cases_options_block.rows()) {
                    auto && options_block = cases_options_block.cols();
                    if(options_block.begin() == options_block.end()) continue;

                    const auto & original_qm =
                        instance_case.vertex_quality_map();
                    const auto & original_pm =
                        instance_case.arc_probability_map();

                    auto enhanced_qm = original_qm;
                    auto enhanced_pm = original_pm;

                    const auto & vertex_options =
                        cases_vertex_options[instance_case.id()];
                    const auto & arc_options =
                        cases_arc_options[instance_case.id()];

                    for(auto it = options_block.begin();;) {
                        option_t option = *it;
                        for(auto && [u, quality_gain] : vertex_options[option])
                            enhanced_qm[u] += quality_gain;
                        for(auto && [a, enhanced_prob] : arc_options[option])
                            enhanced_pm[a] =
                                std::max(enhanced_pm[a], enhanced_prob);

                        options_cases_eca[option][instance_case.id()] = eca(
                            instance_case.graph(), enhanced_qm, enhanced_pm);

                        if(++it == options_block.end()) break;

                        for(auto && [u, quality_gain] : vertex_options[option])
                            enhanced_qm[u] = original_qm[u];
                        for(auto && [a, enhanced_prob] : arc_options[option])
                            enhanced_pm[a] = original_pm[a];
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
                (instance.eval_criterion(options_cases_eca[o]) - base_score) /
                instance.option_cost(o);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] > options_ratios[o2];
        });

        double purchaised = 0.0;
        for(option_t option : options) {
            const double price = instance.option_cost(option);
            if(purchaised + price > budget) continue;
            purchaised += price;
            solution[option] = true;
            if(verbose) {
                std::cout << "add option: " << option
                          << "\n\t costing: " << price
                          << "\n\t total cost: " << purchaised << std::endl;
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP