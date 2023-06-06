#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP

#include <tbb/blocked_range.h>
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
        auto solution = instance.template create_option_map<I, bool>();

        std::vector<option_t> options =
            ranges::to<std::vector>(ranges::views::filter(instance.options(), [&](option_t i) { return instance.option_cost(i) <= budget; }));

        instance_option_map_t<I, instance_case_map_t<I, double>>
            options_enhanced_eca = instance.template create_option_map<
                instance_case_map_t<I, double>>(instance.template create_case_map<double>());

        // const double base_score = eca(instance.landscape());
        // if(verbose) {
        //     std::cout << "base ECA: " << base_eca << std::endl;
        //     std::cout << "total cost: 0" << std::endl;
        // }


        for(const auto & instance_case : instance.cases()) {
            const auto vertexOptions = detail::computeOptionsForVertices(
                instance_case, instance.nb_options());
            const auto arcOptions = detail::computeOptionsForArcs(
                instance_case, instance.nb_options());
        }


        
        auto options_ratios = instance.template create_option_map<I, double>();

        auto compute_delta_eca_inc =
            [&instance, &vertexOptions, &arcOptions, base_eca, &options_ratios](
                const tbb::blocked_range<decltype(options.begin())> &
                    options_block) {
                const QualityMap & original_qm =
                    instance.landscape().quality_map();
                const ProbabilityMap & original_pm =
                    instance.landscape().probability_map();

                QualityMap qm = original_qm;
                ProbabilityMap pm = original_pm;

                for(auto it = options_block.begin();;) {
                    Option option = *it;
                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] += quality_gain;
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = std::max(pm[a], enhanced_prob);

                    const double increased_eca =
                        eca(instance.landscape().graph(), qm, pm);

                    options_ratios[option] = (increased_eca - base_eca) /
                                             instance.option_cost(option);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] = original_qm[u];
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = original_pm[a];
                }
            };

        if(parallel) {
            tbb::parallel_for(
                tbb::blocked_range(options.begin(), options.end()),
                compute_delta_eca_inc);
        } else {
            compute_delta_eca_inc(
                tbb::blocked_range(options.begin(), options.end()));
        }

        auto zipped_view = ranges::view::zip(options_ratios, options);
        ranges::sort(zipped_view, [](auto && e1, auto && e2) {
            return e1.first > e2.first;
        });

        double purchaised = 0.0;
        for(Option option : options) {
            const double price = instance.option_cost(option);
            if(purchaised + price > budget) continue;
            purchaised += price;
            solution[option] = 1.0;
            if(verbose) {
                std::cout << "add option: " << option
                          << "\n\t costing: " << price
                          << "\n\t total cost: " << purchaised << std::endl;
            }
        }

        // time_ms = chrono.time_ms();

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP