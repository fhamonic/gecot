#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP

#include <tbb/blocked_range.h>
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

struct StaticIncremental {
    bool verbose = false;
    bool parallel = false;

    template <instance_c I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        // int time_ms = 0;
        chronometer chrono;
        Solution solution = instance.create_solution();

        const auto vertexOptions = detail::computeOptionsForVertices(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        const double base_eca = eca(instance.landscape());
        if(verbose) {
            std::cout << "base ECA: " << base_eca << std::endl;
            std::cout << "total cost: 0" << std::endl;
        }

        std::vector<Option> options =
            ranges::to<std::vector>(instance.options());
        std::vector<double> options_ratios(options.size());

        options.erase(
            std::remove_if(
                options.begin(), options.end(),
                [&](Option i) { return instance.option_cost(i) > budget; }),
            options.end());

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