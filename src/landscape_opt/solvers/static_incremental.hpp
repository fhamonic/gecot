#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "concepts/instance.hpp"
#include "helper.hpp"
#include "indices/eca.hpp"
#include "utils/chrono.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct StaticIncremental {
    bool verbose = false;
    bool parallel = false;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        int time_ms = 0;
        Chrono chrono;
        Solution solution = instance.create_solution();

        const auto nodeOptions = detail::computeOptionsForNodes(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        const double base_eca = eca(instance.landscape());
        if(verbose) {
            std::cout << "base ECA: " << base_eca << std::endl;
            std::cout << "total cost: 0" << std::endl;
        }

        std::vector<Option> options =
            ranges::to<std::vector>(instance.options());
        std::vector<double> options_ratios(options.size());

        auto compute_delta_eca_inc =
            [&instance, &nodeOptions, &arcOptions, base_eca, &options_ratios](
                const tbb::blocked_range<Option> & options_block) {
                const QualityMap & original_qm =
                    instance.landscape().quality_map();
                const ProbabilityMap & original_pm =
                    instance.landscape().probability_map();

                QualityMap qm = original_qm;
                ProbabilityMap pm = original_pm;

                for(Option option = options_block.begin();;) {
                    for(auto && [u, quality_gain] : nodeOptions[option])
                        qm[u] += quality_gain;
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = std::max(pm[a], enhanced_prob);

                    const double increased_eca =
                        eca(instance.landscape().graph(), qm, pm);

                    options_ratios[option] = (increased_eca - base_eca) /
                                             instance.option_cost(option);

                    if(++option == options_block.end()) break;

                    for(auto && [u, quality_gain] : nodeOptions[option])
                        qm[u] = original_qm[u];
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = original_pm[a];
                }
            };

        if(parallel) {
            auto options_range = instance.options();
            tbb::parallel_for(
                tbb::blocked_range<Option>(0, instance.options().size()),
                compute_delta_eca_inc);
        } else {
            compute_delta_eca_inc(
                tbb::blocked_range<Option>(0, instance.options().size()));
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

        time_ms = chrono.timeMs();

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP