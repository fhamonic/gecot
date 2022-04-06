#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP

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

struct StaticDecremental {
    bool verbose = false;
    bool parallel = false;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        Chrono chrono;
        Solution solution = instance.create_solution();
        int time_ms = 0;

        const auto nodeOptions = detail::computeOptionsForNodes(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        QualityMap quality_map = instance.landscape().quality_map();
        ProbabilityMap probability_map = instance.landscape().probability_map();

        QualityMap enhanced_qm = quality_map;
        ProbabilityMap enhanced_pm = probability_map;
        for(auto && option : instance.options()) {
            for(auto && [u, quality_gain] : nodeOptions[option])
                enhanced_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[option])
                enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
        }

        const double enhanced_eca =
            eca(instance.landscape().graph(), enhanced_qm, enhanced_pm);
        if(verbose) {
            std::cout << "ECA with all improvments: " << enhanced_eca
                      << std::endl;
        }

        std::vector<Option> options =
            ranges::to<std::vector>(instance.options());
        std::vector<double> options_ratios(options.size());

        double purchaised = 0.0;
        for(auto && option : instance.options()) {
            purchaised += instance.option_cost(option);
            solution[option] = 1.0;
        }

        // auto compute_delta_eca_dec =
        //     [&](const tbb::blocked_range<std::vector<Option>::iterator> &
        //             options_block) {
        //         QualityMap qm = enhanced_qm;
        //         ProbabilityMap pm = enhanced_pm;

        //         for(auto it = options_block.begin();;) {
        //             Option option = *it;
        //             for(auto && [u, quality_gain] : nodeOptions[option])
        //                 qm[u] -= quality_gain;
        //             for(auto && [a, enhanced_prob] : arcOptions[option]) {
        //                 pm[a] = probability_map[a];
        //                 for(auto && [enhanced_prob, i] :
        //                     instance.arc_options_map()[a]) {
        //                     if(option == i) continue;
        //                     pm[a] = std::max(pm[a], enhanced_prob);
        //                 }
        //             }

        //             const double increased_eca =
        //                 eca(instance.landscape().graph(), qm, pm);

        //             options_ratios[option] = (enhanced_eca - decreased_eca) /
        //                                      instance.option_cost(option);

        //             if(++it == options_block.end()) break;

        //             for(auto && [u, quality_gain] : nodeOptions[option])
        //                 qm[u] = enhanced_qm[u];
        //             for(auto && [a, enhanced_prob] : arcOptions[option])
        //                 pm[a] = enhanced_pm[a];
        //         }
        //     };

        // if(parallel) {
        //     tbb::parallel_for(
        //         tbb::blocked_range(options.begin(), options.end()),
        //         compute_delta_eca_dec);
        // } else {
        //     compute_delta_eca_dec(
        //         tbb::blocked_range(options.begin(), options.end()));
        // }

        auto zipped_view_dec = ranges::view::zip(options_ratios, options);
        ranges::sort(zipped_view_dec, [](auto && e1, auto && e2) {
            return e1.first < e2.first;
        });

        std::vector<Option> free_options;

        for(Option option : options) {
            if(purchaised <= budget) break;
            const double price = instance.option_cost(option);
            purchaised -= price;
            solution[option] = 0.0;
            free_options.emplace_back(option);
            if(verbose) {
                std::cout << "remove option: " << option
                          << "\n\t costing: " << price
                          << "\n\t total cost: " << purchaised << std::endl;
            }
        }

        typename Landscape::QualityMap current_qm = quality_map;
        typename Landscape::ProbabilityMap current_pm = probability_map;

        for(Option option : options) {
            if(solution[option] == 0.0) continue;
            for(auto && [u, quality_gain] : nodeOptions[option])
                current_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[option])
                current_pm[a] = std::max(current_pm[a], enhanced_prob);
        }

        const double current_eca =
            eca(instance.landscape().graph(), current_qm, current_pm);

        // auto compute_delta_eca_inc =
        //     [&](const tbb::blocked_range<std::vector<Option>::const_iterator> &
        //             options_block) {
                // QualityMap qm = current_qm;
                // ProbabilityMap pm = current_pm;

                // for(auto it = options_block.begin();;) {
                //     Option option = *it;
                //     for(auto && [u, quality_gain] : nodeOptions[option])
                //         qm[u] += quality_gain;
                //     for(auto && [a, enhanced_prob] : arcOptions[option])
                //         pm[a] = std::max(pm[a], enhanced_prob);

                //     const double increased_eca =
                //         eca(instance.landscape().graph(), qm, pm);

                //     options_ratios[option] = (increased_eca - current_eca) /
                //                              instance.option_cost(option);

                //     if(++it == options_block.end()) break;

                //     for(auto && [u, quality_gain] : nodeOptions[option])
                //         qm[u] = current_qm[u];
                //     for(auto && [a, enhanced_prob] : arcOptions[option])
                //         pm[a] = current_pm[a];
                // }
        //         }
        //     ;

        // options_ratios.resize(free_options.size());

        // if(parallel) {
        //     tbb::parallel_for(
        //         tbb::blocked_range(free_options.begin(), free_options.end()),
        //         compute_delta_eca_inc);
        // } else {
        //     compute_delta_eca_inc(
        //         tbb::blocked_range(free_options.begin(), free_options.end()));
        // }

        auto zipped_view_inc = ranges::view::zip(options_ratios, free_options);
        ranges::sort(zipped_view_inc, [](auto && e1, auto && e2) {
            return e1.first > e2.first;
        });

        for(Option option : free_options) {
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

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP