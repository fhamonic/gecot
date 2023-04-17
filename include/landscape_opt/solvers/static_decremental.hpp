#ifndef LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP

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

struct StaticDecremental {
    bool verbose = false;
    bool parallel = false;
    bool only_dec = false;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        // int time_ms = 0;
        chronometer chrono;
        Solution solution = instance.create_solution();
        double purchaised = 0.0;

        std::vector<Option> options;
        std::vector<double> options_ratios(instance.options().size());

        for(auto && option : instance.options()) {
            const double price = instance.option_cost(option);
            purchaised += price;
            solution[option] = 1.0;
            options.emplace_back(option);
        }

        const auto vertexOptions = detail::computeOptionsForVertices(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        const QualityMap & original_qm = instance.landscape().quality_map();
        const ProbabilityMap & original_pm =
            instance.landscape().probability_map();

        QualityMap enhanced_qm = original_qm;
        ProbabilityMap enhanced_pm = original_pm;
        for(auto && option : options) {
            for(auto && [u, quality_gain] : vertexOptions[option])
                enhanced_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[option])
                enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
        }

        const double enhanced_eca =
            eca(instance.landscape().graph(), enhanced_qm, enhanced_pm);
        if(verbose) {
            std::cout << "ECA with all possible improvments: " << enhanced_eca
                      << std::endl;
        }

        auto compute_delta_eca_dec =
            [&](const tbb::blocked_range<decltype(options.begin())> &
                    options_block) {
                QualityMap qm = enhanced_qm;
                ProbabilityMap pm = enhanced_pm;

                for(auto it = options_block.begin();;) {
                    Option option = *it;
                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] -= quality_gain;
                    for(auto && [a, _] : arcOptions[option]) {
                        pm[a] = original_pm[a];
                        for(auto && [enhanced_prob, i] :
                            instance.arc_options_map()[a]) {
                            if(option == i) continue;
                            pm[a] = std::max(pm[a], enhanced_prob);
                        }
                    }

                    const double decreased_eca =
                        eca(instance.landscape().graph(), qm, pm);

                    options_ratios[option] = (enhanced_eca - decreased_eca) /
                                             instance.option_cost(option);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] += quality_gain;
                    for(auto && [a, _] : arcOptions[option])
                        pm[a] = enhanced_pm[a];
                }
            };

        if(parallel) {
            tbb::parallel_for(
                tbb::blocked_range(options.begin(), options.end()),
                compute_delta_eca_dec);
        } else {
            compute_delta_eca_dec(
                tbb::blocked_range(options.begin(), options.end()));
        }

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

        free_options.erase(
            std::remove_if(free_options.begin(), free_options.end(),
                           [&](Option i) {
                               return purchaised + instance.option_cost(i) >
                                      budget;
                           }),
            free_options.end());

        if(!only_dec && !free_options.empty()) {
            typename Landscape::QualityMap current_qm = original_qm;
            typename Landscape::ProbabilityMap current_pm = original_pm;

            for(Option option : options) {
                if(solution[option] == 0.0) continue;
                for(auto && [u, quality_gain] : vertexOptions[option])
                    current_qm[u] += quality_gain;
                for(auto && [a, enhanced_prob] : arcOptions[option])
                    current_pm[a] = std::max(current_pm[a], enhanced_prob);
            }

            const double current_eca =
                eca(instance.landscape().graph(), current_qm, current_pm);

            auto compute_delta_eca_inc =
                [&](const tbb::blocked_range<decltype(free_options.begin())> &
                        options_block) {
                    QualityMap qm = current_qm;
                    ProbabilityMap pm = current_pm;

                    for(auto it = options_block.begin();;) {
                        Option option = *it;
                        for(auto && [u, quality_gain] : vertexOptions[option])
                            qm[u] += quality_gain;
                        for(auto && [a, enhanced_prob] : arcOptions[option])
                            pm[a] = std::max(pm[a], enhanced_prob);

                        const double increased_eca =
                            eca(instance.landscape().graph(), qm, pm);

                        options_ratios[option] = (increased_eca - current_eca) /
                                                 instance.option_cost(option);

                        if(++it == options_block.end()) break;

                        for(auto && [u, quality_gain] : vertexOptions[option])
                            qm[u] = current_qm[u];
                        for(auto && [a, enhanced_prob] : arcOptions[option])
                            pm[a] = current_pm[a];
                    }
                };

            options_ratios.resize(free_options.size());

            if(parallel) {
                tbb::parallel_for(tbb::blocked_range(free_options.begin(),
                                                     free_options.end()),
                                  compute_delta_eca_inc);
            } else {
                compute_delta_eca_inc(tbb::blocked_range(free_options.begin(),
                                                         free_options.end()));
            }

            auto zipped_view_inc =
                ranges::view::zip(options_ratios, free_options);
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
        }
        // time_ms = chrono.time_ms();

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_DECREMENTAL_HPP