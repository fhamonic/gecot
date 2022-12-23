#ifndef LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

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

struct GreedyDecremental {
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
        Chrono chrono;
        Solution solution = instance.create_solution();
        double purchaised = 0.0;

        std::vector<Option> options;
        std::vector<Option> free_options;
        for(auto && option : instance.options()) {
            const double price = instance.option_cost(option);
            if(price > budget) continue;
            purchaised += price;
            solution[option] = 1.0;
            options.emplace_back(option);
        }

        const auto vertexOptions = detail::computeOptionsForVertices(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        const QualityMap & original_qm = instance.landscape().quality_map();
        const ProbabilityMap & original_pm =
            instance.landscape().probability_map();

        QualityMap current_qm = original_qm;
        ProbabilityMap current_pm = original_pm;
        for(auto && option : options) {
            for(auto && [u, quality_gain] : vertexOptions[option])
                current_qm[u] += quality_gain;
            for(auto && [a, current_prob] : arcOptions[option])
                current_pm[a] = std::max(current_pm[a], current_prob);
        }

        double current_eca =
            eca(instance.landscape().graph(), current_qm, current_pm);
        if(verbose) {
            std::cout << "ECA with all possible improvments: " << current_eca
                      << std::endl;
        }

        auto compute_delta_eca_dec =
            [&](const tbb::blocked_range<decltype(options.begin())> &
                    options_block,
                std::pair<double, Option> init) {
                QualityMap qm = current_qm;
                ProbabilityMap pm = current_pm;

                for(auto it = options_block.begin();;) {
                    Option option = *it;
                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] -= quality_gain;
                    for(auto && [a, current_prob] : arcOptions[option]) {
                        pm[a] = original_pm[a];
                        for(auto && [current_prob, i] :
                            instance.arc_options_map()[a]) {
                            if(solution[i] == 0.0 || option == i) continue;
                            pm[a] = std::max(pm[a], current_prob);
                        }
                    }

                    const double decreased_eca =
                        eca(instance.landscape().graph(), qm, pm);

                    const double ratio = (current_eca - decreased_eca) /
                                         instance.option_cost(option);

                    if(init.first > ratio) init = std::make_pair(ratio, option);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : vertexOptions[option])
                        qm[u] = current_qm[u];
                    for(auto && [a, current_prob] : arcOptions[option])
                        pm[a] = current_pm[a];
                }

                return init;
            };

        while(purchaised > budget) {
            std::pair<double, Option> worst_option_p;
            if(parallel) {
                worst_option_p = tbb::parallel_reduce(
                    tbb::blocked_range(options.begin(), options.end()),
                    std::pair<double, Option>(
                        std::numeric_limits<double>::max(), -1),
                    compute_delta_eca_dec, [](auto && p1, auto && p2) {
                        return p1.first > p2.first ? p1 : p2;
                    });
            } else {
                worst_option_p = compute_delta_eca_dec(
                    tbb::blocked_range(options.begin(), options.end()),
                    std::pair<double, Option>(
                        std::numeric_limits<double>::max(), -1));
            }

            Option worst_option = worst_option_p.second;
            const double price = instance.option_cost(worst_option);
            purchaised -= price;
            solution[worst_option] = 0.0;
            current_eca -= worst_option_p.first * price;
            free_options.emplace_back(worst_option);

            for(auto && [u, quality_gain] : vertexOptions[worst_option])
                current_qm[u] -= quality_gain;
            for(auto && [a, current_prob] : arcOptions[worst_option]) {
                current_pm[a] = original_pm[a];
                for(auto && [current_prob, i] : instance.arc_options_map()[a]) {
                    if(solution[i] == 0.0) continue;
                    current_pm[a] = std::max(current_pm[a], current_prob);
                }
            }

            options.erase(
                std::remove(options.begin(), options.end(), worst_option),
                options.end());
        }

        // std::vector<Option> options =
        //     ranges::to<std::vector>(instance.options());

        // QualityMap current_qm = instance.landscape().quality_map();
        // ProbabilityMap current_pm = instance.landscape().probability_map();

        // auto compute_delta_eca_inc =
        //     [&](const tbb::blocked_range<decltype(options.begin())> &
        //             options_block,
        //         std::pair<double, Option> init) {
        //         QualityMap qm = current_qm;
        //         ProbabilityMap pm = current_pm;

        //         for(auto it = options_block.begin();;) {
        //             Option option = *it;
        //             for(auto && [u, quality_gain] : vertexOptions[option])
        //                 qm[u] += quality_gain;
        //             for(auto && [a, current_prob] : arcOptions[option])
        //                 pm[a] = std::max(pm[a], current_prob);

        //             const double increased_eca =
        //                 eca(instance.landscape().graph(), qm, pm);

        //             const double ratio = (increased_eca - current_eca) /
        //                                  instance.option_cost(option);

        //             if(init.first < ratio) init = std::make_pair(ratio,
        //             option);

        //             if(++it == options_block.end()) break;

        //             for(auto && [u, quality_gain] : vertexOptions[option])
        //                 qm[u] = current_qm[u];
        //             for(auto && [a, current_prob] : arcOptions[option])
        //                 pm[a] = current_pm[a];
        //         }

        //         return init;
        //     };

        // double purchaised = 0.0;
        // for(;;) {
        //     options.erase(
        //         std::remove_if(options.begin(), options.end(),
        //                        [&](Option i) {
        //                            return purchaised +
        //                            instance.option_cost(i) >
        //                                   budget;
        //                        }),
        //         options.end());

        //     if(options.empty()) break;

        //     std::pair<Option, double> best_option_p;
        //     if(parallel) {
        //         best_option_p = tbb::parallel_reduce(
        //             tbb::blocked_range(options.begin(), options.end()),
        //             std::pair<double, Option>(-1.0, -1),
        //             compute_delta_eca_inc,
        //             [](auto && p1, auto && p2) {
        //                 return p1.first > p2.first ? p1 : p2;
        //             });
        //     } else {
        //         best_option_p = compute_delta_eca_inc(
        //             tbb::blocked_range(options.begin(), options.end()),
        //             std::pair<double, Option>(-1.0, -1));
        //     }

        //     const double price = instance.option_cost(best_option_p.first);
        //     purchaised += price;
        //     solution[best_option_p.first] = 1.0;
        //     current_eca += best_option_p.second * price;

        //     if(verbose) {
        //         std::cout << "add option: " << best_option_p.first
        //                   << "\n\t costing: " << price
        //                   << "\n\t total cost: " << purchaised << std::endl;
        //     }
        // }
        if(!only_dec) {
            auto compute_delta_eca_inc =
                [&](const tbb::blocked_range<decltype(options.begin())> &
                        options_block,
                    std::pair<double, Option> init) {
                    QualityMap qm = current_qm;
                    ProbabilityMap pm = current_pm;

                    for(auto it = options_block.begin();;) {
                        Option option = *it;
                        for(auto && [u, quality_gain] : vertexOptions[option])
                            qm[u] += quality_gain;
                        for(auto && [a, current_prob] : arcOptions[option])
                            pm[a] = std::max(pm[a], current_prob);

                        const double increased_eca =
                            eca(instance.landscape().graph(), qm, pm);

                        const double ratio = (increased_eca - current_eca) /
                                             instance.option_cost(option);

                        if(init.first < ratio)
                            init = std::make_pair(ratio, option);

                        if(++it == options_block.end()) break;

                        for(auto && [u, quality_gain] : vertexOptions[option])
                            qm[u] = current_qm[u];
                        for(auto && [a, current_prob] : arcOptions[option])
                            pm[a] = current_pm[a];
                    }

                    return init;
                };

            for(;;) {
                options.erase(
                    std::remove_if(options.begin(), options.end(),
                                   [&](Option i) {
                                       return purchaised +
                                                  instance.option_cost(i) >
                                              budget;
                                   }),
                    options.end());

                if(options.empty()) break;

                std::pair<double, Option> best_option_p;
                if(parallel) {
                    best_option_p = tbb::parallel_reduce(
                        tbb::blocked_range(options.begin(), options.end()),
                        std::pair<double, Option>(-1.0, -1),
                        compute_delta_eca_inc, [](auto && p1, auto && p2) {
                            return p1.first > p2.first ? p1 : p2;
                        });
                } else {
                    best_option_p = compute_delta_eca_inc(
                        tbb::blocked_range(options.begin(), options.end()),
                        std::pair<double, Option>(-1.0, -1));
                }

                const double price = instance.option_cost(best_option_p.second);
                purchaised += price;
                solution[best_option_p.second] = 1.0;
                current_eca += best_option_p.first * price;

                options.erase(std::remove(options.begin(), options.end(),
                                          best_option_p.second),
                              options.end());

                if(verbose) {
                    std::cout << "add option: " << best_option_p.second
                              << "\n\t costing: " << price
                              << "\n\t total cost: " << purchaised << std::endl;
                }
            }
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP