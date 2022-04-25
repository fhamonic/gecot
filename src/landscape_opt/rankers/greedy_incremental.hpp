#ifndef LANDSCAPE_OPT_SOLVERS_GREEDY_INCREMENTAL_HPP
#define LANDSCAPE_OPT_SOLVERS_GREEDY_INCREMENTAL_HPP

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
namespace rankers {

struct GreedyIncremental {
    bool verbose = false;
    bool parallel = false;

    template <concepts::Instance I>
    typename I::OptionPotentialMap rank_options(const I & instance) const {
        using Landscape = typename I::Landscape;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using OptionPotentialMap = typename I::OptionPotentialMap;

        std::vector<Option> options =
            ranges::to<std::vector>(instance.options());
        OptionPotentialMap options_potentials =
            instance.create_options_potentials_map();

        const auto nodeOptions = detail::computeOptionsForNodes(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        double prec_eca = eca(instance.landscape());
        if(verbose) {
            std::cout << "base ECA: " << prec_eca << std::endl;
        }

        QualityMap current_qm = instance.landscape().quality_map();
        ProbabilityMap current_pm = instance.landscape().probability_map();

        auto compute_delta_eca_inc =
            [&](const tbb::blocked_range<decltype(options.begin())> &
                    options_block,
                std::pair<double, Option> init) {
                QualityMap qm = current_qm;
                ProbabilityMap pm = current_pm;

                for(auto it = options_block.begin();;) {
                    Option option = *it;
                    for(auto && [u, quality_gain] : nodeOptions[option])
                        qm[u] += quality_gain;
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = std::max(pm[a], enhanced_prob);

                    const double increased_eca =
                        eca(instance.landscape().graph(), qm, pm);

                    const double ratio = (increased_eca - prec_eca) /
                                         instance.option_cost(option);

                    if(init.first < ratio) init = std::make_pair(ratio, option);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : nodeOptions[option])
                        qm[u] = current_qm[u];
                    for(auto && [a, enhanced_prob] : arcOptions[option])
                        pm[a] = current_pm[a];
                }

                return init;
            };

        while(!options.empty()) {
            std::pair<double, Option> best_option_p;
            if(parallel) {
                best_option_p = tbb::parallel_reduce(
                    tbb::blocked_range(options.begin(), options.end()),
                    std::pair<double, Option>(-1.0, -1), compute_delta_eca_inc,
                    [](auto && p1, auto && p2) {
                        return p1.first > p2.first ? p1 : p2;
                    });
            } else {
                best_option_p = compute_delta_eca_inc(
                    tbb::blocked_range(options.begin(), options.end()),
                    std::pair<double, Option>(-1.0, -1));
            }

            Option best_option = best_option_p.second;
            const double price = instance.option_cost(best_option);
            prec_eca += best_option_p.first * price;
            options_potentials[best_option] = options.size();

            for(auto && [u, quality_gain] : nodeOptions[best_option])
                current_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[best_option])
                current_qm[a] = std::max(current_qm[a], enhanced_prob);

            options.erase(
                std::remove(options.begin(), options.end(), best_option),
                options.end());

            if(verbose) {
                std::cout << "add option: " << best_option
                          << "\n\t costing: " << price << std::endl;
            }
        }

        return options_potentials;
    }
};

}  // namespace rankers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_GREEDY_INCREMENTAL_HPP