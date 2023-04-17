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
namespace rankers {

struct GreedyDecremental {
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
                    for(auto && [a, _] : arcOptions[option]) {
                        pm[a] = original_pm[a];
                        for(auto && [current_prob, i] :
                            instance.arc_options_map()[a]) {
                            if(options_potentials[i] != 0.0 || option == i)
                                continue;
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
                        qm[u] += quality_gain;
                    for(auto && [a, _] : arcOptions[option])
                        pm[a] = current_pm[a];
                }

                return init;
            };

        const std::size_t nb_options = options.size();

        while(!options.empty()) {
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
            current_eca -= worst_option_p.first * price;
            options_potentials[worst_option] = static_cast<double>(nb_options - options.size() + 1);

            for(auto && [u, quality_gain] : vertexOptions[worst_option])
                current_qm[u] -= quality_gain;
            for(auto && [a, _] : arcOptions[worst_option]) {
                current_pm[a] = original_pm[a];
                for(auto && [current_prob, i] : instance.arc_options_map()[a]) {
                    if(options_potentials[i] != 0.0) continue;
                    current_pm[a] = std::max(current_pm[a], current_prob);
                }
            }

            options.erase(
                std::remove(options.begin(), options.end(), worst_option),
                options.end());
        }

        return options_potentials;
    }
};

}  // namespace rankers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_GREEDY_DECREMENTAL_HPP