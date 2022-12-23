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
namespace rankers {

struct StaticIncremental {
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

        const double base_eca = eca(instance.landscape());
        if(verbose) {
            std::cout << "base ECA: " << base_eca << std::endl;
        }

        const QualityMap & original_qm = instance.landscape().quality_map();
        const ProbabilityMap & original_pm =
            instance.landscape().probability_map();

        auto compute_delta_eca_inc =
            [&](const tbb::blocked_range<decltype(options.begin())> &
                    options_block) {
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

                    options_potentials[option] = (increased_eca - base_eca) /
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

        return options_potentials;
    }
};

}  // namespace rankers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP