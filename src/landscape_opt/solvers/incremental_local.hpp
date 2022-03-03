#ifndef LANDSCAPE_OPT_SOLVERS_INCREMENTAL_LOCAL_HPP
#define LANDSCAPE_OPT_SOLVERS_INCREMENTAL_LOCAL_HPP

#include <execution>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/zip.hpp>

#include "concepts/instance.hpp"
#include "helper.hpp"
#include "indices/eca.hpp"
#include "utils/chrono.hpp"
#include "utils/random_chooser.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct IncrementalLocal {
    int seed = 314159265;
    int log_level = 0;
    bool parallel = false;

    int time_ms = 0;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        Chrono chrono;
        Solution solution = instance.create_solution();

        const auto nodeOptions = detail::computeOptionsForNodes(instance);
        const auto arcOptions = detail::computeOptionsForArcs(instance);

        const double base_eca = eca(instance.landscape);
        if(log_level > 1) {
            std::cout << "base ECA: " << base_eca << std::endl;
        }

        std::vector<Option> options(instance.options());
        std::vector<double> options_ratios(options.size());

        auto compute = [&instance, &nodeOptions, &arcOptions,
                        base_eca](Option option) {
            typename Landscape::QualityMap qm = instance.landscape.quality_map();
            typename Landscape::ProbabilityMap pm = instance.landscape.probability_map();

            for(auto && [u, quality_gain] : nodeOptions[option])
                qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arcOptions[option])
                pm[a] = std::max(pm[a], enhanced_prob);

            const double increased_eca =
                eca(instance.landscape.graph(), qm, pm);
            const double ratio =
                (increased_eca - base_eca) / instance.option_cost(option);

            return ratio;
        };

        if(parallel)
            std::ranges::transform(std::execution::par_unseq, options,
                                   options_ratios.begin(), compute);
        else
            std::ranges::transform(std::execution::seq, options,
                                   options_ratios.begin(), compute);

        auto zipped_view = ranges::view::zip(options_ratios, options);
        ranges::sort(zipped_view, [](std::pair<double, Option> & e1,
                                     std::pair<double, Option> & e2) {
            return e1.first > e2.first;
        });

        double purchaised = 0.0;
        for(Option option : options) {
            const double price = instance.option_cost(option);
            if(purchaised + price > budget) continue;
            purchaised += price;
            solution[option] = 1.0;
            if(log_level > 1)
                std::cout << "add option: " << option
                          << "\t purchaised: " << purchaised << std::endl;
        }

        time_ms = chrono.timeMs();

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_INCREMENTAL_LOCAL_HPP