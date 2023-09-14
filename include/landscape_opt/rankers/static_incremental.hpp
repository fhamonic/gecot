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
namespace rankers {

struct StaticIncremental {
    bool verbose = false;
    bool parallel = false;

    template <instance_c I>
    instance_options_rank_t<I> rank_options(const I & instance) const {
        auto options_rank = instance.create_option_map(0u);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & option : instance.options()) {
            options.emplace_back(option);
        }

        const double base_score = compute_base_score(instance, parallel);
        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        compute_options_cases_incr_eca(
            instance, options,
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].vertex_quality_map();
            }),
            melon::views::map([&cases](auto case_id) -> decltype(auto) {
                return cases[case_id].arc_probability_map();
            }),
            cases_vertex_options, cases_arc_options, options_cases_eca,
            parallel);

        auto options_ratios = instance.create_option_map(0.0);
        for(const option_t & option : options) {
            options_ratios[option] =
                (instance.eval_criterion(options_cases_eca[option]) -
                 base_score) /
                instance.option_cost(option);
        }
        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] > options_ratios[o2];
        });

        unsigned int rank = 0;
        for(const option_t & option : options) {
            options_rank[option] = rank;
            if(verbose) {
                std::cout << "ranked option: " << option
                          << "\n\t rank: " << rank
                          << "\n\t ratio: " << options_ratios[option]
                          << std::endl;
            }
            ++rank;
        }

        return options_rank;
    }
};

}  // namespace rankers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_STATIC_INCREMENTAL_HPP