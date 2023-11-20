#ifndef GECOT_SOLVERS_STATIC_DECREMENTAL_HPP
#define GECOT_SOLVERS_STATIC_DECREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/eca.hpp"
#include "gecot/utils/chronometer.hpp"

namespace fhamonic {
namespace gecot {
namespace rankers {

struct StaticDecremental {
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

        auto options_cases_eca = instance.create_option_map(
            instance.template create_case_map<double>());
        auto cases_current_qm =
            instance.template create_case_map<instance_quality_map_t<I>>();
        auto cases_current_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);

        for(const auto & instance_case : cases) {
            auto & current_qm = (cases_current_qm[instance_case.id()] =
                                     instance_case.vertex_quality_map());
            auto & current_pm = (cases_current_pm[instance_case.id()] =
                                     instance_case.arc_probability_map());
            const auto & vertex_options =
                cases_vertex_options[instance_case.id()];
            const auto & arc_options = cases_arc_options[instance_case.id()];
            for(auto && option : options) {
                for(auto && [u, quality_gain] : vertex_options[option])
                    current_qm[u] += quality_gain;
                for(auto && [a, enhanced_prob] : arc_options[option])
                    current_pm[a] = std::max(current_pm[a], enhanced_prob);
            }
        }

        const double max_score =
            compute_score(instance, cases_current_qm, cases_current_pm);
        compute_options_cases_decr_eca(
            instance, melon::views::map([](option_t o) { return true; }),
            options, cases_current_qm, cases_current_pm, cases_vertex_options,
            cases_arc_options, options_cases_eca, parallel);

        auto options_ratios = instance.create_option_map(0.0);
        for(auto && option : options) {
            options_ratios[option] =
                (max_score -
                 instance.eval_criterion(options_cases_eca[option])) /
                instance.option_cost(option);
        }

        std::ranges::sort(options, [&options_ratios](auto && o1, auto && o2) {
            return options_ratios[o1] < options_ratios[o2];
        });

        unsigned int rank = options.size();
        for(auto && option : options) {
            options_rank[option] = rank;
            if(verbose) {
                std::cout << "ranked option: " << option
                          << "\n\t rank: " << rank
                          << "\n\t ratio: " << options_ratios[option]
                          << std::endl;
            }
            --rank;
        }

        return options_rank;
    }
};

}  // namespace rankers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_STATIC_DECREMENTAL_HPP