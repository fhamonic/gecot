#ifndef GECOT_SOLVERS_GREEDY_DECREMENTAL_HPP
#define GECOT_SOLVERS_GREEDY_DECREMENTAL_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/utils/chronometer.hpp"

namespace fhamonic {
namespace gecot {
namespace rankers {

struct GreedyDecremental {
    bool parallel = false;
    bool only_dec = false;

    template <instance_c I>
    instance_options_rank_t<I> rank_options(const I & instance) const {
        auto options_rank = instance.create_option_map(0.0);
        auto solution = instance.create_option_map(false);

        const auto & cases = instance.cases();
        std::vector<option_t> options;
        for(const option_t & option : instance.options()) {
            options.emplace_back(option);
            solution[option] = true;
        }

        auto options_cases_pc_num = instance.create_option_map(
            instance.template create_case_map<double>());
        auto cases_current_qm =
            instance.template create_case_map<instance_quality_map_t<I>>();
        auto cases_current_pm =
            instance.template create_case_map<instance_probability_map_t<I>>();
        const auto cases_vertex_options =
            compute_cases_vertex_options(instance);
        const auto cases_arc_options = compute_cases_arc_options(instance);
        auto options_ratios = instance.create_option_map(0.0);

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

        unsigned int rank = options.size();
        double previous_score = compute_score(instance, cases_current_qm,
                                              cases_current_pm);
        while(!options.empty()) {
            compute_options_cases_decr_pc_num(
                instance, solution, options, cases_current_qm,
                cases_current_pm, cases_vertex_options, cases_arc_options,
                options_cases_pc_num);
            for(const option_t & option : options) {
                options_ratios[option] =
                    (previous_score -
                     instance.eval_criterion(options_cases_pc_num[option])) /
                    instance.option_cost(option);
            }
            auto worst_option_it = std::ranges::min_element(
                options, [&options_ratios](auto && o1, auto && o2) {
                    return options_ratios[o1] < options_ratios[o2];
                });
            option_t worst_option = *worst_option_it;
            const double worst_option_price =
                instance.option_cost(worst_option);
            previous_score -= options_ratios[worst_option] * worst_option_price;
            options_rank[worst_option] = rank;
            solution[worst_option] = false;
            for(const auto & instance_case : cases) {
                auto & current_qm = cases_current_qm[instance_case.id()];
                auto & current_pm = cases_current_pm[instance_case.id()];
                const auto & original_pm = instance_case.arc_probability_map();
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];
                for(auto && [u, quality_gain] : vertex_options[worst_option])
                    current_qm[u] -= quality_gain;
                for(auto && [a, _] : arc_options[worst_option]) {
                    current_pm[a] = original_pm[a];
                    for(auto && [current_prob, i] :
                        instance_case.arc_options_map()[a]) {
                        if(!solution[i] || worst_option == i) continue;
                        current_pm[a] = std::max(current_pm[a], current_prob);
                    }
                }
            }
            options.erase(worst_option_it);

            // std::cout << "ranked option: " << worst_option
            //           << "\n\t rank: " << rank
            //           << "\n\t ratio: " << options_ratios[worst_option]
            //           << "\n\t costing: " << worst_option_price
            //           << std::endl;
            
            --rank;
        }

        return options_rank;
    }
};

}  // namespace rankers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_GREEDY_DECREMENTAL_HPP