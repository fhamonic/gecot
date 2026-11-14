#ifndef GECOT_HELPER_HPP
#define GECOT_HELPER_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>

#include "melon/container/static_map.hpp"
#include "melon/graph.hpp"
#include "melon/mapping.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/indices/pc_num.hpp"

#include "gecot/utils/progress_bar.hpp"

namespace gecot {

// the default of -t, for the structs of the algorithms and their options on
// the command line
inline constexpr double default_feasibility_tol = 1e-7;

template <instance_c I>
auto compute_cases_vertex_options(const I & instance) noexcept {
    auto cases_vertex_options =
        instance.create_case_map(instance.create_option_map(
            std::vector<std::tuple<melon::vertex_t<instance_graph_t<I>>, double,
                                   double>>{}));

    for(auto && instance_case : instance.cases()) {
        const auto & vertex_options_map = instance_case.vertex_options_map();
        auto & vertex_options = cases_vertex_options[instance_case.id()];
        for(auto && u : melon::vertices(instance_case.graph())) {
            for(auto && [source_quality_gain, target_quality_gain, option] :
                vertex_options_map[u]) {
                vertex_options[option].emplace_back(u, source_quality_gain,
                                                    target_quality_gain);
            }
        }
    }
    return cases_vertex_options;
}

template <case_c C>
void compute_case_arc_options(const C & instance_case,
                              auto & arc_options) noexcept {
    const auto & arc_options_map = instance_case.arc_options_map();
    for(auto && a : melon::arcs(instance_case.graph())) {
        for(auto && [improved_probability, option] : arc_options_map[a])
            arc_options[option].emplace_back(a, improved_probability);
    }
}

template <instance_c I>
auto compute_cases_arc_options(const I & instance) noexcept {
    auto cases_arc_options =
        instance.create_case_map(instance.create_option_map(
            std::vector<
                std::pair<melon::arc_t<instance_graph_t<I>>, double>>{}));

    for(auto && instance_case : instance.cases()) {
        compute_case_arc_options(instance_case,
                                 cases_arc_options[instance_case.id()]);
    }
    return cases_arc_options;
}

template <instance_c I>
auto compute_cases_pc_num(const I & instance, const auto & cases_current_sqm,
                          const auto & cases_current_tqm,
                          const auto & cases_current_pm) noexcept {
    const auto & cases = instance.cases();
    auto cases_pc_num = instance.template create_case_map<double>();
    tbb::parallel_for(
        tbb::blocked_range(cases.begin(), cases.end()),
        [&](const tbb::blocked_range<decltype(cases.begin())> & cases_block) {
            for(const auto & instance_case : cases_block) {
                cases_pc_num[instance_case.id()] =
                    parallel_pc_num(instance_case.graph(),
                                    cases_current_sqm[instance_case.id()],
                                    cases_current_tqm[instance_case.id()],
                                    cases_current_pm[instance_case.id()]);
            }
        });
    return cases_pc_num;
}

template <instance_c I>
double compute_score(const I & instance, const auto & cases_current_sqm,
                     const auto & cases_current_tqm,
                     const auto & cases_current_pm) noexcept {
    return instance.eval_criterion(compute_cases_pc_num(
        instance, cases_current_sqm, cases_current_tqm, cases_current_pm));
}

template <instance_c I>
auto compute_base_cases_pc_num(const I & instance) noexcept {
    return compute_cases_pc_num(
        instance, melon::maps::function([&](auto && case_id) {
            return instance.cases()[case_id].source_quality_map();
        }),
        melon::maps::function([&](auto && case_id) {
            return instance.cases()[case_id].target_quality_map();
        }),
        melon::maps::function([&](auto && case_id) {
            return instance.cases()[case_id].arc_probability_map();
        }));
}

template <instance_c I>
double compute_base_score(const I & instance) noexcept {
    return instance.eval_criterion(compute_base_cases_pc_num(instance));
}

// not noexcept: option_cost(i) throws if the options have several costs
template <instance_c I, melon::mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
double compute_solution_cost(const I & instance, const S & solution) {
    double sum = 0;
    for(auto && option : instance.options())
        sum += static_cast<double>(solution[option]) *
               instance.option_cost(option);
    return sum;
}

// the cost of the solution in the dimension r
template <instance_c I, melon::mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
double compute_solution_cost(const I & instance, const S & solution,
                             const std::size_t r) {
    double sum = 0;
    for(auto && option : instance.options())
        sum += static_cast<double>(solution[option]) *
               instance.option_cost(option, r);
    return sum;
}

// within the budget of every dimension, up to the tolerance, and at most one
// option of each conflict set
template <instance_c I, melon::mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
bool is_feasible(const I & instance, const S & solution,
                 const budget_t & budget, const double tolerance) {
    for(std::size_t r = 0; r < budget.size(); ++r)
        if(compute_solution_cost(instance, solution, r) > budget[r] + tolerance)
            return false;
    return std::ranges::all_of(instance.conflict_sets(), [&](auto && set) {
        return std::ranges::count_if(set, [&](auto && i) {
                   return static_cast<bool>(solution[i]);
               }) <= 1;
    });
}

// the option alone within the budget of every dimension, up to the tolerance
template <instance_c I>
bool fits_alone(const I & instance, const option_t i, const budget_t & budget,
                const double tolerance = 0.0) {
    for(std::size_t r = 0; r < budget.size(); ++r)
        if(instance.option_cost(i, r) > budget[r] + tolerance) return false;
    return true;
}

// one cost dimension and no conflict set
template <instance_c I>
bool has_plain_budget(const I & instance) {
    return instance.num_cost_dimensions() == 1 &&
           std::ranges::empty(instance.conflict_sets());
}

// for the algorithms that read a single budget and no conflict set
template <instance_c I>
void check_plain_budget(const I & instance, const std::string & method) {
    if(!has_plain_budget(instance))
        throw std::invalid_argument(
            method +
            ": conflict sets and several cost dimensions are not supported "
            "yet");
}

// the first cost dimension of positive budget, 0 if none: the unit of the
// heuristics' costs
inline std::size_t reference_dimension(const budget_t & budget) {
    for(std::size_t r = 0; r < budget.size(); ++r)
        if(budget[r] > 0) return r;
    return 0;
}

// the cost that the heuristics divide a gain by: the sum over the dimensions
// of the costs over their budgets, in the unit of the reference dimension, so
// exactly the cost with a single dimension
template <instance_c I>
auto compute_heuristic_costs(const I & instance, const budget_t & budget) {
    const std::size_t r0 = reference_dimension(budget);
    auto costs = instance.create_option_map(0.0);
    for(auto && i : instance.options()) {
        costs[i] = instance.option_cost(i, r0);
        for(std::size_t r = 0; r < budget.size(); ++r)
            if(r != r0 && budget[r] > 0 && instance.option_cost(i, r) > 0)
                costs[i] += instance.option_cost(i, r) / budget[r] * budget[r0];
        if(!std::isfinite(costs[i]) ||
           (costs[i] != 0 && !std::isnormal(costs[i])))
            throw std::logic_error("the costs of option '" +
                                   instance.option_name(i) +
                                   "' are out of the range of the heuristics");
    }
    return costs;
}

// for each option, the indices of its conflict sets
template <instance_c I>
auto compute_options_conflict_sets(const I & instance) {
    auto options_sets =
        instance.template create_option_map<std::vector<std::size_t>>();
    std::size_t l = 0;
    for(auto && set : instance.conflict_sets()) {
        for(auto && i : set) options_sets[i].push_back(l);
        ++l;
    }
    return options_sets;
}

template <instance_c I, melon::mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
auto compute_solution_cases_pc_num(const I & instance, const S & solution,
                                   const auto & cases_vertex_options,
                                   const auto & cases_arc_options) noexcept {
    return compute_cases_pc_num(
        instance, melon::maps::function([&](auto && case_id) {
            auto enhanced_sqm = instance.cases()[case_id].source_quality_map();
            auto && vertex_options = cases_vertex_options[case_id];
            for(auto && option : instance.options()) {
                if(!solution[option]) continue;
                for(auto && [u, source_quality_gain, _] :
                    vertex_options[option])
                    enhanced_sqm[u] += source_quality_gain;
            }
            return enhanced_sqm;
        }),
        melon::maps::function([&](auto && case_id) {
            auto enhanced_tqm = instance.cases()[case_id].target_quality_map();
            auto && vertex_options = cases_vertex_options[case_id];
            for(auto && option : instance.options()) {
                if(!solution[option]) continue;
                for(auto && [u, _, target_quality_gain] :
                    vertex_options[option])
                    enhanced_tqm[u] += target_quality_gain;
            }
            return enhanced_tqm;
        }),
        melon::maps::function([&](auto && case_id) {
            auto enhanced_pm = instance.cases()[case_id].arc_probability_map();
            auto && arc_options = cases_arc_options[case_id];
            for(auto && option : instance.options()) {
                if(!solution[option]) continue;
                for(auto && [a, improved_probability] : arc_options[option]) {
                    enhanced_pm[a] =
                        std::max(enhanced_pm[a], improved_probability);
                }
            }
            return enhanced_pm;
        }));
}

template <instance_c I, melon::mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
auto compute_solution_cases_pc_num(const I & instance,
                                   const S & solution) noexcept {
    const auto cases_vertex_options = compute_cases_vertex_options(instance);
    const auto cases_arc_options = compute_cases_arc_options(instance);
    return compute_solution_cases_pc_num(
        instance, solution, cases_vertex_options, cases_arc_options);
}

}  // namespace gecot

#endif  // GECOT_HELPER_HPP