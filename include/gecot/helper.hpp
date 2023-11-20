#ifndef GECOT_HELPER_HPP
#define GECOT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "melon/container/static_map.hpp"
#include "melon/graph.hpp"
#include "melon/mapping.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/eca.hpp"
#include "gecot/indices/parallel_eca.hpp"

namespace fhamonic {
namespace gecot {

template <instance_c I>
auto compute_cases_vertex_options(const I & instance) noexcept {
    auto cases_vertex_options =
        instance.create_case_map(instance.create_option_map(
            std::vector<
                std::pair<melon::vertex_t<instance_graph_t<I>>, double>>{}));

    for(auto && instance_case : instance.cases()) {
        const auto & vertex_options_map = instance_case.vertex_options_map();
        auto & vertex_options = cases_vertex_options[instance_case.id()];
        for(auto && u : melon::vertices(instance_case.graph())) {
            for(auto && [quality_gain, option] : vertex_options_map[u]) {
                vertex_options[option].emplace_back(u, quality_gain);
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
        for(auto && [enhanced_prob, option] : arc_options_map[a])
            arc_options[option].emplace_back(a, enhanced_prob);
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
double compute_score(const I & instance, const auto & cases_current_qm,
                     const auto & cases_current_pm,
                     const bool parallel = false) noexcept {
    const auto & cases = instance.cases();
    auto cases_eca = instance.template create_case_map<double>();
    auto compute_base_eca =
        [&](const tbb::blocked_range<decltype(cases.begin())> & cases_block) {
            for(auto instance_case : cases_block) {
                cases_eca[instance_case.id()] = eca(
                    instance_case.graph(), cases_current_qm[instance_case.id()],
                    cases_current_pm[instance_case.id()]);
            }
        };
    if(parallel) {
        tbb::parallel_for(tbb::blocked_range(cases.begin(), cases.end()),
                          compute_base_eca);
    } else {
        compute_base_eca(tbb::blocked_range(cases.begin(), cases.end()));
    }
    return instance.eval_criterion(cases_eca);
}

template <instance_c I>
double compute_base_score(const I & instance,
                          const bool parallel = false) noexcept {
    return compute_score(
        instance, melon::views::map([&](auto && case_id) {
            return instance.cases()[case_id].vertex_quality_map();
        }),
        melon::views::map([&](auto && case_id) {
            return instance.cases()[case_id].arc_probability_map();
        }),
        parallel);
}

template <instance_c I, melon::input_mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
double compute_solution_cost(const I & instance, const S & solution) noexcept {
    double sum = 0;
    for(auto && option : instance.options())
        sum += static_cast<double>(solution[option]) *
               instance.option_cost(option);
    return sum;
}

template <instance_c I, melon::input_mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
double compute_solution_score(const I & instance, const S & solution,
                              const auto & cases_vertex_options,
                              const auto & cases_arc_options,
                              const bool parallel = false) noexcept {
    return compute_score(
        instance, melon::views::map([&](auto && case_id) {
            auto enhanced_qm = instance.cases()[case_id].vertex_quality_map();
            auto && vertex_options = cases_vertex_options[case_id];
            for(auto && option : instance.options()) {
                if(!solution[option]) continue;
                for(auto && [u, quality_gain] : vertex_options[option])
                    enhanced_qm[u] += quality_gain;
            }
            return enhanced_qm;
        }),
        melon::views::map([&](auto && case_id) {
            auto enhanced_pm = instance.cases()[case_id].arc_probability_map();
            auto && arc_options = cases_arc_options[case_id];
            for(auto && option : instance.options()) {
                if(!solution[option]) continue;
                for(auto && [a, enhanced_prob] : arc_options[option]) {
                    enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
                }
            }
            return enhanced_pm;
        }),
        parallel);
}

template <instance_c I, melon::input_mapping<option_t> S>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
double compute_solution_score(const I & instance, const S & solution,
                              const bool parallel = false) noexcept {
    const auto cases_vertex_options = compute_cases_vertex_options(instance);
    const auto cases_arc_options = compute_cases_arc_options(instance);
    return compute_solution_score(instance, solution, cases_vertex_options,
                                  cases_arc_options, parallel);
}

template <instance_c I, detail::range_of<option_t> O>
void compute_options_cases_incr_eca(const I & instance, const O & free_options,
                                    auto && cases_current_qm,
                                    auto && cases_current_pm,
                                    auto && cases_vertex_options,
                                    auto && cases_arc_options,
                                    auto & options_cases_eca,
                                    const bool parallel = false) {
    const auto & cases = instance.cases();
    auto compute_options_enhanced_eca =
        [&](const tbb::blocked_range2d<decltype(cases.begin()),
                                       decltype(free_options.begin())> &
                cases_options_block) {
            for(const auto & instance_case : cases_options_block.rows()) {
                const auto & options_block = cases_options_block.cols();
                if(options_block.begin() == options_block.end()) continue;

                const auto & current_qm = cases_current_qm[instance_case.id()];
                const auto & current_pm = cases_current_pm[instance_case.id()];
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];

                auto enhanced_qm = current_qm;
                auto enhanced_pm = current_pm;

                for(auto it = options_block.begin();;) {
                    option_t option = *it;
                    for(auto && [u, quality_gain] : vertex_options[option])
                        enhanced_qm[u] += quality_gain;
                    for(auto && [a, enhanced_prob] : arc_options[option])
                        enhanced_pm[a] =
                            std::max(enhanced_pm[a], enhanced_prob);

                    options_cases_eca[option][instance_case.id()] =
                        eca(instance_case.graph(), enhanced_qm, enhanced_pm);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : vertex_options[option])
                        enhanced_qm[u] = current_qm[u];
                    for(auto && [a, enhanced_prob] : arc_options[option])
                        enhanced_pm[a] = current_pm[a];
                }
            }
        };

    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range2d(cases.begin(), cases.end(),
                                 free_options.begin(), free_options.end()),
            compute_options_enhanced_eca);
    } else {
        compute_options_enhanced_eca(
            tbb::blocked_range2d(cases.begin(), cases.end(),
                                 free_options.begin(), free_options.end()));
    }
}

template <instance_c I, melon::input_mapping<option_t> S,
          detail::range_of<option_t> O>
    requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
void compute_options_cases_decr_eca(
    const I & instance, const S & current_solution, const O & taken_options,
    auto && cases_current_qm, auto && cases_current_pm,
    auto && cases_vertex_options, auto && cases_arc_options,
    auto & options_cases_eca, const bool parallel = false) {
    const auto & cases = instance.cases();
    auto compute_options_enhanced_eca =
        [&](const tbb::blocked_range2d<decltype(cases.begin()),
                                       decltype(taken_options.begin())> &
                cases_options_block) {
            for(const auto & instance_case : cases_options_block.rows()) {
                const auto & options_block = cases_options_block.cols();
                if(options_block.begin() == options_block.end()) continue;

                const auto & current_qm = cases_current_qm[instance_case.id()];
                const auto & current_pm = cases_current_pm[instance_case.id()];
                const auto & original_pm = instance_case.arc_probability_map();
                const auto & vertex_options =
                    cases_vertex_options[instance_case.id()];
                const auto & arc_options =
                    cases_arc_options[instance_case.id()];

                auto enhanced_qm = current_qm;
                auto enhanced_pm = current_pm;

                for(auto it = options_block.begin();;) {
                    option_t option = *it;
                    for(auto && [u, quality_gain] : vertex_options[option])
                        enhanced_qm[u] -= quality_gain;
                    for(auto && [a, _] : arc_options[option]) {
                        enhanced_pm[a] = original_pm[a];
                        for(auto && [current_prob, i] :
                            instance_case.arc_options_map()[a]) {
                            if(current_solution[i] == 0.0 || option == i)
                                continue;
                            enhanced_pm[a] =
                                std::max(enhanced_pm[a], current_prob);
                        }
                    }

                    options_cases_eca[option][instance_case.id()] =
                        eca(instance_case.graph(), enhanced_qm, enhanced_pm);

                    if(++it == options_block.end()) break;

                    for(auto && [u, quality_gain] : vertex_options[option])
                        enhanced_qm[u] += quality_gain;
                    for(auto && [a, _] : arc_options[option])
                        enhanced_pm[a] = current_pm[a];
                }
            }
        };

    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range2d(cases.begin(), cases.end(),
                                 taken_options.begin(), taken_options.end()),
            compute_options_enhanced_eca);
    } else {
        compute_options_enhanced_eca(
            tbb::blocked_range2d(cases.begin(), cases.end(),
                                 taken_options.begin(), taken_options.end()));
    }
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_HELPER_HPP