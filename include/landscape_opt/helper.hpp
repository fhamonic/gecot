#ifndef LANDSCAPE_OPT_HELPER_HPP
#define LANDSCAPE_OPT_HELPER_HPP

#include <optional>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "melon/container/static_map.hpp"
#include "melon/graph.hpp"

#include "landscape_opt/concepts/instance.hpp"
#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/indices/parallel_eca.hpp"

namespace fhamonic {
namespace landscape_opt {

template <case_c C>
auto compute_case_eca(const C & instance_case) noexcept {
    return eca(instance_case.graph(), instance_case.vertex_quality_map(),
               instance_case.arc_probability_map());
}

template <instance_c I>
auto compute_cases_vertex_options(const I & instance) noexcept {
    auto cases_vertex_options =
        instance.create_case_map(instance.create_option_map(
            std::vector<
                std::pair<melon::vertex_t<instance_graph_t<I>>, double>>{}));

    for(auto && instance_case : instance.cases()) {
        for(auto && u : melon::vertices(instance_case.graph())) {
            const auto & vertex_options_map =
                instance_case.vertex_options_map();
            for(auto && [quality_gain, option] : vertex_options_map[u]) {
                cases_vertex_options[instance_case.id()][option].emplace_back(
                    u, quality_gain);
            }
        }
    }
    return cases_vertex_options;
}

template <instance_c I>
auto compute_cases_arc_options(const I & instance) noexcept {
    auto cases_arc_options =
        instance.create_case_map(instance.create_option_map(
            std::vector<
                std::pair<melon::arc_t<instance_graph_t<I>>, double>>{}));

    for(auto && instance_case : instance.cases()) {
        for(auto && a : melon::arcs(instance_case.graph())) {
            const auto & arc_options_map = instance_case.arc_options_map()[a];
            for(auto && [enhanced_prob, option] : arc_options_map[a])
                cases_arc_options[instance_case.id()][option].emplace_back(
                    a, enhanced_prob);
        }
    }
    return cases_arc_options;
}

template <instance_c I, melon::input_value_map_of<option_t, bool> S>
double compute_solution_eca(const I & instance, const S & solution) {
    double sum = 0;
    const std::size_t nb_options = instance.nb_options();

    const auto cases_vertex_options = compute_cases_vertex_options(instance);
    const auto cases_arcs_options = compute_cases_arc_options(instance);

    for(auto && instance_case : instance.cases()) {
        auto && vertex_options = cases_vertex_options[instance_case.id()];
        auto && arc_options = cases_arcs_options[instance_case.id()];

        auto enhanced_qm = instance_case.vertex_quality_map();
        auto enhanced_pm = instance_case.arc_probability_map();

        for(auto && option : instance.options()) {
            if(!solution[option]) continue;
            for(auto && [u, quality_gain] : vertex_options[option])
                enhanced_qm[u] += quality_gain;
            for(auto && [a, enhanced_prob] : arc_options[option])
                enhanced_pm[a] = std::max(enhanced_pm[a], enhanced_prob);
        }

        sum += eca(instance_case.graph(), enhanced_qm, enhanced_pm);
    }
    return sum;
}

// typename Instance::Option compute_worst_option(instance, taken_options)

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_HELPER_HPP