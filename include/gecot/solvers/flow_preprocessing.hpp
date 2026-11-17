#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <tbb/concurrent_vector.h>

#include "mippp/model_concepts.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/log_table.hpp"

namespace gecot {
namespace solvers {

// What the algorithms that model the flow toward each target share before
// their model: the labels of the strong and useless arcs, the switches of the
// contraction of the graphs G_t, and the table of the cases.
struct flow_preprocessing : public mip_solver_base {
    double probability_resolution = 1e-8;
    int num_mus = 0;
    bool decades_sweep = true;
    // For comparison, a preprocessing step switched off: the contraction of
    // the strong arcs, the removal of the useless arcs, the skipping of the
    // vertices that can never be targets, and in the graph of a target the
    // removal of the arcs that leave it and of the vertices no quality
    // reaches.
    bool contract_strong_arcs = true;
    bool remove_useless_arcs = true;
    bool all_targets = false;
    bool pruning = true;
    // of the graphs G_t: plain for comparison, tight experimental. Not read
    // by tree_lp, whose pricing models take the plain ones.
    big_M_kind big_Ms = big_M_kind::knapsack_bnb;

    // a vertex that some options make a target
    template <case_c C>
    bool _is_target(const C & instance_case,
                    const melon::vertex_t<case_graph_t<C>> & t) const {
        return all_targets || instance_case.target_quality_map()[t] > 0 ||
               std::ranges::any_of(
                   instance_case.vertex_options_map()[t], [](auto && e) {
                       auto && [source_gain, target_gain, i] = e;
                       return target_gain > 0;
                   });
    }

    // Both kinds of arcs are labelled together: contracting or removing
    // alone saves no labelling time.
    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const budget_t & budget) const {
        using arcs = tbb::concurrent_vector<melon::arc_t<case_graph_t<C>>>;
        auto no_arcs = [&instance_case] {
            return melon::create_vertex_map<arcs>(instance_case.graph());
        };
        if(!contract_strong_arcs && !remove_useless_arcs)
            return std::make_pair(no_arcs(), no_arcs());
        auto affordable = [&instance, &budget](const option_t & i) {
            return fits_alone(instance, i, budget);
        };
        // on the cost dimension 0: dropping the others relaxes the problem
        auto labels = num_mus >= 1
                          ? compute_constrained_strong_and_useless_arcs(
                                instance, instance_case, budget[0], affordable,
                                probability_resolution, num_mus, decades_sweep,
                                all_targets)
                          : compute_strong_and_useless_arcs(
                                instance_case, affordable,
                                probability_resolution, all_targets);
        if(!contract_strong_arcs) labels.first = no_arcs();
        if(!remove_useless_arcs) labels.second = no_arcs();
        return labels;
    }

    // the arcs that a target of the case has in arcs_map, on average
    template <case_c C>
    double _average_size(const C & instance_case, const auto & arcs_map) const {
        std::size_t num_arcs = 0, num_targets = 0;
        for(auto && t : melon::vertices(instance_case.graph())) {
            if(!_is_target(instance_case, t)) continue;
            num_arcs += arcs_map[t].size();
            ++num_targets;
        }
        return static_cast<double>(num_arcs) /
               static_cast<double>(std::max<std::size_t>(num_targets, 1));
    }

    // of a case: its name, the _average_size of its strong and of its
    // useless arcs, then the values of the algorithm's own columns
    template <typename... V>
    using preprocessing_row = std::tuple<std::string, double, double, V...>;

    // Logs at -v the table of the cases' preprocessing, once they all are
    // preprocessed: a row logged after each case would be cut from the
    // others by the steps of the next one.
    template <typename... V, std::size_t... WIDTHS>
    static void _log_preprocessing(
        const std::vector<preprocessing_row<V...>> & cases,
        log_column<WIDTHS>... columns) {
        log_table<spdlog::level::debug, 0, 12, 11, 12, WIDTHS...> table(
            {"case"}, {"strong arcs per target", log_format::fixed(2)},
            {"useless arcs per target", log_format::fixed(2)},
            std::move(columns)...);
        for(const auto & values : cases)
            std::apply([&table](const auto &... v) { table.row(v...); },
                       values);
    }

    // the vertices and the arcs of the graphs G_t of a case
    struct graph_sizes {
        std::size_t num_graphs = 0, num_vertices = 0, num_arcs = 0;
        void add(const auto & graph) {
            ++num_graphs;
            num_vertices += graph.num_vertices();
            num_arcs += graph.num_arcs();
        }
    };
    // the row of a case for _log_graph_sizes
    template <case_c C>
    preprocessing_row<double, double> _sizes_row(
        const C & instance_case, const auto & strong_arcs_map,
        const auto & useless_arcs_map, const graph_sizes & sizes) const {
        const auto n =
            static_cast<double>(std::max<std::size_t>(sizes.num_graphs, 1));
        return {instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map),
                static_cast<double>(sizes.num_vertices) / n,
                static_cast<double>(sizes.num_arcs) / n};
    }
    // _log_preprocessing with the graphs G_t, on average
    static void _log_graph_sizes(
        const std::vector<preprocessing_row<double, double>> & cases) {
        _log_preprocessing(
            cases, log_column<10>{"vertices per target", log_format::fixed(2)},
            log_column<10>{"arcs per target", log_format::fixed(2)});
    }

    // the options_model of --big-m exact (compute_exact_big_M_map): a model
    // of M with the options and their rows, at the tolerances of the solver;
    // none for a solver without a dual bound
    template <typename M, instance_c I>
    auto _options_model(const I & instance, const budget_t & budget) const {
        if constexpr(mippp::has_dual_bound<M> && std::move_constructible<M>)
            return [this, &instance, &budget] {
                M model;
                _set_tolerances(model, mip_gap, feasibility_tol);
                const auto x = model.add_binary_variables(
                    instance.options().size(),
                    [](const option_t & i) { return i; });
                _add_budget_rows(model, instance, budget, x);
                return std::make_pair(std::move(model), x);
            };
        else
            return nullptr;
    }
};

}  // namespace solvers
}  // namespace gecot
