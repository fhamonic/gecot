#ifndef GECOT_COMPUTE_EXACT_BIG_M_MAP_HPP
#define GECOT_COMPUTE_EXACT_BIG_M_MAP_HPP

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "mippp/model_concepts.hpp"

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/graph.hpp"
#include "melon/views/reverse.hpp"
#include "melon/views/subgraph.hpp"

#include "gecot/flow_rows.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {

// exact: for each vertex u of vertices, the largest flow that can reach u
// within the budgets and the conflict sets, by a MIP on the vertices that
// reach u: flow.md's rows with u as the sink, the improved copies within
// big_M_map. This is what knapsack_bnb bounds, not the least valid constant
// of a copy, which can carry less than what reaches its tail. At most
// big_M_map[u]; big_M_map[u] again for the vertices left after a stop.
// options_model() returns a model and the variables x of the options, with
// the rows of the budgets and the conflict sets.
template <typename GR>
auto compute_exact_big_M_map(const auto & options_model, const GR & graph,
                             const auto & source_quality_map,
                             const auto & source_options_map,
                             const auto & probability_map,
                             const auto & improvement_option_map,
                             const auto & arc_no_map, const auto & big_M_map,
                             auto && vertices) {
    using namespace mippp::operators;
    using vertex = melon::vertex_t<GR>;
    using model_t = typename std::invoke_result_t<
        const std::remove_cvref_t<decltype(options_model)> &>::first_type;
    auto exact_big_M_map = big_M_map;
    std::vector<vertex> vertices_vector;
    std::ranges::copy(vertices, std::back_inserter(vertices_vector));
    tbb::parallel_for(
        tbb::blocked_range(vertices_vector.begin(), vertices_vector.end()),
        [&](auto && block) {
            // the search of the vertices that reach u and the subgraph they
            // induce, for every u of the block
            const auto reversed_graph = melon::views::reverse(graph);
            auto search = melon::breadth_first_search(reversed_graph);
            const auto reach = melon::views::subgraph(
                graph,
                [&search](const vertex & v) { return search.reached(v); });
            for(const vertex & u : block) {
                if(stop_requested) return;
                search.reset().add_source(u);
                while(!search.finished()) search.advance();
                auto [model, x] = options_model();
                // several are solved at once
                if constexpr(mippp::has_thread_limit<model_t>)
                    if(vertices_vector.size() > 1) model.set_thread_limit(1);
                const auto f_u = model.add_variable();
                model.set_maximization();
                model.set_objective(f_u);
                const auto phi = model.add_variables(
                    graph.num_arcs(),
                    [&arc_no_map](const auto & a) { return arc_no_map[a]; });
                add_flow_rows(model, reach, u, f_u, phi, x, probability_map,
                              source_quality_map, source_options_map);
                for(auto && a : melon::arcs(reach))
                    if(improvement_option_map[a].has_value())
                        model.add_constraint(
                            phi(a) <=
                            x(improvement_option_map[a].value()) *
                                big_M_map[melon::arc_source(graph, a)]);
                model.solve();
                // the dual bound, up to the solver's tolerances
                if(mippp::status::solution_available(model.get_status()))
                    exact_big_M_map[u] =
                        std::min(exact_big_M_map[u],
                                 model.get_dual_bound() * (1 + 1e-6));
            }
        });
    return exact_big_M_map;
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_EXACT_BIG_M_MAP_HPP
