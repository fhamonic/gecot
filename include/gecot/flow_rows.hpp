#ifndef GECOT_FLOW_ROWS_HPP
#define GECOT_FLOW_ROWS_HPP

#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"

#include "melon/graph.hpp"

namespace gecot {

// flow.md's conservation of the flow toward sink in graph: what leaves a
// vertex u on its out-arcs a, phi(a), and at sink f_sink too, is at most what
// reaches u. x(i) is the variable of the option i.
void add_flow_rows(auto & model, const auto & graph, const auto & sink,
                   const auto & f_sink, const auto & phi, const auto & x,
                   const auto & probability_map,
                   const auto & source_quality_map,
                   const auto & source_options_map) {
    using namespace mippp::operators;
    auto inflow = [&](auto && u) {
        return xsum(melon::in_arcs(graph, u),
                    [&](auto && a) { return probability_map[a] * phi(a); }) +
               source_quality_map[u] +
               xsum(source_options_map[u],
                    [&x](auto && gain) { return gain.first * x(gain.second); });
    };
    model.add_constraints(
        melon::vertices(graph),
        [&](auto && u) {
            return (u == sink)
                       ? std::make_optional(
                             f_sink + xsum(melon::out_arcs(graph, u), phi) <=
                             inflow(u))
                       : std::nullopt;
        },
        [&](auto && u) {
            return xsum(melon::out_arcs(graph, u), phi) <= inflow(u);
        });
}

}  // namespace gecot

#endif  // GECOT_FLOW_ROWS_HPP
