#ifndef GECOT_COMPUTE_CONTRACTED_GRAPH_HPP
#define GECOT_COMPUTE_CONTRACTED_GRAPH_HPP

#include <algorithm>
#include <cstddef>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "mippp/linear_expression.hpp"

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/container/mutable_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/mapping.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_exact_big_M_map.hpp"
#include "gecot/preprocessing/compute_tight_big_M_maps.hpp"

#include "melon/container/static_digraph.hpp"
#include "melon/utility/make_static_digraph.hpp"

namespace gecot {

// The graph of a target t, contracted from its labels, before its vertices
// are renumbered: an improvement is an arc of its own.
struct contracted_flow_graph {
    using graph_t = melon::mutable_digraph;
    using vertex = melon::vertex_t<graph_t>;
    using arc = melon::arc_t<graph_t>;
    template <typename V>
    using vertex_map = melon::vertex_map_t<graph_t, V>;
    template <typename V>
    using arc_map = melon::arc_map_t<graph_t, V>;

    graph_t graph;
    vertex t;
    vertex_map<double> source_quality_map;
    vertex_map<std::vector<std::pair<double, option_t>>> source_options_map;
    // The rank of an arc among those of the graph: the contraction leaves
    // holes in the ids, so a variable per arc is not indexed by the id.
    arc_map<std::size_t> arc_no_map;
    // of an improvement: its probability with its option
    arc_map<double> probability_map;
    arc_map<std::optional<option_t>> improvement_option_map;
    // none when no big-M is asked for: the two accessors below then throw
    std::optional<vertex_map<double>> big_M_map;
    // with tight and exact big-Ms only, below big_M_map: the big-M of each
    // improvement, and per option the big-M of a target quality gain at t
    std::optional<std::pair<arc_map<double>, std::vector<double>>>
        lowered_big_Ms;

    // a must be an improvement: with lowered big-Ms, the other arcs have none
    double improvement_big_M(const arc & a) const {
        return lowered_big_Ms ? lowered_big_Ms->first[a]
                              : big_M_map.value()[melon::arc_source(graph, a)];
    }
    // of the target quality gain of the option i at t
    double target_big_M(const option_t i) const {
        return lowered_big_Ms ? lowered_big_Ms->second[i]
                              : big_M_map.value()[t];
    }
};

// by how much the tight or exact big-Ms lower the knapsack ones
struct lowered_big_M_statistics {
    double arc_sum = 0, arc_knapsack_sum = 0, target_sum = 0,
           target_knapsack_sum = 0;
    std::size_t num_arcs = 0, num_arcs_halved = 0, num_arcs_kept = 0;

    void add(const contracted_flow_graph & G_t, const auto & target_gains) {
        if(!G_t.lowered_big_Ms) return;
        const auto & knapsack_big_M_map = G_t.big_M_map.value();
        for(const auto & [source_gain, target_gain, i] : target_gains) {
            target_sum += G_t.target_big_M(i);
            target_knapsack_sum += knapsack_big_M_map[G_t.t];
        }
        for(auto && a : melon::arcs(G_t.graph)) {
            if(!G_t.improvement_option_map[a].has_value()) continue;
            const double knapsack_M =
                knapsack_big_M_map[melon::arc_source(G_t.graph, a)];
            arc_sum += G_t.improvement_big_M(a);
            arc_knapsack_sum += knapsack_M;
            ++num_arcs;
            num_arcs_halved += G_t.improvement_big_M(a) <= knapsack_M / 2;
            num_arcs_kept += G_t.improvement_big_M(a) >= knapsack_M;
        }
    }
    void log(const std::string_view kind) const {
        spdlog::debug(
            "flow_mip {} big-Ms / knapsack's: arcs {:.4g}, targets {:.4g}",
            kind, arc_knapsack_sum > 0 ? arc_sum / arc_knapsack_sum : 1.0,
            target_knapsack_sum > 0 ? target_sum / target_knapsack_sum : 1.0);
        spdlog::debug("Arcs: {}, halved at least: {}, not lowered: {}",
                      num_arcs, num_arcs_halved, num_arcs_kept);
    }
};

// big_Ms: the big-Ms to compute, none without a value; without ArcBigMs,
// those of t only. options_model: of compute_exact_big_M_map, for exact.
template <bool ArcBigMs = true, instance_c I, case_c C,
          typename OM = std::nullptr_t>
contracted_flow_graph compute_contracted_flow_graph(
    const I & instance, const C & instance_case, const budget_t & budget,
    const auto & original_t, const auto & strong_arcs,
    const auto & useless_arcs, const std::optional<big_M_kind> big_Ms,
    const double tolerance, const bool pruning,
    const OM & options_model = nullptr) {
    auto [graph, source_quality_map, source_options_map, arc_no_map,
          probability_map, improvement_option_map, t] =
        compute_contracted_generalized_flow_graph(
            instance_case, strong_arcs, useless_arcs, original_t, pruning);

    auto is_tail = [&](auto && u) {
        return std::ranges::any_of(melon::out_arcs(graph, u), [&](auto && a) {
            return improvement_option_map[a].has_value();
        });
    };
    // t, and the tails of the improvements
    auto tails_and_t =
        std::views::filter(melon::vertices(graph),
                           [&](auto && u) { return u == t || is_tail(u); });
    auto big_M_vertices = [&] {
        if constexpr(ArcBigMs)
            return tails_and_t;
        else
            return std::views::single(t);
    }();
    auto compute_map = [&](auto && vertices) {
        return big_Ms == big_M_kind::plain
                   ? compute_big_M_map(graph, source_quality_map,
                                       source_options_map, probability_map,
                                       vertices)
                   : compute_knapsack_big_M_map(
                         instance, budget,
                         big_Ms == big_M_kind::knapsack_dantzig, graph,
                         source_quality_map, source_options_map,
                         probability_map, vertices);
    };
    std::optional<contracted_flow_graph::vertex_map<double>> big_M_map;
    // exact's MIPs bound their copies by the constants of their tails: it
    // needs those of every tail
    if(big_Ms == big_M_kind::exact)
        big_M_map = compute_map(tails_and_t);
    else if(big_Ms)
        big_M_map = compute_map(big_M_vertices);
    const auto target_options =
        std::views::transform(instance_case.vertex_options_map()[original_t],
                              [](auto && e) { return std::get<2>(e); });
    std::optional<
        std::pair<contracted_flow_graph::arc_map<double>, std::vector<double>>>
        lowered_big_Ms;
    if(big_Ms == big_M_kind::tight) {
        lowered_big_Ms = compute_tight_big_M_maps<ArcBigMs>(
            instance, budget[0], tolerance, graph, source_quality_map,
            source_options_map, probability_map, improvement_option_map, t,
            target_options);
        auto & [arc_big_M_map, target_big_M_map] = lowered_big_Ms.value();
        if constexpr(ArcBigMs)
            for(const auto & a : melon::arcs(graph))
                if(improvement_option_map[a].has_value())
                    arc_big_M_map[a] = std::min(
                        arc_big_M_map[a],
                        big_M_map.value()[melon::arc_source(graph, a)]);
        // those of the other options are read nowhere
        std::vector<double> lowered(target_big_M_map.size(),
                                    big_M_map.value()[t]);
        for(const option_t i : target_options)
            lowered[i] = std::min(lowered[i], target_big_M_map[i]);
        target_big_M_map = std::move(lowered);
    }
    if(big_Ms == big_M_kind::exact) {
        if constexpr(std::is_null_pointer_v<OM>)
            throw std::invalid_argument(
                "--big-m exact needs a MIP solver with a dual bound");
        else {
            // the constants read: of t only for its options or as a tail
            const auto exact_big_M_map = compute_exact_big_M_map(
                options_model, graph, source_quality_map, source_options_map,
                probability_map, improvement_option_map, arc_no_map,
                big_M_map.value(),
                std::views::filter(big_M_vertices, [&](auto && u) {
                    return u != t || !std::ranges::empty(target_options) ||
                           is_tail(t);
                }));
            // the layer of tight: the copies of the arcs that leave u and
            // the target quality gains read exact_big_M_map[u]
            auto & [arc_big_M_map, target_big_M_map] = lowered_big_Ms.emplace(
                melon::create_arc_map<double>(graph, 0.0),
                std::vector<double>(std::ranges::size(instance.options()),
                                    exact_big_M_map[t]));
            if constexpr(ArcBigMs)
                for(const auto & a : melon::arcs(graph))
                    if(improvement_option_map[a].has_value())
                        arc_big_M_map[a] =
                            exact_big_M_map[melon::arc_source(graph, a)];
        }
    }
    return {.graph = std::move(graph),
            .t = t,
            .source_quality_map = std::move(source_quality_map),
            .source_options_map = std::move(source_options_map),
            .arc_no_map = std::move(arc_no_map),
            .probability_map = std::move(probability_map),
            .improvement_option_map = std::move(improvement_option_map),
            .big_M_map = std::move(big_M_map),
            .lowered_big_Ms = std::move(lowered_big_Ms)};
}

// The graph of G with its vertices renumbered, as the Benders decompositions
// read it: make_static_digraph renumbers them in vertex_cmp's order. Returns
// t in the static graph, the graph, its source qualities and options, the
// probabilities and the options of its arcs, then each of arc_maps
// renumbered.
auto static_flow_graph(const contracted_flow_graph & G,
                       const auto &... arc_maps) {
    const auto vertex_cmp = [](auto && u, auto && v) { return u < v; };
    return std::tuple_cat(
        std::make_tuple(static_cast<melon::vertex_t<melon::static_digraph>>(
            std::ranges::count_if(
                melon::vertices(G.graph),
                [&](auto && u) { return vertex_cmp(u, G.t); }))),
        melon::make_static_digraph(
            G.graph, vertex_cmp,
            std::tie(G.source_quality_map, G.source_options_map),
            std::tie(G.probability_map, G.improvement_option_map,
                     arc_maps...)));
}

// a + b, the terms of a first. mippp joins the terms of a sum with
// unordered_concat, which under GCC 15.1 puts the right operand first where
// views::concat rejects the written order: this then does not compile.
auto ordered_sum(auto && a, auto && b) {
    using namespace mippp::operators;
#if defined(__cpp_lib_ranges_concat) && !defined(MIPPP_PORTABLE_RANGE_SHAPES)
    static_assert(std::ranges::range<decltype(std::views::concat(
                      std::forward<decltype(a)>(a).linear_terms(),
                      std::forward<decltype(b)>(b).linear_terms()))>,
                  "the terms of the sum reach the solver in the written order");
#endif
    return std::forward<decltype(a)>(a) + std::forward<decltype(b)>(b);
}

// The graph of a target as static_flow_graph gives it, with the big-Ms that
// flow_benders and multicut_benders read.
template <case_c C>
struct contracted_graph_data {
    using original_vertex = melon::vertex_t<case_graph_t<C>>;
    using graph_t = melon::static_digraph;
    using vertex = melon::vertex_t<graph_t>;
    using arc = melon::arc_t<graph_t>;
    template <typename V>
    using vertex_map = melon::vertex_map_t<graph_t, V>;
    template <typename V>
    using arc_map = melon::arc_map_t<graph_t, V>;

    std::reference_wrapper<const C> instance_case;
    original_vertex original_t;
    vertex t;
    graph_t graph;
    vertex_map<double> source_quality_map;
    vertex_map<std::vector<std::pair<double, option_t>>> source_options_map;
    // of an improvement: its probability with its option
    arc_map<double> probability_map;
    arc_map<std::optional<option_t>> option_map;
    // of an improvement, 0 for the other arcs
    arc_map<double> improvement_big_M_map;
    // of t, for the target quality gain of every option; with tight and
    // exact big-Ms, lowered per option in target_big_M_map
    double t_big_M;
    std::vector<double> target_big_M_map;

    double target_big_M(const option_t i) const {
        return target_big_M_map.empty() ? t_big_M : target_big_M_map[i];
    }

    // f_t(y) and the potentials rho_u = Pi^y(u, t) of the cut of t at the
    // scenario y (benders.md, "The cut of a target"), by one run of
    // Dijkstra's algorithm from t on the reversed graph
    auto flow_and_potentials(const auto & y) const {
        double flow = 0.0;
        auto rho = melon::create_vertex_map<double>(graph, 0.0);
        const auto & reversed_graph = melon::views::reverse(graph);
        for(const auto & [u, probability] : melon::dijkstra(
                detail::pc_num_dijkstra_traits<decltype(reversed_graph),
                                               double>{},
                reversed_graph,
                [&](auto && a) {
                    return option_map[a].has_value() &&
                                   !y[option_map[a].value()]
                               ? 0.0
                               : probability_map[a];
                },
                t)) {
            double quality = source_quality_map[u];
            for(auto && [source_gain, i] : source_options_map[u])
                if(y[i]) quality += source_gain;
            flow += quality * probability;
            rho[u] = probability;
        }
        return std::make_pair(flow, std::move(rho));
    }

    // cut_{t,y}(x) of benders.md, "The cut of a target": x(i) is the variable
    // of the option i, and rho the potentials of y that flow_and_potentials
    // returns, or these times a factor. A lazy sum that reads this graph, y
    // and rho: only valid while they live.
    auto cut(const auto & x, const auto & y, const auto & rho) const {
        using namespace mippp::operators;
        return ordered_sum(
            xsum(graph.vertices(),
                 [&](auto && u) {
                     return (source_quality_map[u] +
                             xsum(source_options_map[u],
                                  [&](auto && gain) {
                                      auto && [source_gain, i] = gain;
                                      return source_gain * x(i);
                                  })) *
                            rho[u];
                 }),
            xsum(std::views::filter(
                     graph.arcs(),
                     [&](auto && a) { return option_map[a].has_value(); }),
                 [&](auto && a) {
                     const option_t i = option_map[a].value();
                     const auto u = graph.arc_source(a);
                     const auto v = graph.arc_target(a);
                     const double sigma =
                         y[i] ? 0.0
                              : std::max(0.0,
                                         probability_map[a] * rho[v] - rho[u]);
                     return improvement_big_M_map[a] * sigma * x(i);
                 }));
    }
};

// The graph of a target for flow_benders and multicut_benders:
// compute_contracted_flow_graph with its vertices renumbered.
template <instance_c I, case_c C, typename OM = std::nullptr_t>
contracted_graph_data<C> compute_contracted_graph_data(
    const I & instance, const C & instance_case, const budget_t & budget,
    const auto & original_t, const auto & strong_arcs,
    const auto & useless_arcs, const big_M_kind big_Ms, const double tolerance,
    const bool pruning, const OM & options_model = nullptr) {
    contracted_flow_graph G = compute_contracted_flow_graph(
        instance, instance_case, budget, original_t, strong_arcs, useless_arcs,
        big_Ms, tolerance, pruning, options_model);

    ////////////////////////////////////////////////////////////////////////////
    auto [t, graph, source_quality_map, source_options_map, probability_map,
          option_map, improvement_big_M_map] =
        static_flow_graph(
            G,
            melon::maps::function([&G](const contracted_flow_graph::arc & a) {
                return G.improvement_option_map[a].has_value()
                           ? G.improvement_big_M(a)
                           : 0.0;
            }));
    return contracted_graph_data<C>{
        .instance_case = instance_case,
        .original_t = original_t,
        .t = t,
        .graph = std::move(graph),
        .source_quality_map = std::move(source_quality_map),
        .source_options_map = std::move(source_options_map),
        .probability_map = std::move(probability_map),
        .option_map = std::move(option_map),
        .improvement_big_M_map = std::move(improvement_big_M_map),
        .t_big_M = G.big_M_map.value()[G.t],
        .target_big_M_map = G.lowered_big_Ms
                                ? std::move(G.lowered_big_Ms->second)
                                : std::vector<double>{}};
    ////////////////////////////////////////////////////////////////////////////

    // melon::mutable_digraph graph;
    // // create new vertices
    // auto original_to_new_vertex_map =
    //     melon::create_vertex_map<melon::vertex_t<melon::mutable_digraph>>(
    //         original_graph);
    // for(const auto & v : melon::vertices(original_graph)) {
    //     original_to_new_vertex_map[v] = graph.create_vertex();
    // }
    // auto source_quality_map = melon::create_vertex_map<double>(graph);
    // auto source_options_map =
    //     melon::create_vertex_map<std::vector<std::pair<double, option_t>>>(
    //         graph);
    // for(const auto & v : melon::vertices(original_graph)) {
    //     auto new_v = original_to_new_vertex_map[v];
    //     source_quality_map[new_v] = original_source_quality_map[v];
    //     for(auto && [source_quality_gain, tqm, option] :
    //         original_vertex_options_map[v]) {
    //         if(source_quality_gain == 0) continue;
    //         source_options_map[new_v].emplace_back(source_quality_gain,
    //         option);
    //     }
    // }

    // auto original_to_new_arc_map = melon::create_arc_map<
    //     std::optional<melon::arc_t<melon::mutable_digraph>>>(original_graph);
    // std::vector<std::tuple<melon::arc_t<melon::mutable_digraph>, double,
    //                        std::optional<option_t>>>
    //     added_arcs;

    // // transform the useless arcs map
    // auto arc_uselessness_map =
    //     melon::create_arc_map<bool>(original_graph, false);
    // // for(const auto & a : useless_arcs) arc_uselessness_map[a] = true;
    // // for(const auto & a : strong_arcs) arc_uselessness_map[a] = false;
    // // // add uselessness of outgoing arcs of t
    // // for(const auto & a : melon::out_arcs(original_graph, original_t))
    // //     arc_uselessness_map[a] = true;

    // // create new arcs
    // for(const auto & a : melon::arcs(original_graph)) {
    //     if(arc_uselessness_map[a]) continue;  // filter the useless arcs
    //     auto new_source =
    //         original_to_new_vertex_map[melon::arc_source(original_graph, a)];
    //     auto new_target =
    //         original_to_new_vertex_map[melon::arc_target(original_graph, a)];
    //     auto new_arc = graph.create_arc(new_source, new_target);
    //     original_to_new_arc_map[a].emplace(new_arc);

    //     added_arcs.emplace_back(new_arc, original_probability_map[a],
    //                             std::nullopt);
    //     for(auto && [improved_probability, option] :
    //     original_arc_options_map[a]) {
    //         added_arcs.emplace_back(new_arc, improved_probability,
    //                                 std::make_optional(option));
    //     }
    // }
    // auto probability_map = melon::create_arc_map<double>(graph);
    // auto arc_options_map =
    //     melon::create_arc_map<std::vector<std::pair<double,
    //     option_t>>>(graph,
    //                                                                     {});
    // for(auto && [a, prob, option] : added_arcs) {
    //     if(!option.has_value()) {
    //         probability_map[a] = prob;
    //         continue;
    //     }
    //     arc_options_map[a].emplace_back(prob, option.value());
    // }

    // // contract strong arcs
    // /*
    // std::vector<melon::arc_t<melon::mutable_digraph>> in_arcs_tmp;
    // for(const auto & original_uv : strong_arcs) {
    //     if(!original_to_new_arc_map[original_uv].has_value())
    //         continue;  // should not be needed (no useless arcs are strong)
    //     const auto uv = original_to_new_arc_map[original_uv].value();
    //     if(!graph.is_valid_arc(uv))
    //         continue;  // if many strong arcs shared the same source
    //     if(arc_options_map[uv].size() > 1) continue;

    //     const auto u = melon::arc_source(graph, uv);
    //     const auto v = melon::arc_target(graph, uv);
    //     const auto uv_prob = probability_map[uv];
    //     in_arcs_tmp.resize(0);
    //     std::ranges::copy(graph.in_arcs(u), std::back_inserter(in_arcs_tmp));

    //     if(arc_options_map[uv].empty()) {  // if uv is not improvable
    //         for(const auto & wu : in_arcs_tmp) {
    //             probability_map[wu] *= uv_prob;
    //             for(auto & [improved_probability, option] :
    //             arc_options_map[wu])
    //                 improved_probability *= uv_prob;
    //             graph.change_arc_target(wu, v);
    //         }
    //         source_quality_map[v] += uv_prob * source_quality_map[u];
    //         for(const auto & [source_quality_gain, option] :
    //             source_options_map[u])
    //             source_options_map[v].emplace_back(
    //                 uv_prob * source_quality_gain, option);
    //         graph.remove_vertex(u);
    //         continue;
    //     }
    //     const auto & [uv_enhanced_prob, uv_option] =
    //         arc_options_map[uv].front();
    //     if(std::ranges::all_of(in_arcs_tmp,
    //                            [&](auto && wu) {
    //                                for(const auto & [improved_probability,
    //                                wu_option] :
    //                                    arc_options_map[wu]) {
    //                                    if(wu_option != uv_option) return
    //                                    false;
    //                                }
    //                                return true;
    //                            }) &&
    //        std::ranges::all_of(source_options_map[u], [&](auto && p) {
    //            const auto & [source_quality_gain, option] = p;
    //            return (option == uv_option);
    //        })) {  // if all elements subjects to contraction are subject to
    //        the
    //               // same option
    //         for(const auto & wu : in_arcs_tmp) {
    //             if(arc_options_map[wu].empty()) {
    //                 arc_options_map[wu].emplace_back(
    //                     probability_map[wu] * uv_enhanced_prob, uv_option);
    //             } else {
    //                 for(auto & [wu_enhanced_prob, option] :
    //                 arc_options_map[wu])
    //                     wu_enhanced_prob *= uv_enhanced_prob;
    //             }
    //             probability_map[wu] *= uv_prob;
    //             graph.change_arc_target(wu, v);
    //         }
    //         source_quality_map[v] += uv_prob * source_quality_map[u];
    //         source_options_map[v].emplace_back(
    //             (uv_enhanced_prob - uv_prob) * source_quality_map[u],
    //             uv_option);
    //         for(const auto & [source_quality_gain, option] :
    //             source_options_map[u])
    //             source_options_map[v].emplace_back(
    //                 uv_enhanced_prob * source_quality_gain, option);
    //         graph.remove_vertex(u);
    //     }
    // }
    // //*/

    // // remove vertices that cannot be traversed by flow
    // // auto bfs = melon::breadth_first_search(graph);
    // // for(const auto & v : melon::vertices(graph)) {
    // //     if(source_quality_map[v] == 0 && source_options_map[v].empty())
    // //         continue;
    // //     if(bfs.reached(v)) continue;
    // //     bfs.add_source(v).run();
    // // }
    // // std::vector<melon::vertex_t<melon::mutable_digraph>>
    // // vertices_to_delete_tmp; std::ranges::copy(std::ranges::views::filter(
    // //                       melon::vertices(graph),
    // //                       [&bfs](const auto & v) { return !bfs.reached(v);
    // //                       }),
    // //                   std::back_inserter(vertices_to_delete_tmp));
    // // for(const auto & v : vertices_to_delete_tmp) {
    // //     graph.remove_vertex(v);
    // // }

    // auto t = original_to_new_vertex_map[original_t];

    // auto improved_probability_map = probability_map;
    // for(const auto & a : melon::arcs(graph)) {
    //     for(const auto & [improved_probability, option] : arc_options_map[a])
    //         improved_probability_map[a] =
    //             std::max(improved_probability_map[a], improved_probability);
    // }
    // auto big_M_map = compute_big_M_map(
    //     graph, source_quality_map, source_options_map,
    //     improved_probability_map, std::views::filter(melon::vertices(graph),
    //     [&](auto && u) {
    //         return u == t || std::ranges::any_of(
    //                              melon::out_arcs(graph, u), [&](auto && a) {
    //                                  return arc_options_map[a].size() > 0;
    //                              });
    //     }));

    // //////////////////////////////////////////////////////////////////////

    // // return contracted_graph_data(
    // //     instance_case, original_t, t,
    // //     std::move(graph), std::move(source_quality_map),
    // //     std::move(source_options_map), std::move(probability_map),
    // //     std::move(arc_options_map), std::move(big_M_map));

    // auto reverse_graph = melon::views::reverse(graph);
    // int order = 0;
    // auto order_map = melon::create_vertex_map<int>(graph);
    // for(const auto & [u, prob] : melon::dijkstra(
    //         detail::pc_num_dijkstra_traits<decltype(reverse_graph),
    //         double>{}, reverse_graph, probability_map, t)) {
    //     order_map[u] = order++;
    // }
    // auto [sgraph, svertex_maps, sarc_maps] = melon::make_static_digraph(
    //     graph,
    //     [&order_map](auto && u, auto && v) {
    //         return order_map[u] < order_map[v];
    //     },
    //     std::make_tuple(source_quality_map, source_options_map, big_M_map),
    //     std::make_tuple(probability_map, arc_options_map));

    // auto && [ssource_quality_map, ssource_options_map, sbig_M_map] =
    //     svertex_maps;
    // auto && [sprobability_map, sarc_options_map] = sarc_maps;

    // return contracted_graph_data(
    //     instance_case, original_t, melon::vertex_t<melon::static_digraph>{0},
    //     std::move(sgraph), std::move(ssource_quality_map),
    //     std::move(ssource_options_map), std::move(sprobability_map),
    //     std::move(sarc_options_map), std::move(sbig_M_map));
}

}  // namespace gecot

#endif  // GECOT_COMPUTE_CONTRACTED_GRAPH_HPP