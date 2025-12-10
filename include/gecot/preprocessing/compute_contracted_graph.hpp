#ifndef GECOT_COMPUTE_CONTRACTED_GRAPH_HPP
#define GECOT_COMPUTE_CONTRACTED_GRAPH_HPP

#include <algorithm>
#include <optional>
#include <ranges>
#include <tuple>

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/container/mutable_digraph.hpp"
#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"

#include "melon/container/static_digraph.hpp"
#include "melon/container/static_map.hpp"

namespace fhamonic {

namespace melon {
template <typename G, typename CMP, typename... VM, typename... AM>
auto make_static_digraph(G && graph, CMP vertex_cmp,
                         std::tuple<VM...> vertex_maps_tuple,
                         std::tuple<AM...> arc_maps_tuple) {
    using SG = static_digraph;
    std::vector<vertex_t<G>> new_to_old_vertex;
    std::ranges::copy(graph.vertices(), std::back_inserter(new_to_old_vertex));
    std::sort(new_to_old_vertex.begin(), new_to_old_vertex.end(), vertex_cmp);
    auto old_to_new_vertex = melon::create_vertex_map<vertex_t<SG>>(graph);
    const auto n = new_to_old_vertex.size();
    for(auto && v :
        std::views::iota(vertex_t<SG>{0}, static_cast<vertex_t<SG>>(n))) {
        old_to_new_vertex[new_to_old_vertex[v]] = v;
    }
    std::vector<vertex_t<SG>> sources;
    std::vector<vertex_t<SG>> targets;
    if constexpr(has_num_arcs<G>) {
        const auto m = num_arcs(graph);
        sources.reserve(m);
        targets.reserve(m);
    }
    std::vector<arc_t<G>> new_to_old_arc;
    for(auto && v :
        std::views::iota(vertex_t<SG>{0}, static_cast<vertex_t<SG>>(n))) {
        for(auto && old_a : melon::out_arcs(
                graph, new_to_old_vertex[static_cast<std::size_t>(v)])) {
            sources.emplace_back(v);
            targets.emplace_back(
                old_to_new_vertex[melon::arc_target(graph, old_a)]);
            new_to_old_arc.emplace_back(old_a);
        }
    }
    const auto m = new_to_old_arc.size();
    return std::make_tuple(
        SG(n, sources, targets),
        std::apply(
            [&](VM const &... old_vertex_map) {
                auto translate_vertex_map = [&](auto && ovm) {
                    static_map<vertex_t<SG>,
                               mapped_value_t<decltype(ovm), vertex_t<G>>>
                        vertex_map(n);
                    for(auto && v : std::views::iota(
                            vertex_t<SG>{0}, static_cast<vertex_t<SG>>(n))) {
                        vertex_map[v] = ovm[new_to_old_vertex[v]];
                    }
                    return vertex_map;
                };
                return std::make_tuple(translate_vertex_map(old_vertex_map)...);
            },
            vertex_maps_tuple),
        std::apply(
            [&](AM const &... old_arc_map) {
                auto translate_vertex_map = [&](auto && oam) {
                    static_map<arc_t<SG>,
                               mapped_value_t<decltype(oam), arc_t<G>>>
                        arc_map(m);
                    for(auto && a : std::views::iota(
                            arc_t<SG>{0}, static_cast<arc_t<SG>>(m))) {
                        arc_map[a] = oam[new_to_old_arc[a]];
                    }
                    return arc_map;
                };
                return std::make_tuple(translate_vertex_map(old_arc_map)...);
            },
            arc_maps_tuple));
}
}  // namespace melon

namespace gecot {

template <case_c C>
struct contracted_graph_data {
public:
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
    arc_map<double> probability_map;
    arc_map<std::vector<std::pair<double, option_t>>> arc_options_map;
    vertex_map<double> big_M_map;

    contracted_graph_data() = default;
    contracted_graph_data(contracted_graph_data &&) = default;
    contracted_graph_data(const contracted_graph_data &) = default;

    contracted_graph_data & operator=(contracted_graph_data &&) = default;
    contracted_graph_data & operator=(const contracted_graph_data &) = default;

    contracted_graph_data(
        const C & instance_case_, original_vertex original_t_, vertex t_,
        graph_t && contracted_graph_, vertex_map<double> && source_quality_map_,
        vertex_map<std::vector<std::pair<double, option_t>>> &&
            source_options_map_,
        arc_map<double> && probability_map_,
        arc_map<std::vector<std::pair<double, option_t>>> && arc_options_map_,
        vertex_map<double> && big_M_map_)
        : instance_case(instance_case_)
        , original_t(original_t_)
        , t(t_)
        , graph(std::move(contracted_graph_))
        , source_quality_map(std::move(source_quality_map_))
        , source_options_map(std::move(source_options_map_))
        , probability_map(std::move(probability_map_))
        , arc_options_map(std::move(arc_options_map_))
        , big_M_map(std::move(big_M_map_)) {}
};

template <instance_c I, case_c C>
contracted_graph_data<C> compute_contracted_graph_data(
    const I & instance, const C & instance_case, const double budget,
    const auto & strong_arcs, const auto & useless_arcs,
    const auto & original_t) {
    const auto & original_graph = instance_case.graph();
    const auto & original_source_quality_map =
        instance_case.source_quality_map();
    const auto & original_vertex_options_map =
        instance_case.vertex_options_map();
    const auto & original_probability_map = instance_case.arc_probability_map();
    const auto & original_arc_options_map = instance_case.arc_options_map();

    ////////////////////////////////////////////////////////////////////////////
    auto original_source_options_map =
        melon::create_vertex_map<std::vector<std::pair<double, option_t>>>(
            original_graph);
    for(const auto & u : melon::vertices(original_graph)) {
        for(auto && [source_quality_gain, tqg, option] :
            original_vertex_options_map[u]) {
            original_source_options_map[u].emplace_back(source_quality_gain,
                                                        option);
        }
    }
    auto improved_probability_map = original_probability_map;
    for(const auto & a : melon::arcs(original_graph)) {
        for(const auto & [improved_prob, option] : original_arc_options_map[a])
            improved_probability_map[a] =
                std::max(improved_probability_map[a], improved_prob);
    }
    // auto big_M_map = compute_big_M_map(
    //     original_graph, original_source_quality_map,
    //     original_source_options_map, improved_probability_map,
    //     std::views::filter(melon::vertices(original_graph), [&](auto && u) {
    //         return u == original_t ||
    //                std::ranges::any_of(
    //                    melon::out_arcs(original_graph, u), [&](auto && a) {
    //                        return original_arc_options_map[a].size() > 0;
    //                    });
    //     }));
    const auto big_M_map = compute_knapsack_big_M_map(
        instance, budget, original_graph, original_source_quality_map,
        original_source_options_map, improved_probability_map,
        std::views::filter(melon::vertices(original_graph), [&](auto && u) {
            return u == original_t ||
                   std::ranges::any_of(
                       melon::out_arcs(original_graph, u), [&](auto && a) {
                           return original_arc_options_map[a].size() > 0;
                       });
        }));

    auto [sgraph, svertex_maps, sarc_maps] = melon::make_static_digraph(
        original_graph, [](auto && u, auto && v) { return u < v; },
        std::make_tuple(original_source_quality_map,
                        original_source_options_map, big_M_map),
        std::make_tuple(original_probability_map, original_arc_options_map));
    auto && [ssource_quality_map, ssource_options_map, sbig_M_map] =
        svertex_maps;
    auto && [sprobability_map, sarc_options_map] = sarc_maps;

    return contracted_graph_data(
        instance_case, original_t, original_t, std::move(sgraph),
        std::move(ssource_quality_map), std::move(ssource_options_map),
        std::move(sprobability_map), std::move(sarc_options_map),
        std::move(sbig_M_map));
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
    //     for(auto && [enhanced_prob, option] : original_arc_options_map[a]) {
    //         added_arcs.emplace_back(new_arc, enhanced_prob,
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
    //             for(auto & [enhanced_prob, option] : arc_options_map[wu])
    //                 enhanced_prob *= uv_prob;
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
    //                                for(const auto & [enhanced_prob,
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
    //     for(const auto & [improved_prob, option] : arc_options_map[a])
    //         improved_probability_map[a] =
    //             std::max(improved_probability_map[a], improved_prob);
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
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_CONTRACTED_GRAPH_HPP