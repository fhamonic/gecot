#ifndef GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/concurrent_dijkstras.hpp"
#include "melon/views/subgraph.hpp"

#include "gecot/helper.hpp"

// namespace fhamonic {
// namespace gecot {
// namespace detail {
// template <melon::outward_incidence_graph _Graph, typename _ValueType>
// struct strong_arc_default_traits {
//     using semiring = melon::most_reliable_path_semiring<_ValueType>;
//     using entry = std::pair<_ValueType, bool>;
//     static bool compare_entries(const entry & e1, const entry & e2) {
//         if(e1.first == e2.first) {
//             return e1.second && !e2.second;
//         }
//         return semiring::less(e1.first, e2.first);
//     }
//     struct entry_cmp {
//         [[nodiscard]] constexpr bool operator()(
//             const auto & e1, const auto & e2) const noexcept {
//             return compare_entries(e1.second, e2.second);
//         }
//     };
//     using heap = melon::d_ary_heap<2, melon::vertex_t<_Graph>, entry,
//     entry_cmp,
//                                    melon::vertex_map_t<_Graph, std::size_t>>;

//     static constexpr bool store_distances = false;
//     static constexpr bool store_paths = false;
// };
// template <melon::outward_incidence_graph _Graph, typename _ValueType>
// struct useless_arc_default_traits {
//     using semiring = melon::most_reliable_path_semiring<_ValueType>;
//     using entry = std::pair<_ValueType, bool>;
//     static bool compare_entries(const entry & e1, const entry & e2) {
//         if(e1.first == e2.first) {
//             return e2.second && !e1.second;
//         }
//         return semiring::less(e1.first, e2.first);
//     }
//     struct entry_cmp {
//         [[nodiscard]] constexpr bool operator()(
//             const auto & e1, const auto & e2) const noexcept {
//             return compare_entries(e1.second, e2.second);
//         }
//     };
//     using heap = melon::d_ary_heap<2, melon::vertex_t<_Graph>, entry,
//     entry_cmp,
//                                    melon::vertex_map_t<_Graph, std::size_t>>;

//     static constexpr bool store_distances = false;
//     static constexpr bool store_paths = false;
// };

// template <typename GR, typename V>
// struct path_dijkstra_traits {
//     using semiring = melon::most_reliable_path_semiring<V>;
//     struct entry_cmp {
//         [[nodiscard]] constexpr bool operator()(
//             const auto & e1, const auto & e2) const noexcept {
//             return semiring::less(e1.second, e2.second);
//         }
//     };
//     using heap = melon::d_ary_heap<4, melon::vertex_t<GR>, V, entry_cmp,
//                                    melon::vertex_map_t<GR, std::size_t>>;

//     static constexpr bool store_paths = true;
//     static constexpr bool store_distances = false;
// };
// }  // namespace detail

// template <case_c C>
// auto compute_strong_and_useless_arcs(
//     const C & instance_case, const bool parallel = false,
//     const auto & option_predicate = [](const option_t & o) { return true; })
//     { using graph_t = case_graph_t<C>; using arc_t = melon::arc_t<graph_t>;

//     const auto & graph = instance_case.graph();
//     const auto & quality_map = instance_case.vertex_quality_map();

//     auto useless_vertices_map = melon::create_vertex_map<bool>(graph);
//     for(const auto & v : melon::vertices(graph)) {
//         useless_vertices_map[v] =
//             (quality_map[v] == 0 &&
//              instance_case.vertex_options_map()[v].empty());
//     }

//     auto strong_arcs_map =
//         melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
//     auto useless_arcs_map =
//         melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

//     auto arcs_range = melon::arcs(graph);
//     const auto & base_probability_map = instance_case.arc_probability_map();
//     auto improved_probability_map = base_probability_map;
//     const auto & arc_options_map = instance_case.arc_options_map();
//     for(const arc_t & a : arcs_range) {
//         for(const auto & [improved_prob, option] : arc_options_map[a]) {
//             if(!option_predicate(option)) continue;
//             improved_probability_map[a] =
//                 std::max(improved_probability_map[a], improved_prob);
//         }
//     }

//     auto compute_strong_arcs =
//         [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
//                 arcs_block) {
//             arc_t uv;
//             auto sgraph = melon::views::subgraph(
//                 graph, {}, [&uv](const arc_t & a) -> bool { return a != uv;
//                 });
//             auto algo = melon::concurrent_dijkstras(
//                 detail::useless_arc_default_traits<graph_t, double>{},
//                 sgraph, improved_probability_map, base_probability_map);
//             for(const auto & a : arcs_block) {
//                 uv = a;
//                 auto && u = melon::arc_source(graph, uv);
//                 auto && v = melon::arc_target(graph, uv);
//                 algo.reset();
//                 algo.add_red_source(u);
//                 algo.add_blue_source(v, base_probability_map[uv]);
//                 for(const auto & [w, w_dist] : algo) {
//                     if(useless_vertices_map[w]) continue;
//                     strong_arcs_map[w].push_back(uv);
//                 }
//             }
//         };
//     if(parallel) {
//         tbb::parallel_for(
//             tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
//             compute_strong_arcs);
//     } else {
//         compute_strong_arcs(
//             tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
//     }

//     auto compute_useless_arcs =
//         [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
//                 arcs_block) {
//             arc_t uv;
//             auto sgraph = melon::views::subgraph(
//                 graph, {}, [&uv](const arc_t & a) -> bool { return a != uv;
//                 });
//             auto algo = melon::concurrent_dijkstras(
//                 detail::useless_arc_default_traits<graph_t, double>{},
//                 sgraph, improved_probability_map, base_probability_map);
//             for(const auto & a : arcs_block) {
//                 uv = a;
//                 auto && u = melon::arc_source(graph, uv);
//                 auto && v = melon::arc_target(graph, uv);
//                 useless_arcs_map[u].push_back(uv);
//                 algo.reset();
//                 algo.add_blue_source(u);
//                 algo.add_red_source(v, improved_probability_map[uv]);
//                 for(const auto & [w, w_dist] : algo) {
//                     if(useless_vertices_map[w]) continue;
//                     useless_arcs_map[w].push_back(uv);
//                 }
//             }
//         };
//     if(parallel) {
//         tbb::parallel_for(
//             tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
//             compute_useless_arcs);
//     } else {
//         compute_useless_arcs(
//             tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
//     }

//     // auto rgraph = melon::views::reverse(graph);
//     // auto algo = melon::dijkstra(
//     //     detail::path_dijkstra_traits<decltype(rgraph), double>{}, rgraph,
//     //     improved_probability_map);

//     // for(const auto & s : melon::vertices(rgraph)) {
//     //     if(useless_vertices_map[s]) continue;
//     //     algo.reset();
//     //     algo.add_source(s);
//     //     strong_arcs_map[s].clear();
//     //     for(const auto & [u, prob] : algo) {
//     //         if(u == s) continue;
//     //         strong_arcs_map[s].push_back(algo.pred_arc(u));
//     //     }
//     // }

//     return std::make_pair(strong_arcs_map, useless_arcs_map);
// }

// }  // namespace gecot
// }  // namespace fhamonic

namespace fhamonic {
namespace gecot {
namespace detail {
template <melon::outward_incidence_graph _Graph, typename _ValueType>
struct strong_arc_default_traits {
    using semiring = melon::shortest_path_semiring<_ValueType>;
    using entry = std::pair<_ValueType, bool>;
    static bool compare_entries(const entry & e1, const entry & e2) {
        if(e1.first == e2.first) {
            return e1.second && !e2.second;
        }
        return semiring::less(e1.first, e2.first);
    }
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return compare_entries(e1.second, e2.second);
        }
    };
    using heap = melon::d_ary_heap<2, melon::vertex_t<_Graph>, entry, entry_cmp,
                                   melon::vertex_map_t<_Graph, std::size_t>>;

    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};
template <melon::outward_incidence_graph _Graph, typename _ValueType>
struct useless_arc_default_traits {
    using semiring = melon::shortest_path_semiring<_ValueType>;
    using entry = std::pair<_ValueType, bool>;
    static bool compare_entries(const entry & e1, const entry & e2) {
        if(e1.first == e2.first) {
            return e2.second && !e1.second;
        }
        return semiring::less(e1.first, e2.first);
    }
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return compare_entries(e1.second, e2.second);
        }
    };
    using heap = melon::d_ary_heap<2, melon::vertex_t<_Graph>, entry, entry_cmp,
                                   melon::vertex_map_t<_Graph, std::size_t>>;

    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};

template <typename GR, typename V>
struct path_dijkstra_traits {
    using semiring = melon::most_reliable_path_semiring<V>;
    struct entry_cmp {
        [[nodiscard]] constexpr bool operator()(
            const auto & e1, const auto & e2) const noexcept {
            return semiring::less(e1.second, e2.second);
        }
    };
    using heap = melon::d_ary_heap<4, melon::vertex_t<GR>, V, entry_cmp,
                                   melon::vertex_map_t<GR, std::size_t>>;

    static constexpr bool store_paths = true;
    static constexpr bool store_distances = false;
};
}  // namespace detail

template <case_c C>
auto compute_strong_and_useless_arcs(
    const C & instance_case, const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; },
    double probability_resolution = 0.00000001) {
    using graph_t = case_graph_t<C>;
    using arc_t = melon::arc_t<graph_t>;
    using log_probability_t = uint64_t;

    spdlog::stopwatch prep_sw;
    spdlog::trace("Preprocessing of the '{}' graph:",
                    instance_case.name());

    auto prob_to_length =
        [probability_resolution](const double p) -> log_probability_t {
        assert(p != 0);
        return static_cast<log_probability_t>(
            -std::log(std::max(p, probability_resolution)) /
            std::log1p(probability_resolution));
    };

    const auto & graph = instance_case.graph();
    const auto & quality_map = instance_case.vertex_quality_map();

    auto useless_vertices_map = melon::create_vertex_map<bool>(graph);
    for(const auto & v : melon::vertices(graph)) {
        useless_vertices_map[v] =
            (quality_map[v] == 0 &&
             instance_case.vertex_options_map()[v].empty());
    }

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();

    auto base_length_map = melon::create_arc_map<log_probability_t>(graph);
    auto improved_length_map = melon::create_arc_map<log_probability_t>(graph);

    const auto & arc_options_map = instance_case.arc_options_map();
    for(const arc_t & a : arcs_range) {
        improved_length_map[a] = base_length_map[a] =
            prob_to_length(base_probability_map[a]);
        for(const auto & [improved_prob, option] : arc_options_map[a]) {
            if(!option_predicate(option)) continue;
            improved_length_map[a] =
                std::min(improved_length_map[a], prob_to_length(improved_prob));
        }
    }

    auto compute_strong_arcs = [&](const tbb::blocked_range<
                                   decltype(arcs_range.begin())> & arcs_block) {
        arc_t uv;
        auto sgraph = melon::views::subgraph(
            graph, {}, [&uv](const arc_t & a) -> bool { return a != uv; });
        auto algo = melon::concurrent_dijkstras(
            detail::useless_arc_default_traits<graph_t, log_probability_t>{},
            sgraph, base_length_map, improved_length_map);
        for(const auto & a : arcs_block) {
            uv = a;
            auto && u = melon::arc_source(graph, uv);
            auto && v = melon::arc_target(graph, uv);
            algo.reset();
            algo.add_red_source(u);
            algo.add_blue_source(v, base_length_map[uv]);
            for(const auto & [w, w_dist] : algo) {
                if(useless_vertices_map[w]) continue;
                strong_arcs_map[w].push_back(uv);
            }
        }
    };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_strong_arcs);
    } else {
        compute_strong_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    auto compute_useless_arcs =
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            arc_t uv;
            auto sgraph = melon::views::subgraph(
                graph, {}, [&uv](const arc_t & a) -> bool { return a != uv; });
            auto algo = melon::concurrent_dijkstras(
                detail::useless_arc_default_traits<graph_t,
                                                   log_probability_t>{},
                sgraph, base_length_map, improved_length_map);
            for(const auto & a : arcs_block) {
                uv = a;
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                algo.reset();
                algo.add_blue_source(u);
                algo.add_red_source(v, improved_length_map[uv]);
                for(const auto & [w, w_dist] : algo) {
                    if(useless_vertices_map[w]) continue;
                    useless_arcs_map[w].push_back(uv);
                }
            }
        };
    if(parallel) {
        tbb::parallel_for(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
            compute_useless_arcs);
    } else {
        compute_useless_arcs(
            tbb::blocked_range(arcs_range.begin(), arcs_range.end()));
    }

    if(spdlog::get_level() == spdlog::level::trace) {
        int nb_strong, nb_useless, nb_sinks;
        nb_strong = nb_useless = nb_sinks = 0;
        for(auto && v : melon::vertices(graph)) {
            if(instance_case.vertex_quality_map()[v] == 0 &&
                instance_case.vertex_options_map()[v].empty())
                continue;
            nb_strong += strong_arcs_map[v].size();
            nb_useless += useless_arcs_map[v].size();
            ++nb_sinks;
        }
        spdlog::trace("  {:>10.2f} strong arcs on average",
                        static_cast<double>(nb_strong) / nb_sinks);
        spdlog::trace("  {:>10.2f} useless arcs on average",
                        static_cast<double>(nb_useless) / nb_sinks);
        spdlog::trace(
            "          (took {} ms)",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                prep_sw.elapsed())
                .count());
    }

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP