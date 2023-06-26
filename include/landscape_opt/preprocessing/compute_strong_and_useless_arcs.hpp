#ifndef LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utility>

#include <tbb/concurrent_vector.h>

#include "melon/algorithm/strong_fiber.hpp"
#include "melon/views/subgraph.hpp"

#include "landscape_opt/helper.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {
template <typename G, typename T>
struct strong_arc_default_traits {
    using semiring = melon::most_reliable_path_semiring<T>;
    template <typename CMP = std::less<std::pair<melon::vertex_t<G>, T>>>
    using heap = melon::d_ary_heap<2, melon::vertex_t<G>, T, CMP,
                                   melon::vertex_map_t<G, std::size_t>>;

    static constexpr bool strictly_strong = false;
    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};
template <typename G, typename T>
struct useless_arc_default_traits {
    using semiring = melon::most_reliable_path_semiring<T>;
    template <typename CMP = std::less<std::pair<melon::vertex_t<G>, T>>>
    using heap = melon::d_ary_heap<2, melon::vertex_t<G>, T, CMP,
                                   melon::vertex_map_t<G, std::size_t>>;

    static constexpr bool strictly_strong = true;
    static constexpr bool store_distances = false;
    static constexpr bool store_paths = false;
};

}  // namespace detail

template <case_c C>
auto compute_strong_and_useless_arcs(
    const C & instance_case, const bool parallel = false,
    const auto & option_predicate = [](const option_t & o) { return true; }) {
    using graph_t = case_graph_t<C>;
    using probability_map_t = case_probability_map_t<C>;
    using arc_t = melon::arc_t<graph_t>;

    const auto & graph = instance_case.graph();

    auto strong_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);
    auto useless_arcs_map =
        melon::create_vertex_map<tbb::concurrent_vector<arc_t>>(graph);

    auto arcs_range = melon::arcs(graph);
    const auto & base_probability_map = instance_case.arc_probability_map();
    auto improved_probability_map = base_probability_map;
    const auto & arc_options_map = instance_case.arc_options_map();
    for(const arc_t & a : arcs_range) {
        for(const auto & [improved_prob, option] : arc_options_map[a]) {
            if(!option_predicate(option)) continue;
            improved_probability_map[a] =
                std::max(improved_probability_map[a], improved_prob);
        }
    }

    auto compute_strong_arcs = [&](const tbb::blocked_range<
                                   decltype(arcs_range.begin())> & arcs_block) {
        arc_t uv;
        auto sgraph = melon::views::subgraph(
            graph, {}, melon::views::map([&uv](const arc_t & a) -> bool {
                return a != uv;
            }));
        // melon::strong_fiber<graph_t, probability_map_t, probability_map_t,
        //                     detail::strong_arc_default_traits<graph_t,
        //                     double>>
        //     algo(graph, improved_probability_map, base_probability_map);
        melon::strong_fiber<decltype(sgraph), probability_map_t,
                            probability_map_t,
                            detail::strong_arc_default_traits<graph_t, double>>
            algo(sgraph, improved_probability_map, base_probability_map);
        for(const auto & a : arcs_block) {
            uv = a;
            auto && u = melon::arc_source(graph, uv);
            auto && v = melon::arc_target(graph, uv);
            // algo.reset().add_source(
            //     u, melon::views::map(
            //            [&uv](const arc_t & a) -> bool { return a == uv; }));
            algo.reset();
            algo.relax_useless_vertex(u);
            algo.relax_strong_vertex(v, base_probability_map[uv]);
            for(const auto & [w, w_dist] : algo) {
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
                graph, {}, melon::views::map([&uv](const arc_t & a) -> bool {
                    return a != uv;
                }));
            // melon::strong_fiber<graph_t, probability_map_t,
            // probability_map_t,
            //                     detail::useless_arc_default_traits<graph_t,
            //                     double>>
            //     algo(graph, improved_probability_map, base_probability_map);
            melon::strong_fiber<
                decltype(sgraph), probability_map_t, probability_map_t,
                detail::useless_arc_default_traits<graph_t, double>>
                algo(sgraph, improved_probability_map, base_probability_map);
            for(const auto & a : arcs_block) {
                uv = a;
                auto && u = melon::arc_source(graph, uv);
                auto && v = melon::arc_target(graph, uv);
                // useless_arcs_map[u].push_back(uv);
                // algo.reset().add_source(
                //     u, melon::views::map(
                //            [&uv](const arc_t & a) -> bool { return a != uv;
                //            }));
                algo.reset();
                algo.relax_strong_vertex(u);
                algo.relax_useless_vertex(v, improved_probability_map[uv]);
                for(const auto & [w, w_dist] : algo) {
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

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP