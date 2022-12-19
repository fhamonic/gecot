#ifndef LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP
#define LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP

#include <utlility>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

#include "melon/robust_fiber.hpp"

#include "helper.hpp"

template <typename Graph, typename LM>
auto compute_strong_and_useless_arcs(const Graph & graph,
                                     const LM & lower_lengths,
                                     const LM & upper_lengths) {
    using node_t = typename Graph::node_t;
    using node_arcs_map = typename Graph::node_map<tbb::concurrent_vector<Arc>>;
    using arc_t = melon::arc_t<typename Graph>;

    node_arcs_map strong_arcs_map(graph.nb_vertices());
    node_arcs_map useless_arcs_map(graph.nb_vertices());

    auto arcs_range = graph.arcs();
    tbb::parallel_for(
        tbb::blocked_range(arcs_range.begin(), arcs_range.end()),
        [&](const tbb::blocked_range<decltype(arcs_range.begin())> &
                arcs_block) {
            arc_t uv;
            melon::RobustFiber identifyStrong(
                graph, lower_lengths, upper_lengths,
                [&strong_arcs_map, &uv](auto && v) {
                    strong_arcs_map[v].push_back(uv);
                },
                [](auto && v) {});
            melon::RobustFiber identifyUseless(
                graph, lower_lengths, upper_lengths, [](auto && v) {},
                [&useless_arcs_map, &uv](auto && v) {
                    useless_arcs_map[v].push_back(uv);
                });

            for(auto it = arcs_block.begin();;) {
                arc_t uv = *it;
                identifyStrong.add_strong_arc_source(uv).run().reset();
                identifyUseless.add_useless_arc_source(uv).run().reset();
            }
        });

    return std::make_pair(strong_arcs_map, useless_arcs_map);
}

#endif  // LANDSCAPE_OPT_COMPUTE_STRONG_AND_USELESS_ARCS_HPP