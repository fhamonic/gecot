#ifndef LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP

#include <algorithm>
#include <atomic>

#include "melon/adaptor/reverse.hpp"
#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/strong_fiber.hpp"

#include "concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_generalized_flow_graph(const I & instance_case,
                                    melon::vertex_t<typename I::Landscape::Graph> t) {
    using Landscape = typename I::Landscape;
    using Graph = typename I::Landscape::Graph;
    using vertex = melon::vertex_t<Graph>;
    using arc = melon::arc_t<Graph>;

    
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP