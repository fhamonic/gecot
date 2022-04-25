#ifndef LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP
#define LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP

#include <algorithm>
#include <atomic>

#include "melon/adaptor/reverse.hpp"
#include "melon/algorithm/dijkstra.hpp"
#include "melon/algorithm/robust_fiber.hpp"

#include "concepts/instance_case.hpp.hpp"

namespace fhamonic {
namespace landscape_opt {

template <concepts::InstanceCase I>
auto compute_generalized_flow_graph(const I & instance_case,
                                    typename I::Landscape::Graph::vertex_t t) {
    using Landscape = typename I::Landscape;
    using Graph = typename I::Landscape::Graph;
    using vertex_t = Graph ::vertex_t;
    using arc_t = Graph ::arc_t;

    
}

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_COMPUTE_GENERALIZED_FLOW_GRAPH_HPP