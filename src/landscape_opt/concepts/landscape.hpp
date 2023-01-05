#ifndef LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP
#define LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP

#include <concepts>

#include "melon/graph.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

// clang-format off
template <typename T>
concept Landscape =
    requires(T l, typename T::Graph, typename T::QualityMap,
             typename T::ProbabilityMap) {
        { l.graph() } -> melon::graph;
        { l.quality_map() }
                -> melon::input_value_map<melon::vertex_t<typename T::Graph>>;
        { l.probability_map() } 
                -> melon::input_value_map<melon::arc_t<typename T::Graph>>;
    };
// clang-format on

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP