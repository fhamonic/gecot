#ifndef LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP
#define LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP

#include <concepts>

#include "melon.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

template <typename T>
concept Landscape = requires(T l, typename T::Graph, typename T::QualityMap,
                             typename T::ProbabilityMap) {
    l.graph();
    l.quality_map();
    l.probability_map();
};

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_LANDSCAPE_HPP