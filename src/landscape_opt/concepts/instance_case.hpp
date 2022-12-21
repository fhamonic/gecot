#ifndef LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP
#define LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP

#include <concepts>

#include "melon/all.hpp"

#include "concepts/landscape.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

template <typename T>
concept InstanceCase = requires(T ic, typename T::Landscape,
                                typename T::Option o) {
    ic.landscape();
    ic.vertex_options_map();
    ic.arc_options_map();
};

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP