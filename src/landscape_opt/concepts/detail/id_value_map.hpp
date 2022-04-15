#ifndef LANDSCAPE_OPT_CONCEPTS_DETAIL_ID_VALUE_MAP_HPP
#define LANDSCAPE_OPT_CONCEPTS_DETAIL_ID_VALUE_MAP_HPP

#include <concepts>

namespace fhamonic {
namespace landscape_opt {
namespace concepts {
namespace detail {

// clang-format off
template <typename M, typename I, typename V>
concept id_value_map = requires(M m, I id) {
    { m[id] } -> std::same_as<V>;
    { m.at(id) } -> std::same_as<V>;
};
// clang-format on

}  // namespace detail
}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_DETAIL_ID_VALUE_MAP_HPP