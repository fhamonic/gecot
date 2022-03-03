#ifndef LANDSCAPE_OPT_CONCEPTS_DETAIL_RANGE_OF_HPP
#define LANDSCAPE_OPT_CONCEPTS_DETAIL_RANGE_OF_HPP

#include <concepts>
#include <ranges>

namespace fhamonic {
namespace landscape_opt {
namespace concepts {
namespace detail {

template <typename T, typename V>
concept range_of = std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, V>;

}  // namespace detail
}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_DETAIL_RANGE_OF_HPP