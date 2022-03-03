#ifndef LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP
#define LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP

#include <concepts>
#include <ranges>

#include "melon.hpp"

#include "concepts/detail/range_of.hpp"
#include "concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

template <typename T>
concept Instance = InstanceCase<T> &&
    requires(T i, typename T::Option o, typename T::Solution s) {
    { i.options() } -> detail::range_of<typename T::Option>;
    { i.option_cost(o) } -> std::same_as<double>;
    { i.create_solution() } -> std::same_as<typename T::Solution>;
    { s[o] } -> std::same_as<double &>;
};

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP