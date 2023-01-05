#ifndef LANDSCAPE_OPT_CONCEPTS_MULTI_INSTANCE_HPP
#define LANDSCAPE_OPT_CONCEPTS_MULTI_INSTANCE_HPP

#include <concepts>

#include "concepts/detail/range_of.hpp"
#include "concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

template <typename T>
concept MultiInstance = requires(T mi, typename T::Option o, typename T::Solution s) {
    { mi.options() } -> detail::range_of<typename T::Option>;
    { mi.option_cost(o) } -> std::same_as<double>;
    { mi.cases() } -> std::ranges::range;
    { *std::ranges::begin(mi.cases()) } -> InstanceCase;
    { mi.option_cost(o) } -> std::same_as<double>;
    { mi.create_solution() } -> std::same_as<typename T::Solution>;
};

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_MULTI_INSTANCE_HPP