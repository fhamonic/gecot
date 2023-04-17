#ifndef LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP
#define LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP

#include <concepts>
#include <ranges>

#include "landscape_opt/concepts/detail/range_of.hpp"
#include "landscape_opt/concepts/instance_case.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace concepts {

template <typename T>
concept Instance = InstanceCase<T> &&
    requires(T i, typename T::Option o, typename T::Solution s, typename T::OptionPotentialMap p) {
    { i.options() } -> detail::range_of<typename T::Option>;
    { i.option_cost(o) } -> std::same_as<double>;
    { i.create_solution() } -> std::same_as<typename T::Solution>;
    { i.create_options_potentials_map() } -> std::same_as<typename T::OptionPotentialMap>;
};

}  // namespace concepts
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_INSTANCE_HPP