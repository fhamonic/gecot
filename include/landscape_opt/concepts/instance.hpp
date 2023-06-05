#ifndef LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP
#define LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP

#include <concepts>

#include "melon/graph.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {
template <typename T, typename V>
concept range_of = std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, V>;
}  // namespace detail

using option_t = unsigned int;

template <typename _Tp>
using case_graph_t = decltype(std::declval<_Tp &>().graph());

// clang-format off
template <typename _Tp>
concept case_c = requires(_Tp ic) {
    { ic.graph() } -> melon::outward_incidence_graph;
    { ic.vertex_quality_map() } 
            -> melon::output_value_map_of<melon::vertex_t<case_graph_t<_Tp>>,
                                    double>;
    { ic.arc_probability_map() } 
            -> melon::output_value_map_of<melon::arc_t<case_graph_t<_Tp>>,
                                    double>;
    { ic.vertex_options_map() } 
            -> melon::output_value_map_of<melon::vertex_t<case_graph_t<_Tp>>, 
                                    std::vector<std::pair<double, option_t>>>;
    { ic.arc_options_map() } 
            -> melon::output_value_map_of<melon::arc_t<case_graph_t<_Tp>>, 
                                    std::vector<std::pair<double, option_t>>>;
};

template <typename _Tp>
concept instance_c = requires(_Tp i, option_t o) {
    { i.options() } -> detail::range_of<option_t>;
    { i.option_cost(o) } -> std::convertible_to<double>;
    { i.create_solution() } -> melon::output_value_map_of<option_t, bool>;
    { i.create_options_potentials_map() } 
            -> melon::output_value_map_of<option_t, double>;
    { i.cases() } -> std::ranges::range;
} && case_c<std::ranges::range_value_t<decltype(std::declval<_Tp>().cases())>>;
// clang-format on

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP