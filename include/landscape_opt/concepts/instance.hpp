#ifndef LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP
#define LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP

#include <concepts>
#include <type_traits>
#include <utility>
#include <vector>

#include "melon/graph.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace detail {
template <typename T, typename V>
concept range_of =
    std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, V>;
}  // namespace detail

using option_t = unsigned int;
using case_id_t = std::size_t;

template <typename _Tp>
using case_graph_t = std::decay_t<decltype(std::declval<_Tp &>().graph())>;

template <typename _Tp>
using case_quality_map_t = std::decay_t<decltype(std::declval<_Tp &>().vertex_quality_map())>;

template <typename _Tp>
using case_probability_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().arc_probability_map())>;

// clang-format off
template <typename _Tp>
concept case_c = requires(_Tp && ic) {
    { ic.graph() } -> melon::outward_incidence_graph;
    { ic.vertex_quality_map() } 
    -> melon::input_value_map_of<melon::vertex_t<case_graph_t<_Tp>>, double>;
    { ic.arc_probability_map() } 
    -> melon::input_value_map_of<melon::arc_t<case_graph_t<_Tp>>, double>;
    { ic.vertex_options_map() } 
    -> melon::input_value_map_of<melon::vertex_t<case_graph_t<_Tp>>,
                                   std::vector<std::pair<double, option_t>>>;
    { ic.arc_options_map() } 
    -> melon::input_value_map_of<melon::arc_t<case_graph_t<_Tp>>,
                                   std::vector<std::pair<double, option_t>>>;
};

template <typename _Tp, typename _V>
using instance_option_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().template create_option_map<_V>())>;

template <typename _Tp>
using instance_solution_t = instance_option_map_t<_Tp, bool>;

template <typename _Tp>
using instance_options_rank_t = instance_option_map_t<_Tp, unsigned int>;

template <typename _Tp, typename _V>
using instance_case_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().template create_case_map<_V>())>;

template <typename _Tp>
using instance_cases_range_t = std::decay_t<decltype(std::declval<_Tp &>().cases())>;

template <typename _Tp>
using instance_case_t = std::ranges::range_value_t<instance_cases_range_t<_Tp>>;

template <typename _Tp>
using instance_graph_t = case_graph_t<instance_case_t<_Tp>>;

template <typename _Tp>
using instance_quality_map_t = case_quality_map_t<instance_case_t<_Tp>>;

template <typename _Tp>
using instance_probability_map_t = case_probability_map_t<instance_case_t<_Tp>>;

template <typename _Tp>
concept instance_c =
    requires(_Tp i, option_t o, instance_case_map_t<_Tp, double> case_values) {
        { i.options() } -> detail::range_of<option_t>;
        { i.option_cost(o) } -> std::convertible_to<double>;
        { i.template create_option_map<int>() } 
        -> melon::output_value_map_of<option_t, int>;
        { i.cases() } -> std::ranges::range;
        { i.template create_case_map<int>() } 
        -> melon::output_value_map_of<case_id_t, int>;
        { i.eval_criterion(case_values) } -> std::same_as<double>;
    } && case_c<instance_case_t<_Tp>>;
// clang-format on

}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_CONCEPTS_INSTANCE_CASE_HPP