#ifndef GECOT_CONCEPTS_INSTANCE_CASE_HPP
#define GECOT_CONCEPTS_INSTANCE_CASE_HPP

#include <concepts>
#include <type_traits>
#include <utility>
#include <vector>

#include "melon/graph.hpp"

namespace fhamonic {
namespace gecot {
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
using case_source_quality_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().source_quality_map())>;

template <typename _Tp>
using source_quality_t =
    std::decay_t<melon::mapped_value_t<case_source_quality_map_t<_Tp>,
                                       melon::vertex_t<case_graph_t<_Tp>>>>;

template <typename _Tp>
using case_target_quality_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().target_quality_map())>;

template <typename _Tp>
using target_quality_t =
    std::decay_t<melon::mapped_value_t<case_target_quality_map_t<_Tp>,
                                       melon::vertex_t<case_graph_t<_Tp>>>>;

template <typename _Tp>
using case_probability_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().arc_probability_map())>;

template <typename _Tp>
using case_probability_t =
    std::decay_t<melon::mapped_value_t<case_probability_map_t<_Tp>,
                                       melon::arc_t<case_graph_t<_Tp>>>>;

// clang-format off
template <typename _Tp>
concept case_c = requires(_Tp && ic) {
    { ic.graph() } -> melon::outward_incidence_graph;
    { ic.source_quality_map() } 
    -> melon::input_mapping<melon::vertex_t<case_graph_t<_Tp>>>;
    { ic.target_quality_map() } 
    -> melon::input_mapping<melon::vertex_t<case_graph_t<_Tp>>>;
    { ic.arc_probability_map() } 
    -> melon::input_mapping<melon::arc_t<case_graph_t<_Tp>>>;
    { ic.vertex_options_map() } 
    -> melon::input_mapping<melon::vertex_t<case_graph_t<_Tp>>>;
    { ic.arc_options_map() } 
    -> melon::input_mapping<melon::arc_t<case_graph_t<_Tp>>>;
} && std::same_as<source_quality_t<_Tp>, double> 
  && std::same_as<case_probability_t<_Tp>, double>;
  // , std::vector<std::pair<double, option_t>>

template <typename _Tp, typename _V>
using instance_option_map_t =
    std::decay_t<decltype(std::declval<_Tp &>().template create_option_map<_V>())>;

template <typename _Tp>
using instance_solution_t = instance_option_map_t<_Tp, bool>;

template <typename _Tp>
using instance_options_rank_t = instance_option_map_t<_Tp, double>;

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
using instance_source_quality_map_t = case_source_quality_map_t<instance_case_t<_Tp>>;

template <typename _Tp>
using instance_probability_map_t = case_probability_map_t<instance_case_t<_Tp>>;

template <typename _Tp>
concept instance_c =
    requires(_Tp i, option_t o, instance_case_map_t<_Tp, double> case_values) {
        { i.options() } -> detail::range_of<option_t>;
        { i.option_cost(o) } -> std::convertible_to<double>;
        { i.template create_option_map<int>() } 
        -> melon::output_mapping<option_t>;
        { i.cases() } -> std::ranges::range;
        { i.template create_case_map<int>() } 
        -> melon::output_mapping<case_id_t>;
        { i.eval_criterion(case_values) } -> std::same_as<double>;
    } && case_c<instance_case_t<_Tp>>;
// clang-format on

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_CONCEPTS_INSTANCE_CASE_HPP