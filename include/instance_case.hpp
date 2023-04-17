#ifndef INSTANCE_CASE_HPP
#define INSTANCE_CASE_HPP

#include <boost/program_options.hpp>


// template <typename T>
// concept Instance = InstanceCase<T> &&
//     requires(T i, typename T::Option o, typename T::Solution s) {
//     { i.options() } -> detail::range_of<typename T::Option>;
//     { i.option_cost(o) } -> std::same_as<double>;
//     { i.create_solution() } -> std::same_as<typename T::Solution>;
//     { s[o] } -> std::same_as<double &>;
// };

#include <cassert>
#include <memory>
#include <utility>

#include <parallel_hashmap/phmap.h>

#include "landscape_opt/landscape/mutable_landscape.hpp"
#include "landscape_opt/solvers/concept/restoration_plan.hpp"

using Option = int;

struct InstanceCase {
    double coef;
    void * graph;

    std::vector<double> vertex_quality_map;
    std::vector<double> arc_probability_map;

    std::vector<std::string> vertex_names;
    phmap::node_hash_map<std::string, unsigned int> vertex_name_to_id_map;
    std::vector<std::string> arc_names;
    phmap::node_hash_map<std::string, unsigned int> arc_name_to_id_map;

    std::vector<std::vector<std::pair<Option, double>>> vertex_options_map;
    std::vector<std::vector<std::pair<Option, double>>> arc_options_map;
};

#endif  // INSTANCE_CASE_HPP