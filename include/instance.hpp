#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <cassert>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <parallel_hashmap/phmap.h>

#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/criterion.hpp"

class InstanceCase {
private:
    using case_id_t = gecot::case_id_t;
    using graph_t = melon::static_digraph;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    using source_quality_map_t = std::vector<double>;
    using target_quality_map_t = std::vector<double>;
    using arc_probability_map_t = std::vector<double>;

    using vertex_options_map_t = melon::vertex_map_t<
        graph_t, std::vector<std::tuple<double, double, gecot::option_t>>>;
    using arc_options_map_t =
        melon::arc_map_t<graph_t,
                         std::vector<std::pair<double, gecot::option_t>>>;

    case_id_t _case_id;
    std::string _case_name;
    graph_t _graph;

    source_quality_map_t _source_quality_map;
    target_quality_map_t _target_quality_map;
    arc_probability_map_t _arc_probability_map;

    std::vector<std::string> _vertex_names;
    phmap::node_hash_map<std::string, vertex_t> _vertex_name_to_id_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, arc_t> _arc_name_to_id_map;

    vertex_options_map_t _vertex_options_map;
    arc_options_map_t _arc_options_map;

public:
    [[nodiscard]] case_id_t id() const noexcept;
    [[nodiscard]] const std::string & name() const noexcept;
    [[nodiscard]] const graph_t & graph() const noexcept;
    [[nodiscard]] const source_quality_map_t & source_quality_map()
        const noexcept;
    [[nodiscard]] const target_quality_map_t & target_quality_map()
        const noexcept;
    [[nodiscard]] const arc_probability_map_t & arc_probability_map()
        const noexcept;
    [[nodiscard]] const vertex_options_map_t & vertex_options_map()
        const noexcept;
    [[nodiscard]] const arc_options_map_t & arc_options_map() const noexcept;

public:
    [[nodiscard]] InstanceCase(
        case_id_t id, const std::string & case_name,
        melon::static_digraph && graph,
        source_quality_map_t && source_quality_map,
        target_quality_map_t && target_quality_map,
        arc_probability_map_t && arc_probability_map,
        std::vector<std::string> && vertex_names,
        phmap::node_hash_map<std::string, vertex_t> && vertex_name_to_id_map,
        std::vector<std::string> && arc_names,
        phmap::node_hash_map<std::string, arc_t> && arc_name_to_id_map);

    bool contains_vertex(const std::string & name) const;
    bool contains_arc(const std::string & name) const;
    [[nodiscard]] vertex_t vertex_from_name(const std::string & name) const;
    [[nodiscard]] arc_t arc_from_name(const std::string & name) const;

    [[nodiscard]] const std::string & vertex_name(const vertex_t & v) const;
    [[nodiscard]] const std::string & arc_name(const arc_t & a) const;
    template <typename T>
    void set_vertex_options_map(T && t) {
        _vertex_options_map = std::forward<T>(t);
    }
    template <typename T>
    void set_arc_options_map(T && t) {
        _arc_options_map = std::forward<T>(t);
    }
};

using gecot::criterion_constant;
using gecot::criterion_formula;
using gecot::criterion_min;
using gecot::criterion_product;
using gecot::criterion_sum;
using gecot::criterion_var;

class Instance {
private:
    std::vector<std::string> _cost_dimensions_names = {"cost"};
    // the costs of option o in every dimension, from o * dimensions on
    std::vector<double> _options_costs;
    std::vector<std::string> _options_names;
    phmap::node_hash_map<std::string, gecot::option_t> _option_name_to_id_map;
    // at most one option of each set is bought
    std::vector<std::vector<gecot::option_t>> _conflict_sets;
    std::vector<std::string> _conflict_sets_names;

    std::vector<InstanceCase> _cases;
    criterion_formula _criterion;

public:
    [[nodiscard]] std::ranges::iota_view<gecot::option_t, gecot::option_t>
    options() const noexcept;
    // throws if the instance has several cost dimensions
    [[nodiscard]] double option_cost(gecot::option_t o) const;
    [[nodiscard]] double option_cost(gecot::option_t o, std::size_t r) const;
    [[nodiscard]] std::size_t num_cost_dimensions() const noexcept;
    [[nodiscard]] const std::string & cost_dimension_name(std::size_t r) const;
    [[nodiscard]] const std::vector<std::vector<gecot::option_t>> &
    conflict_sets() const noexcept;
    [[nodiscard]] const std::string & conflict_set_name(std::size_t l) const;
    template <typename V>
    [[nodiscard]] auto create_option_map() const {
        return melon::static_map<gecot::option_t, V>(num_options());
    }
    template <typename V>
    [[nodiscard]] auto create_option_map(V v) const {
        return melon::static_map<gecot::option_t, V>(num_options(), v);
    }
    [[nodiscard]] const std::vector<InstanceCase> & cases() const noexcept;
    template <typename V>
    [[nodiscard]] auto create_case_map() const {
        return melon::static_map<gecot::case_id_t, V>(_cases.size());
    }
    template <typename V>
    [[nodiscard]] auto create_case_map(V v) const {
        return melon::static_map<gecot::case_id_t, V>(_cases.size(), v);
    }
    template <melon::mapping<gecot::case_id_t> M>
        requires std::convertible_to<melon::mapped_value_t<M, gecot::case_id_t>,
                                     double>
    [[nodiscard]] double eval_criterion(const M & case_values) const noexcept {
        return std::visit(gecot::formula_eval_visitor{case_values}, _criterion);
    }
    [[nodiscard]] const criterion_formula & criterion() const noexcept;

public:
    Instance() = default;

    [[nodiscard]] std::size_t num_options() const noexcept;
    // before any option is added
    void set_cost_dimensions(std::vector<std::string> names);
    gecot::option_t add_option(std::string identifier,
                               const std::vector<double> & costs);
    void add_conflict_set(std::string name,
                          std::vector<gecot::option_t> options);
    bool contains_option(gecot::option_t i) const noexcept;
    bool contains_option(std::string identifier) const noexcept;
    const std::string & option_name(gecot::option_t i) const noexcept;
    gecot::option_t option_from_name(const std::string & name) const;
    template <class... T>
    [[nodiscard]] decltype(auto) emplace_case(T &&... args) {
        return _cases.emplace_back(_cases.size(), std::forward<T>(args)...);
    }
    void set_criterion(criterion_formula && c);
    void set_criterion(const criterion_formula & c);
    gecot::case_id_t case_id_from_name(const std::string & name) const;
};

#endif  // INSTANCE_HPP