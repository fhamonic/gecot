#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <cassert>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <parallel_hashmap/phmap.h>

#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"

#include "landscape_opt/concepts/instance.hpp"

namespace fhamonic {

class InstanceCase {
private:
    using graph_t = fhamonic::melon::static_digraph;

    std::size_t _case_id;
    std::string _case_name;
    graph_t _graph;

    std::vector<double> _vertex_quality_map;
    std::vector<double> _arc_probability_map;

    std::vector<std::string> _vertex_names;
    phmap::node_hash_map<std::string, melon::vertex_t<graph_t>>
        _vertex_name_to_id_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, melon::arc_t<graph_t>>
        _arc_name_to_id_map;

    melon::vertex_map_t<melon::static_digraph,
                        std::vector<std::pair<double, landscape_opt::option_t>>>
        _vertex_options_map;
    melon::arc_map_t<melon::static_digraph,
                     std::vector<std::pair<double, landscape_opt::option_t>>>
        _arc_options_map;

public:
    [[nodiscard]] auto id() const noexcept { return _case_id; }
    [[nodiscard]] auto name() const noexcept { return _case_name; }
    [[nodiscard]] auto & graph() const noexcept { return _graph; }
    [[nodiscard]] const auto & vertex_quality_map() const noexcept {
        return _vertex_quality_map;
    }
    [[nodiscard]] const auto & arc_probability_map() const noexcept {
        return _arc_probability_map;
    }
    [[nodiscard]] auto & vertex_options_map() const noexcept {
        return _vertex_options_map;
    }
    [[nodiscard]] auto & arc_options_map() const noexcept {
        return _arc_options_map;
    }

public:
    [[nodiscard]] InstanceCase(
        std::size_t id, std::string && case_name,
        fhamonic::melon::static_digraph && graph,
        std::vector<double> && vertex_quality_map,
        std::vector<double> && arc_probability_map,
        std::vector<std::string> && vertex_names,
        phmap::node_hash_map<std::string, melon::vertex_t<graph_t>> &&
            vertex_name_to_id_map,
        std::vector<std::string> && arc_names,
        phmap::node_hash_map<std::string, melon::arc_t<graph_t>> &&
            arc_name_to_id_map)
        : _case_id(id)
        , _case_name(std::move(case_name))
        , _graph(std::move(graph))
        , _vertex_quality_map(std::move(vertex_quality_map))
        , _arc_probability_map(std::move(arc_probability_map))
        , _vertex_names(std::move(vertex_names))
        , _vertex_name_to_id_map(std::move(vertex_name_to_id_map))
        , _arc_names(std::move(arc_names))
        , _arc_name_to_id_map(std::move(arc_name_to_id_map)) {}

    bool contains_vertex(const std::string & name) const {
        return _vertex_name_to_id_map.contains(name);
    }
    bool contains_arc(const std::string & name) const {
        return _arc_name_to_id_map.contains(name);
    }
    [[nodiscard]] auto vertex_from_name(const std::string & name) const {
        if(!contains_vertex(name))
            throw std::invalid_argument("unknwon vertex id '" + name + "'");
        return _vertex_name_to_id_map.at(name);
    }
    [[nodiscard]] auto arc_from_name(const std::string & name) const {
        if(!contains_arc(name))
            throw std::invalid_argument("unknwon arc id '" + name + "'");
        return _arc_name_to_id_map.at(name);
    }

    [[nodiscard]] auto vertex_name(const melon::vertex_t<graph_t> & v) const {
        return _vertex_names.at(v);
    }
    [[nodiscard]] auto arc_name(const melon::arc_t<graph_t> & a) const {
        return _arc_names.at(a);
    }
    template <typename T>
    void set_vertex_options_map(T && t) {
        _vertex_options_map = std::forward<T>(t);
    }
    template <typename T>
    void set_arc_options_map(T && t) {
        _arc_options_map = std::forward<T>(t);
    }
};

using criterion_constant = double;
using criterion_var = landscape_opt::case_id_t;
struct criterion_sum;
struct criterion_product;
struct criterion_min;

using criterion_formula =
    std::variant<criterion_constant, criterion_var,
                 criterion_sum, criterion_product, criterion_min>;

struct criterion_sum {
    std::vector<criterion_formula> values;
};
struct criterion_product {
    std::vector<criterion_formula> values;
};
struct criterion_min {
    std::vector<criterion_formula> values;
};

template <typename M>
struct formula_eval_visitor {
    std::reference_wrapper<const M> value_map;

    formula_eval_visitor(const M & t_value_map) : value_map{t_value_map} {}

    double operator()(const criterion_constant & c) { return c; }
    double operator()(const criterion_var & v) { return value_map.get()[v]; }
    double operator()(const criterion_sum & f) {
        double sum = 0.0;
        for(auto && e : f.values) sum += std::visit(*this, e);
        return sum;
    }
    double operator()(const criterion_product & f) {
        double product = 1.0;
        for(auto && e : f.values) product *= std::visit(*this, e);
        return product;
    }
    double operator()(const criterion_min & f) {
        double min = std::numeric_limits<double>::max();
        for(auto && e : f.values) min = std::min(min, std::visit(*this, e));
        return min;
    }
};

class Instance {
private:
    std::vector<double> _options_costs;
    std::vector<std::string> _options_names;
    phmap::node_hash_map<std::string, landscape_opt::option_t>
        _option_name_to_id_map;

    std::vector<InstanceCase> _cases;
    criterion_formula _criterion;

public:
    [[nodiscard]] auto options() const noexcept {
        return std::ranges::iota_view<landscape_opt::option_t,
                                      landscape_opt::option_t>(
            landscape_opt::option_t{0},
            static_cast<landscape_opt::option_t>(nb_options()));
    }
    [[nodiscard]] double option_cost(landscape_opt::option_t o) const {
        return _options_costs[o];
    }
    template <typename V>
    [[nodiscard]] auto create_option_map(V v = {}) const {
        return melon::static_map<landscape_opt::option_t, V>(nb_options(), v);
    }
    [[nodiscard]] auto & cases() const noexcept { return _cases; }
    template <typename V>
    [[nodiscard]] auto create_case_map(V v = {}) const {
        return melon::static_map<landscape_opt::case_id_t, V>(_cases.size(), v);
    }
    template <melon::input_value_map_of<landscape_opt::case_id_t, double> M>
    [[nodiscard]] double eval_criterion(const M & case_values) const noexcept {
        return std::visit(formula_eval_visitor{case_values}, _criterion);
    }
   [[nodiscard]] auto & criterion() const noexcept {
        return _criterion;
    }

public:
    Instance() = default;

    [[nodiscard]] std::size_t nb_options() const noexcept {
        return _options_costs.size();
    }
    landscape_opt::option_t add_option(std::string identifier,
                                       double c) noexcept {
        assert(!_option_name_to_id_map.contains(identifier));
        landscape_opt::option_t i =
            static_cast<landscape_opt::option_t>(nb_options());
        _options_names.emplace_back(identifier);
        _options_costs.emplace_back(c);
        _option_name_to_id_map[identifier] = i;
        return i;
    }
    bool contains_option(landscape_opt::option_t i) const noexcept {
        return i < static_cast<landscape_opt::option_t>(nb_options());
    }
    bool contains_option(std::string identifier) const noexcept {
        return _option_name_to_id_map.contains(identifier);
    }
    void set_option_cost(landscape_opt::option_t i, double cost) noexcept {
        _options_costs[i] = cost;
    }
    const std::string & option_name(landscape_opt::option_t i) const noexcept {
        return _options_names[i];
    }
    landscape_opt::option_t option_from_name(const std::string & name) const {
        if(!contains_option(name))
            throw std::invalid_argument("unknwon option id '" + name + "'");
        return _option_name_to_id_map.at(name);
    }
    template <class... T>
    [[nodiscard]] decltype(auto) emplace_case(T &&... args) {
        return _cases.emplace_back(_cases.size(), std::forward<T>(args)...);
    }
    void set_criterion(criterion_formula && c) { _criterion = c; }
    landscape_opt::case_id_t case_id_from_name(const std::string & name) const {
        for(auto && instance_case : _cases) {
            if(instance_case.name() == name) return instance_case.id();
        }
        throw std::invalid_argument("unknwon case '" + name + "'");
    }
};

}  // namespace fhamonic

#endif  // INSTANCE_HPP