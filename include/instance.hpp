#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <cassert>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include <parallel_hashmap/phmap.h>

#include "static_landscape.hpp"

namespace fhamonic {

class instance_case {
private:
    using graph_t = fhamonic::melon::static_digraph;

    std::string _case_name;
    graph_t _graph;

    std::vector<double> _vertex_quality_map;
    std::vector<double> _arc_probability_map;

    std::vector<std::string> _vertex_names;
    phmap::node_hash_map<std::string, melon::vertex_t<graph_t>>
        _vertex_name_to_id_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, melon::vertex_t<arc_t>>
        _arc_name_to_id_map;

    melon::vertex_map_t<melon::static_digraph,
                        std::vector<std::pair<double, Instance::option_t>>>
        _vertex_options_map;
    melon::arc_map_t<melon::static_digraph,
                     std::vector<std::pair<double, Instance::option_t>>>
        _arc_options_map;

public:
    [[nodiscard]] auto & graph() const noexcept { return _graph; }
    [[nodiscard]] auto & vertex_quality_map() const noexcept {
        return _vertex_quality_map;
    }
    [[nodiscard]] auto & arc_probability_map() const noexcept {
        return _arc_probability_map;
    }
    [[nodiscard]] auto & vertex_options_map() const noexcept {
        return _vertex_options_map;
    }
    [[nodiscard]] auto & arc_options_map() const noexcept {
        return _arc_options_map;
    }

public:
    [[nodiscard]] instance_case(
        std::string && case_name, fhamonic::melon::static_digraph && graph,
        std::vector<double> && vertex_quality_map,
        std::vector<double> && arc_probability_map,
        std::vector<std::string> && vertex_names,
        phmap::node_hash_map<std::string, melon::vertex_t<graph_t>> &&
            vertex_name_to_id_map,
        std::vector<std::string> && arc_names,
        phmap::node_hash_map<std::string, melon::arc_t<graph_t>> &&
            arc_name_to_id_map)
        : _case_name(std::move(_case_name))
        , _graph(std::move(graph))
        , _vertex_quality_map(std::move(vertex_quality_map))
        , _arc_probability_map(std::move(arc_probability_map))
        , _vertex_names(std::move(vertex_names))
        , _vertex_name_to_id_map(std::move(vertex_name_to_id_map))
        , _arc_names(std::move(arc_names))
        , _arc_name_to_id_map(std::move(arc_name_to_id_map)) {}

    auto option_from_name(const std::string & name) const {
        if(!contains_option(name))
            throw std::invalid_argument("in instance case '" + _case_name +
                                        "': unknwon vertex id '" + name + "'");
        return _vertex_name_to_id_map.at(name);
    }

    option_t option_from_name(const std::string & name) const {
        if(!contains_option(name))
            throw std::invalid_argument("in instance case '" + _case_name +
                                        "': unknwon arc id '" + name + "'");
        return _arc_name_to_id_map.at(name);
    }
};

class instance {
private:
    std::vector<double> _options_costs;
    std::vector<std::string> _options_names;
    phmap::node_hash_map<std::string, option_t> _option_name_to_id_map;

    std::vector<instance_case> _cases;

public:
    [[nodiscard]] auto options() const noexcept {
        return std::ranges::iota_view<option_t, option_t>(
            option_t{0}, static_cast<option_t>(nb_options()));
    }
    [[nodiscard]] double option_cost(option_t o) const {
        return _options_costs[o];
    }
    [[nodiscard]] auto create_solution() const {
        return std::vector<bool>(nb_options(), false);
    }
    [[nodiscard]] auto create_options_potentials_map() const {
        return std::vector<double>(nb_options(), 0.0);
    }
    [[nodiscard]] decltype(auto) cases() const noexcept { return _cases; }

public:
    instance() = default;

    std::size_t nb_options() const noexcept { return _options_costs.size(); }
    option_t add_option(std::string identifier, double c) noexcept {
        assert(!_option_name_to_id_map.contains(identifier));
        option_t i = static_cast<option_t>(nb_options());
        _options_names.emplace_back(identifier);
        _options_costs.emplace_back(c);
        _option_name_to_id_map[identifier] = i;
        return i;
    }
    bool contains_option(option_t i) const noexcept {
        return i < static_cast<option_t>(nb_options());
    }
    bool contains_option(std::string identifier) const noexcept {
        return _option_name_to_id_map.contains(identifier);
    }
    void set_option_cost(option_t i, double cost) noexcept {
        _options_costs[i] = cost;
    }
    const std::string & option_name(option_t i) const noexcept {
        return _options_names[i];
    }
    option_t option_from_name(const std::string & name) const {
        if(!contains_option(name))
            throw std::invalid_argument("unknwon option id '" + name + "'");
        return _option_name_to_id_map.at(name);
    }
};

}  // namespace fhamonic

#endif  // INSTANCE_HPP