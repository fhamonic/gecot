#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <cassert>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
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
    template <typename T>
    void set_vertex_options_map(T && t) {
        _vertex_options_map = std::forward<T>(t);
    }
    template <typename T>
    void set_arc_options_map(T && t) {
        _arc_options_map = std::forward<T>(t);
    }
};

class Instance {
private:
    std::vector<double> _options_costs;
    std::vector<std::string> _options_names;
    phmap::node_hash_map<std::string, landscape_opt::option_t>
        _option_name_to_id_map;

    std::vector<InstanceCase> _cases;

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
    [[nodiscard]] decltype(auto) cases() const noexcept { return _cases; }
    template <typename V>
    [[nodiscard]] auto create_case_map(V v = {}) const {
        return melon::static_map<std::size_t, V>(_cases.size(), v);
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
};

}  // namespace fhamonic

#endif  // INSTANCE_HPP