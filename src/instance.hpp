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

class Instance {
public:
    using Landscape = StaticLandscape;
    using Option = int;
    using Solution = std::vector<double>;

private:
    std::vector<double> _options_costs;
    std::vector<std::string> _options_names;
    phmap::node_hash_map<std::string, Option> _option_name_to_id_map;

    Landscape _landscape;

    std::vector<std::vector<std::pair<double, Option>>> _node_options_map;
    std::vector<std::vector<std::pair<double, Option>>> _arc_options_map;

public:
    Instance() = default;

    auto options() const noexcept {
        return std::ranges::iota_view<Option, Option>(
            static_cast<Option>(0), static_cast<Option>(nb_options()));
    }
    double option_cost(Option o) const { return _options_costs[o]; }
    Solution create_solution() const { return Solution(nb_options(), 0.0); }

    auto & landscape() const noexcept { return _landscape; }
    auto & node_options_map() const noexcept { return _node_options_map; }
    auto & arc_options_map() const noexcept { return _arc_options_map; }

    std::size_t nb_options() const noexcept { return _options_costs.size(); }

    Option add_option(std::string identifier, double c) noexcept {
        assert(!_option_name_to_id_map.contains(identifier));
        Option i = _options_costs.size();
        _options_names.emplace_back(identifier);
        _options_costs.emplace_back(c);
        _option_name_to_id_map[identifier] = i;
        return i;
    }

    bool contains_option(Option i) const noexcept {
        return i >= 0 && i < nb_options();
    }
    bool contains_option(std::string identifier) const noexcept {
        return _option_name_to_id_map.contains(identifier);
    }
    void set_option_cost(Option i, double cost) noexcept {
        _options_costs[i] = cost;
    }
    const std::string & option_name(Option i) const noexcept {
        return _options_names[i];
    }
    Option option_from_name(std::string identifier) const noexcept {
        assert(contains_option(identifier));
        return _option_name_to_id_map.at(identifier);
    }

    void set_landscape(StaticLandscape && l) noexcept {
        _landscape = std::move(l);
    }
    void set_node_options(
        std::vector<std::vector<std::pair<double, Instance::Option>>> &&
            node_options) noexcept {
        _node_options_map = std::move(node_options);
    }
    void set_arc_options(
        std::vector<std::vector<std::pair<double, Instance::Option>>> &&
            arc_options) noexcept {
        _arc_options_map = std::move(arc_options);
    }
};

}  // namespace fhamonic

#endif  // INSTANCE_HPP