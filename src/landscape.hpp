#ifndef LANDSCAPE_HPP
#define LANDSCAPE_HPP

#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <parallel_hashmap/phmap.h>

#include "melon/static_digraph.hpp"

namespace fhamonic {

class Landscape {
public:
    using Option = int;
    using Solution = std::vector<double>;

private:
    melon::static_digraph _graph;

    std::vector<double> _node_quality_map;
    std::vector<double> _arc_probability_map;

    std::vector<std::string> _node_names;
    phmap::node_hash_map<std::string, unsigned int> _node_name_to_id_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, unsigned int> _arc_name_to_id_map;

public:
    Landscape() = default;

    auto & graph() const { return _graph; }
    auto & quality_map() const { return _node_quality_map; };
    auto & probability_map() const { return _arc_probability_map; };
};

}  // namespace fhamonic

#endif  // LANDSCAPE_HPP