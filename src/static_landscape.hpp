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

class StaticLandscape {
public:
    using Graph = melon::static_digraph;
    using QualityMap = std::vector<double>;
    using ProbabilityMap = std::vector<double>;

private:
    Graph _graph;

    QualityMap _node_quality_map;
    ProbabilityMap _arc_probability_map;

    std::vector<std::string> _node_names;
    phmap::node_hash_map<std::string, unsigned int> _node_name_to_id_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, unsigned int> _arc_name_to_id_map;

public:
    StaticLandscape() = default;

    auto & graph() const { return _graph; }
    auto & quality_map() const { return _node_quality_map; };
    auto & probability_map() const { return _arc_probability_map; };
};

}  // namespace fhamonic

#endif  // LANDSCAPE_HPP