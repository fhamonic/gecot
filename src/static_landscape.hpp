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
    phmap::node_hash_map<std::string, melon::static_digraph::vertex>
        _name_to_vertex_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, melon::static_digraph::arc>
        _name_to_arc_map;

public:
    StaticLandscape() = default;
    StaticLandscape(const StaticLandscape &) = default;
    StaticLandscape(StaticLandscape &&) = default;

    StaticLandscape & operator=(const StaticLandscape &) = default;
    StaticLandscape & operator=(StaticLandscape &&) = default;

    StaticLandscape(
        const Graph & g, const QualityMap & qm, const ProbabilityMap & pm,
        const std::vector<std::string> & node_names,
        const phmap::node_hash_map<std::string, melon::static_digraph::vertex>
            name_to_vertex_map,
        const std::vector<std::string> & arc_names,
        const phmap::node_hash_map<std::string, melon::static_digraph::arc>
            name_to_arc_map)
        : _graph(g)
        , _node_quality_map(qm)
        , _arc_probability_map(pm)
        , _node_names(node_names)
        , _name_to_vertex_map(name_to_vertex_map)
        , _arc_names(arc_names)
        , _name_to_arc_map(name_to_arc_map) {}

    auto & graph() const { return _graph; }
    auto & quality_map() const { return _node_quality_map; };
    auto & probability_map() const { return _arc_probability_map; };

    bool contains_vertex(const std::string & name) const {
        return _name_to_vertex_map.contains(name);
    }
    bool contains_arc(const std::string & name) const {
        return _name_to_arc_map.contains(name);
    }
    Graph::vertex vertex_from_name(const std::string & name) const {
        assert(contains_vertex(name));
        return _name_to_vertex_map.at(name);
    }
    Graph::arc arc_from_name(const std::string & name) const {
        assert(contains_arc(name));
        return _name_to_arc_map.at(name);
    }
};

}  // namespace fhamonic

#endif  // LANDSCAPE_HPP