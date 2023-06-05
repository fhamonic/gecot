#ifndef LANDSCAPE_HPP
#define LANDSCAPE_HPP

#include <cassert>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <parallel_hashmap/phmap.h>

#include "melon/graph.hpp"
#include "melon/container/static_digraph.hpp"

namespace fhamonic {

class StaticLandscape {
public:
    using Graph = melon::static_digraph;
    using QualityMap = std::vector<double>;
    using ProbabilityMap = std::vector<double>;

private:
    Graph _graph;

    QualityMap _vertex_quality_map;
    ProbabilityMap _arc_probability_map;

    std::vector<std::string> _vertex_names;
    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        _name_to_vertex_map;
    std::vector<std::string> _arc_names;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        _name_to_arc_map;

public:
    StaticLandscape() = default;
    StaticLandscape(const StaticLandscape &) = default;
    StaticLandscape(StaticLandscape &&) = default;

    StaticLandscape & operator=(const StaticLandscape &) = default;
    StaticLandscape & operator=(StaticLandscape &&) = default;

    StaticLandscape(
        const Graph & g, const QualityMap & qm, const ProbabilityMap & pm,
        const std::vector<std::string> & vertex_names,
        const phmap::node_hash_map<std::string,
                                   melon::vertex_t<melon::static_digraph>>
            name_to_vertex_map,
        const std::vector<std::string> & arc_names,
        const phmap::node_hash_map<std::string,
                                   melon::arc_t<melon::static_digraph>>
            name_to_arc_map)
        : _graph(g)
        , _vertex_quality_map(qm)
        , _arc_probability_map(pm)
        , _vertex_names(vertex_names)
        , _name_to_vertex_map(name_to_vertex_map)
        , _arc_names(arc_names)
        , _name_to_arc_map(name_to_arc_map) {}

    auto & graph() const noexcept { return _graph; }
    auto & quality_map() const noexcept { return _vertex_quality_map; };
    auto & probability_map() const noexcept { return _arc_probability_map; };

    
};

}  // namespace fhamonic

#endif  // LANDSCAPE_HPP