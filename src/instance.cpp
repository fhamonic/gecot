#include "instance.hpp"

using namespace fhamonic;

// InstanceCase

[[nodiscard]] InstanceCase::case_id_t InstanceCase::id() const noexcept {
    return _case_id;
}
[[nodiscard]] const std::string & InstanceCase::name() const noexcept {
    return _case_name;
}
[[nodiscard]] const InstanceCase::graph_t & InstanceCase::graph()
    const noexcept {
    return _graph;
}
[[nodiscard]] const InstanceCase::source_quality_map_t &
InstanceCase::source_quality_map() const noexcept {
    return _source_quality_map;
}
[[nodiscard]] const InstanceCase::target_quality_map_t &
InstanceCase::target_quality_map() const noexcept {
    return _target_quality_map;
}
[[nodiscard]] const InstanceCase::arc_probability_map_t &
InstanceCase::arc_probability_map() const noexcept {
    return _arc_probability_map;
}
[[nodiscard]] const InstanceCase::vertex_options_map_t &
InstanceCase::vertex_options_map() const noexcept {
    return _vertex_options_map;
}
[[nodiscard]] const InstanceCase::arc_options_map_t &
InstanceCase::arc_options_map() const noexcept {
    return _arc_options_map;
}

[[nodiscard]] InstanceCase::InstanceCase(
    case_id_t id, const std::string & case_name,
    fhamonic::melon::static_digraph && graph,
    std::vector<double> && source_quality_map,
    std::vector<double> && target_quality_map,
    std::vector<double> && arc_probability_map,
    std::vector<std::string> && vertex_names,
    phmap::node_hash_map<std::string, vertex_t> && vertex_name_to_id_map,
    std::vector<std::string> && arc_names,
    phmap::node_hash_map<std::string, arc_t> && arc_name_to_id_map)
    : _case_id(id)
    , _case_name(std::move(case_name))
    , _graph(std::move(graph))
    , _source_quality_map(std::move(source_quality_map))
    , _target_quality_map(std::move(target_quality_map))
    , _arc_probability_map(std::move(arc_probability_map))
    , _vertex_names(std::move(vertex_names))
    , _vertex_name_to_id_map(std::move(vertex_name_to_id_map))
    , _arc_names(std::move(arc_names))
    , _arc_name_to_id_map(std::move(arc_name_to_id_map)) {}

bool InstanceCase::contains_vertex(const std::string & name) const {
    return _vertex_name_to_id_map.contains(name);
}
bool InstanceCase::contains_arc(const std::string & name) const {
    return _arc_name_to_id_map.contains(name);
}
[[nodiscard]] InstanceCase::vertex_t InstanceCase::vertex_from_name(
    const std::string & name) const {
    if(!contains_vertex(name))
        throw std::invalid_argument("unknwon vertex id '" + name + "'");
    return _vertex_name_to_id_map.at(name);
}
[[nodiscard]] InstanceCase::arc_t InstanceCase::arc_from_name(
    const std::string & name) const {
    if(!contains_arc(name))
        throw std::invalid_argument("unknwon arc id '" + name + "'");
    return _arc_name_to_id_map.at(name);
}

[[nodiscard]] const std::string & InstanceCase::vertex_name(
    const vertex_t & v) const {
    return _vertex_names.at(v);
}
[[nodiscard]] const std::string & InstanceCase::arc_name(
    const arc_t & a) const {
    return _arc_names.at(a);
}

// Instance

[[nodiscard]] std::ranges::iota_view<gecot::option_t, gecot::option_t>
Instance::options() const noexcept {
    return std::ranges::iota_view<gecot::option_t, gecot::option_t>(
        gecot::option_t{0}, static_cast<gecot::option_t>(num_options()));
}
[[nodiscard]] double Instance::option_cost(gecot::option_t o) const {
    return _options_costs[o];
}
[[nodiscard]] const std::vector<InstanceCase> & Instance::cases()
    const noexcept {
    return _cases;
}
[[nodiscard]] const criterion_formula & Instance::criterion() const noexcept {
    return _criterion;
}
[[nodiscard]] std::size_t Instance::num_options() const noexcept {
    return _options_costs.size();
}
gecot::option_t Instance::add_option(std::string identifier,
                                     double c) noexcept {
    assert(!_option_name_to_id_map.contains(identifier));
    gecot::option_t i = static_cast<gecot::option_t>(num_options());
    _options_names.emplace_back(identifier);
    _options_costs.emplace_back(c);
    _option_name_to_id_map[identifier] = i;
    return i;
}
bool Instance::contains_option(gecot::option_t i) const noexcept {
    return i < static_cast<gecot::option_t>(num_options());
}
bool Instance::contains_option(std::string identifier) const noexcept {
    return _option_name_to_id_map.contains(identifier);
}
void Instance::set_option_cost(gecot::option_t i, double cost) noexcept {
    _options_costs[i] = cost;
}
const std::string & Instance::option_name(gecot::option_t i) const noexcept {
    return _options_names[i];
}
gecot::option_t Instance::option_from_name(const std::string & name) const {
    if(!contains_option(name))
        throw std::invalid_argument("unknwon option id '" + name + "'");
    return _option_name_to_id_map.at(name);
}
void Instance::set_criterion(criterion_formula && c) { _criterion = c; }
void Instance::set_criterion(const criterion_formula & c) { _criterion = c; }
gecot::case_id_t Instance::case_id_from_name(const std::string & name) const {
    for(auto && instance_case : _cases) {
        if(instance_case.name() == name) return instance_case.id();
    }
    throw std::invalid_argument("unknwon case '" + name + "'");
}
