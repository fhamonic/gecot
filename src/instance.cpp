#include "instance.hpp"

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
    melon::static_digraph && graph,
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
        throw std::invalid_argument("unknown vertex id '" + name + "'");
    return _vertex_name_to_id_map.at(name);
}
[[nodiscard]] InstanceCase::arc_t InstanceCase::arc_from_name(
    const std::string & name) const {
    if(!contains_arc(name))
        throw std::invalid_argument("unknown arc id '" + name + "'");
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
    if(num_cost_dimensions() != 1)
        throw std::logic_error("the options have several costs");
    return _options_costs[o];
}
[[nodiscard]] double Instance::option_cost(gecot::option_t o,
                                           std::size_t r) const {
    return _options_costs[o * num_cost_dimensions() + r];
}
[[nodiscard]] std::size_t Instance::num_cost_dimensions() const noexcept {
    return _cost_dimensions_names.size();
}
[[nodiscard]] const std::string & Instance::cost_dimension_name(
    std::size_t r) const {
    return _cost_dimensions_names.at(r);
}
[[nodiscard]] const std::vector<std::vector<gecot::option_t>> &
Instance::conflict_sets() const noexcept {
    return _conflict_sets;
}
[[nodiscard]] const std::string & Instance::conflict_set_name(
    std::size_t l) const {
    return _conflict_sets_names.at(l);
}
[[nodiscard]] const std::vector<InstanceCase> & Instance::cases()
    const noexcept {
    return _cases;
}
[[nodiscard]] const criterion_formula & Instance::criterion() const noexcept {
    return _criterion;
}
[[nodiscard]] std::size_t Instance::num_options() const noexcept {
    return _options_names.size();
}
void Instance::set_cost_dimensions(std::vector<std::string> names) {
    assert(num_options() == 0 && !names.empty());
    _cost_dimensions_names = std::move(names);
}
gecot::option_t Instance::add_option(std::string identifier,
                                     const std::vector<double> & costs) {
    assert(!_option_name_to_id_map.contains(identifier));
    if(costs.size() != num_cost_dimensions())
        throw std::invalid_argument(
            "option '" + identifier + "' has " + std::to_string(costs.size()) +
            " costs instead of " + std::to_string(num_cost_dimensions()));
    gecot::option_t i = static_cast<gecot::option_t>(num_options());
    _options_names.emplace_back(identifier);
    _options_costs.insert(_options_costs.end(), costs.begin(), costs.end());
    _option_name_to_id_map[identifier] = i;
    return i;
}
void Instance::add_conflict_set(std::string name,
                                std::vector<gecot::option_t> options) {
    _conflict_sets_names.emplace_back(std::move(name));
    _conflict_sets.emplace_back(std::move(options));
}
bool Instance::contains_option(gecot::option_t i) const noexcept {
    return i < static_cast<gecot::option_t>(num_options());
}
bool Instance::contains_option(std::string identifier) const noexcept {
    return _option_name_to_id_map.contains(identifier);
}
const std::string & Instance::option_name(gecot::option_t i) const noexcept {
    return _options_names[i];
}
gecot::option_t Instance::option_from_name(const std::string & name) const {
    if(!contains_option(name))
        throw std::invalid_argument("unknown option id '" + name + "'");
    return _option_name_to_id_map.at(name);
}
void Instance::set_criterion(criterion_formula && c) { _criterion = c; }
void Instance::set_criterion(const criterion_formula & c) { _criterion = c; }
gecot::case_id_t Instance::case_id_from_name(const std::string & name) const {
    for(auto && instance_case : _cases) {
        if(instance_case.name() == name) return instance_case.id();
    }
    throw std::invalid_argument("unknown case '" + name + "'");
}
