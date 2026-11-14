#ifndef TRIVIAL_REFORMULATE_HPP
#define TRIVIAL_REFORMULATE_HPP

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <variant>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "melon/algorithm/strongly_connected_components.hpp"
#include "melon/utility/static_digraph_builder.hpp"
#include "melon/views/subgraph.hpp"

#include "instance.hpp"

// the criterion of the scores divided by s: a product is homogeneous of the
// degree of its non-constant factors, so [c, f] keeps its weight c
struct formula_scale_visitor {
    double s;

    criterion_formula operator()(const criterion_constant & c) { return c / s; }
    criterion_formula operator()(const criterion_var & v) { return v; }
    criterion_formula operator()(const criterion_sum & f) {
        return criterion_sum{scaled(f.values)};
    }
    criterion_formula operator()(const criterion_product & f) {
        criterion_product product;
        double weight = 1 / s;  // exactly 1 for [c, f], s being a power of 2
        for(auto && e : f.values) {
            if(std::holds_alternative<criterion_constant>(e)) {
                product.values.push_back(e);
                continue;
            }
            product.values.push_back(std::visit(*this, e));
            weight *= s;
        }
        if(weight != 1) product.values.insert(product.values.begin(), weight);
        return product;
    }
    criterion_formula operator()(const criterion_min & f) {
        return criterion_min{scaled(f.values)};
    }
    std::vector<criterion_formula> scaled(
        const std::vector<criterion_formula> & values) {
        std::vector<criterion_formula> scaled_values;
        for(auto && e : values) scaled_values.push_back(std::visit(*this, e));
        return scaled_values;
    }
};

template <gecot::instance_c I>
std::vector<std::string> cost_dimensions_names(const I & instance) {
    std::vector<std::string> names;
    for(std::size_t r = 0; r < instance.num_cost_dimensions(); ++r)
        names.push_back(instance.cost_dimension_name(r));
    return names;
}

template <gecot::instance_c I>
std::vector<double> option_costs(const I & instance, const gecot::option_t o) {
    std::vector<double> costs;
    for(std::size_t r = 0; r < instance.num_cost_dimensions(); ++r)
        costs.push_back(instance.option_cost(o, r));
    return costs;
}

// the conflict sets on the options that option_map keeps, under their new ids;
// a set left with one option or none constrains nothing
template <gecot::instance_c I>
void add_conflict_sets(const I & instance, Instance & new_instance,
                       const auto & option_map) {
    for(std::size_t l = 0; l < instance.conflict_sets().size(); ++l) {
        std::vector<gecot::option_t> set;
        for(auto && o : instance.conflict_sets()[l])
            if(option_map[o]) set.push_back(option_map[o].value());
        if(set.size() >= 2)
            new_instance.add_conflict_set(instance.conflict_set_name(l),
                                          std::move(set));
    }
}

template <gecot::instance_c I>
struct reformulation {
    Instance instance;
    double quality_scale;
    // the options of the original instance that are always bought
    gecot::instance_solution_t<I> static_options;
};

// returns the instance with its qualities divided by the returned power of 2,
// so its scores are divided by its square. Gurobi's absolute tolerances gave
// wrong optima on scores near 1e11, and near 1e-2 on bike with its qualities
// divided by the largest: the largest case score is brought into [1e5, 4e5),
// and an instance whose cases score less is left unscaled
template <gecot::instance_c I>
reformulation<I> trivial_reformulate_instance(const I & instance,
                                              const gecot::budget_t & budget,
                                              const bool merge = true) {
    // one factor for all cases: a sum or a min criterion weighs them
    const auto base_cases_pc_num = gecot::compute_base_cases_pc_num(instance);
    double max_case_score = 0;
    for(auto && instance_case : instance.cases())
        max_case_score =
            std::max(max_case_score, base_cases_pc_num[instance_case.id()]);
    const double quality_scale = std::exp2(
        std::max(0.0, std::floor(std::log2(max_case_score / 1e5) / 2)));

    Instance reformulated_instance;
    reformulated_instance.set_criterion(
        std::visit(formula_scale_visitor{quality_scale * quality_scale},
                   instance.criterion()));

    reformulated_instance.set_cost_dimensions(cost_dimensions_names(instance));

    // an option over the budget of a dimension is never bought
    auto affordable = instance.create_option_map(true);
    for(auto && o : instance.options())
        affordable[o] = gecot::fits_alone(instance, o, budget);
    // a set of one affordable option or none constrains nothing
    auto in_conflict = instance.create_option_map(false);
    for(auto && set : instance.conflict_sets())
        if(std::ranges::count_if(set, [&](auto o) { return affordable[o]; }) >=
           2)
            for(auto && o : set) in_conflict[o] = true;

    auto reformulated_option_map =
        instance.template create_option_map<std::optional<gecot::option_t>>();
    // free options are always bought, unless they exclude others: applied to
    // the cases instead
    auto static_options = instance.create_option_map(false);
    for(auto && original_option : instance.options()) {
        if(!affordable[original_option]) continue;
        const auto costs = option_costs(instance, original_option);
        static_options[original_option] =
            !in_conflict[original_option] &&
            std::ranges::all_of(costs, [](double c) { return c == 0; });
        if(static_options[original_option]) continue;
        auto option = reformulated_instance.add_option(
            instance.option_name(original_option), costs);
        reformulated_option_map[original_option].emplace(option);
    }
    add_conflict_sets(instance, reformulated_instance, reformulated_option_map);

    for(auto && instance_case : instance.cases()) {
        trivial_reformulate_case(instance_case, reformulated_instance,
                                 reformulated_option_map, static_options,
                                 quality_scale, merge);
    }

    return {std::move(reformulated_instance), quality_scale,
            std::move(static_options)};
}

// the instance without the options that keep rejects, the others under their
// names
template <gecot::instance_c I>
Instance restrict_instance(const I & instance, const auto & keep,
                           const bool merge = true) {
    Instance restricted_instance;
    restricted_instance.set_criterion(instance.criterion());
    restricted_instance.set_cost_dimensions(cost_dimensions_names(instance));
    auto restricted_option_map =
        instance.template create_option_map<std::optional<gecot::option_t>>();
    for(auto && option : instance.options())
        if(keep(option))
            restricted_option_map[option].emplace(
                restricted_instance.add_option(instance.option_name(option),
                                               option_costs(instance, option)));
    add_conflict_sets(instance, restricted_instance, restricted_option_map);
    for(auto && instance_case : instance.cases())
        trivial_reformulate_case(instance_case, restricted_instance,
                                 restricted_option_map,
                                 instance.create_option_map(false), 1.0, merge);
    return restricted_instance;
}

template <gecot::case_c C>
void trivial_reformulate_case(const C & instance_case, Instance & instance,
                              auto && reformulated_option_map,
                              const auto & free_options,
                              const double quality_scale, const bool merge) {
    using graph_t = gecot::case_graph_t<C>;
    using vertex_t = melon::vertex_t<graph_t>;
    using arc_t = melon::arc_t<graph_t>;

    const auto & original_graph = instance_case.graph();
    auto original_source_quality_map = instance_case.source_quality_map();
    auto original_target_quality_map = instance_case.target_quality_map();
    auto original_probability_map = instance_case.arc_probability_map();

    const auto & original_vertex_options_map =
        instance_case.vertex_options_map();
    const auto & original_arc_options_map = instance_case.arc_options_map();

    // before the contraction, as free options can create probability 1 arcs
    for(const vertex_t & v : melon::vertices(original_graph)) {
        for(const auto & [source_quality_gain, target_quality_gain, option] :
            original_vertex_options_map[v]) {
            if(!free_options[option]) continue;
            original_source_quality_map[v] += source_quality_gain;
            original_target_quality_map[v] += target_quality_gain;
        }
    }
    for(const arc_t & a : melon::arcs(original_graph)) {
        for(const auto & [improved_prob, option] :
            original_arc_options_map[a]) {
            if(!free_options[option]) continue;
            original_probability_map[a] =
                std::max(original_probability_map[a], improved_prob);
        }
    }

    std::vector<double> source_quality_map;
    std::vector<double> target_quality_map;
    std::vector<std::string> vertex_names_map;
    std::vector<std::vector<vertex_t>> components;
    auto component_num_map =
        melon::create_vertex_map<std::size_t>(original_graph);

    for(auto component :
        melon::strongly_connected_components(melon::views::subgraph(
            original_graph, {},
            [&original_probability_map, merge](const arc_t & a) {
                return merge && original_probability_map[a] == 1;
            }))) {
        source_quality_map.push_back(0.0);
        target_quality_map.push_back(0.0);
        vertex_names_map.emplace_back();
        const std::size_t component_num = components.size();
        components.emplace_back();
        for(const vertex_t & v : component) {
            source_quality_map.back() +=
                original_source_quality_map[v] / quality_scale;
            target_quality_map.back() +=
                original_target_quality_map[v] / quality_scale;
            if(!vertex_names_map.back().empty())
                vertex_names_map.back().append("+");
            vertex_names_map.back().append(instance_case.vertex_name(v));
            components.back().push_back(v);
            component_num_map[v] = component_num;
        }
    }

    std::size_t num_components = components.size();
    melon::static_digraph_builder<melon::static_digraph, double, std::string,
                                  arc_t>
        builder(num_components);
    for(std::size_t i = 0; i < num_components; ++i) {
        for(const vertex_t & v : components[i]) {
            for(const arc_t & a : melon::out_arcs(original_graph, v)) {
                const vertex_t & w = melon::arc_target(original_graph, a);
                if(i == component_num_map[w]) continue;
                builder.add_arc({static_cast<vertex_t>(i),
                                 static_cast<vertex_t>(component_num_map[w])},
                                original_probability_map[a],
                                instance_case.arc_name(a), a);
            }
        }
    }

    auto [graph, arc_probability_map, arc_names, original_arc_map] =
        builder.build();

    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        vertex_name_to_id_map;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        arc_name_to_id_map;

    for(vertex_t v : melon::vertices(graph))
        vertex_name_to_id_map[vertex_names_map[v]] = v;
    for(arc_t a : melon::arcs(graph)) arc_name_to_id_map[arc_names[a]] = a;

    auto vertex_options = melon::create_vertex_map<
        std::vector<std::tuple<double, double, gecot::option_t>>>(graph, {});
    for(vertex_t v : melon::vertices(graph)) {
        for(vertex_t original_v : components[v]) {
            for(const auto & [source_quality_gain, target_quality_gain,
                              original_option] :
                original_vertex_options_map[original_v]) {
                if(!reformulated_option_map[original_option].has_value())
                    continue;
                vertex_options[v].emplace_back(
                    source_quality_gain / quality_scale,
                    target_quality_gain / quality_scale,
                    reformulated_option_map[original_option].value());
            }
        }
    }
    auto arc_options =
        melon::create_arc_map<std::vector<std::pair<double, gecot::option_t>>>(
            graph, {});
    for(arc_t a : melon::arcs(graph)) {
        for(const auto & [improved_prob, original_option] :
            original_arc_options_map[original_arc_map[a]]) {
            if(!reformulated_option_map[original_option].has_value()) continue;
            arc_options[a].emplace_back(
                improved_prob,
                reformulated_option_map[original_option].value());
        }
    }

    auto & reformulated_case = instance.emplace_case(
        instance_case.name(), std::move(graph), std::move(source_quality_map),
        std::move(target_quality_map), std::move(arc_probability_map),
        std::move(vertex_names_map), std::move(vertex_name_to_id_map),
        std::move(arc_names), std::move(arc_name_to_id_map));

    reformulated_case.set_vertex_options_map(std::move(vertex_options));
    reformulated_case.set_arc_options_map(std::move(arc_options));
}

#endif  // TRIVIAL_REFORMULATE_HPP