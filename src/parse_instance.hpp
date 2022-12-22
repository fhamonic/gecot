#ifndef INSTANCE2_PARSER_HPP
#define INSTANCE2_PARSER_HPP

#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>

#include <fast-cpp-csv-parser/csv.h>
#include <nlohmann/json.hpp>

#include "melon/static_digraph_builder.hpp"

#include "instance.hpp"

namespace fhamonic {
namespace detail {

std::string json_type_str(nlohmann::detail::value_t type) {
    switch(type) {
        case nlohmann::detail::value_t::null:
            return "null";
        case nlohmann::detail::value_t::object:
            return "object";
        case nlohmann::detail::value_t::array:
            return "array";
        case nlohmann::detail::value_t::string:
            return "string";
        case nlohmann::detail::value_t::boolean:
            return "boolean";
        case nlohmann::detail::value_t::number_integer:
            return "number_integer";
        case nlohmann::detail::value_t::number_unsigned:
            return "number_unsigned";
        case nlohmann::detail::value_t::number_float:
            return "number_float";
        case nlohmann::detail::value_t::binary:
            return "binary";
        default:
            throw "unknowned";
    }
}

template <typename T>
void assert_json_properties(
    T json_object,
    std::initializer_list<std::pair<std::string, nlohmann::detail::value_t>>
        properties) {
    for(const auto & [property, type] : properties) {
        if(!json_object.contains(property))
            throw std::invalid_argument("No '" + property + "' property");
        const auto json_property = json_object[property];
        if(json_property.type() != type)
            throw std::invalid_argument(
                "Property '" + property + "' expected of type '" +
                json_type_str(type) + "' but is '" +
                json_type_str(json_property.type()) + "'");
    }
}

template <typename C, typename T, typename... Columns>
void configure_csv_reader(C & csv, T json_object,
                          Columns &&... default_column_names) {
    auto get_column_name = [&](const std::string & c) -> std::string {
        if(json_object.contains("columns")) {
            auto columns_json = json_object["columns"];
            if(columns_json.contains(c)) return columns_json[c];
        }
        return c;
    };
    csv.read_header(io::ignore_extra_column,
                    get_column_name(default_column_names)...);
}
}  // namespace detail

template <typename T, typename I>
void parse_options(T json_object, std::filesystem::path parent_path,
                   I & instance) {
    detail::assert_json_properties(
        json_object, {{"file", nlohmann::detail::value_t::string}});

    {
        std::filesystem::path options_csv_path = json_object["file"];
        if(options_csv_path.is_relative())
            options_csv_path = (parent_path / options_csv_path);
        io::CSVReader<2> options_csv(options_csv_path);
        detail::configure_csv_reader(options_csv, json_object, "option_id",
                                     "cost");

        std::string option_id;
        double option_cost;
        while(options_csv.read_row(option_id, option_cost)) {
            if(instance.contains_option(option_id))
                throw std::invalid_argument("Option identifier '" + option_id +
                                            "' appears multiple times in ");
            instance.add_option(option_id, option_cost);
        }
    }
}

template <typename T>
StaticLandscape parse_landscape(T json_object,
                                std::filesystem::path parent_path) {
    std::vector<double> node_quality_map;

    std::vector<std::string> node_names;
    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        name_to_vertex_map;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        name_to_arc_map;

    detail::assert_json_properties(
        json_object, {{"name", nlohmann::detail::value_t::string},
                      {"nodes", nlohmann::detail::value_t::object},
                      {"arcs", nlohmann::detail::value_t::object}});

    auto nodes_json = json_object["nodes"];
    std::filesystem::path nodes_csv_path = nodes_json["file"];
    if(nodes_csv_path.is_relative())
        nodes_csv_path = (parent_path / nodes_csv_path);
    io::CSVReader<2> nodes_csv(nodes_csv_path);
    detail::configure_csv_reader(nodes_csv, nodes_json, "id", "quality");

    std::string node_id;
    double node_quality;
    while(nodes_csv.read_row(node_id, node_quality)) {
        node_quality_map.emplace_back(node_quality);
        node_names.emplace_back(node_id);
        name_to_vertex_map[node_id] = node_names.size() - 1;
    }

    melon::static_digraph_builder<melon::static_digraph, double, std::string>
        builder(node_quality_map.size());

    auto arcs_json = json_object["arcs"];
    std::filesystem::path arcs_csv_path = arcs_json["file"];
    if(arcs_csv_path.is_relative())
        arcs_csv_path = (parent_path / arcs_csv_path);
    io::CSVReader<4> arcs_csv(arcs_csv_path);
    detail::configure_csv_reader(arcs_csv, arcs_json, "id", "from", "to",
                                 "probability");

    std::string arc_id, from, to;
    double arc_probability;
    while(arcs_csv.read_row(arc_id, from, to, arc_probability)) {
        builder.add_arc(name_to_vertex_map[from], name_to_vertex_map[to],
                        arc_probability, arc_id);
    }

    auto [graph, arc_probability_map, arc_names] = builder.build();

    for(auto && a : graph.arcs()) {
        name_to_arc_map[arc_names[a]] = a;
    }

    return StaticLandscape(graph, node_quality_map, arc_probability_map,
                           node_names, name_to_vertex_map, arc_names,
                           name_to_arc_map);
}

template <typename T, typename I>
std::vector<std::vector<std::pair<double, Instance::Option>>>
parse_node_options_if_exists(T json_object, std::filesystem::path parent_path,
                             const I & instance,
                             const StaticLandscape & landscape) {
    std::vector<std::vector<std::pair<double, Instance::Option>>> node_options(
        landscape.graph().nb_vertices());

    if(json_object.contains("node_options")) {
        auto node_options_json = json_object["node_options"];
        std::filesystem::path node_options_csv_path = node_options_json["file"];
        if(node_options_csv_path.is_relative())
            node_options_csv_path = (parent_path / node_options_csv_path);
        io::CSVReader<3> node_options_csv(node_options_csv_path);
        detail::configure_csv_reader(node_options_csv, node_options_json,
                                     "option_id", "node_id", "quality_gain");

        std::size_t line_no = 2;
        try {
            std::string option_id, node_id;
            double quality_gain;
            while(node_options_csv.read_row(option_id, node_id, quality_gain)) {
                node_options[landscape.vertex_from_name(node_id)].emplace_back(
                    quality_gain, instance.option_from_name(option_id));
                ++line_no;
            }
        } catch(const std::invalid_argument & e) {
            throw std::invalid_argument(
                node_options_csv_path.filename().string() + " line " +
                std::to_string(line_no) + ": " + e.what());
        }
    }
    return node_options;
}

template <typename T, typename I>
std::vector<std::vector<std::pair<double, Instance::Option>>>
parse_arc_options_if_exists(T json_object, std::filesystem::path parent_path,
                            const I & instance,
                            const StaticLandscape & landscape) {
    std::vector<std::vector<std::pair<double, Instance::Option>>> arc_options(
        landscape.graph().nb_arcs());

    if(json_object.contains("arc_options")) {
        auto arc_options_json = json_object["arc_options"];
        std::filesystem::path arc_options_csv_path = arc_options_json["file"];
        if(arc_options_csv_path.is_relative())
            arc_options_csv_path = (parent_path / arc_options_csv_path);
        io::CSVReader<3> arc_options_csv(arc_options_csv_path);
        detail::configure_csv_reader(arc_options_csv, arc_options_json,
                                     "option_id", "arc_id",
                                     "improved_probability");

        std::size_t line_no = 2;
        try {
            std::string option_id, arc_id;
            double improved_probability;
            while(arc_options_csv.read_row(option_id, arc_id,
                                           improved_probability)) {
                arc_options[landscape.arc_from_name(arc_id)].emplace_back(
                    improved_probability, instance.option_from_name(option_id));
                ++line_no;
            }
        } catch(const std::invalid_argument & e) {
            throw std::invalid_argument(
                arc_options_csv_path.filename().string() + " line " +
                std::to_string(line_no) + ": " + e.what());
        }
    }

    return arc_options;
}

// Instance parse_multi_instance(std::filesystem::path instance_path) {
//     Instance instance;

//     std::ifstream instance_stream(instance_path);
//     nlohmann::json instance_json;
//     instance_stream >> instance_json;

//     detail::assert_json_properties(
//         instance_json, {{"options", nlohmann::detail::value_t::object},
//                         {"cases", nlohmann::detail::value_t::array}});
//     auto options_json = instance_json["options"];
//     detail::assert_json_properties(
//         options_json, {{"file", nlohmann::detail::value_t::string}});

//     {
//         std::filesystem::path options_csv_path = options_json["file"];
//         if(options_csv_path.is_relative())
//             options_csv_path = (instance_path.parent_path() /
//             options_csv_path);
//         io::CSVReader<2> options_csv(options_csv_path);
//         detail::configure_csv_reader(options_csv, options_json, "option_id",
//                                      "cost");

//         std::string option_id;
//         double option_cost;
//         while(options_csv.read_row(option_id, option_cost)) {
//             if(instance.containsOption(option_id))
//                 throw std::invalid_argument("Option identifier '" + option_id
//                 +
//                                             "' appears multiple times in ");
//             instance.addOption(option_id, option_cost);
//         }
//     }

//     for(auto case_json : instance_json["cases"]) {
//         detail::assert_json_properties(
//             case_json, {{"name", nlohmann::detail::value_t::string},
//                         {"nodes", nlohmann::detail::value_t::object},
//                         {"arcs", nlohmann::detail::value_t::object}});

//         InstanceCase & instance_case = instance.emplaceCase();

//         auto nodes_json = case_json["nodes"];
//         std::filesystem::path nodes_csv_path = nodes_json["file"];
//         if(nodes_csv_path.is_relative())
//             nodes_csv_path = (instance_path.parent_path() / nodes_csv_path);
//         io::CSVReader<2> nodes_csv(nodes_csv_path);
//         detail::configure_csv_reader(nodes_csv, nodes_json, "node_id",
//                                      "node_quality");

//         std::string node_id;
//         double node_quality;
//         while(nodes_csv.read_row(node_id, node_quality)) {
//             // if(instance.containsOption(node_id))
//             //     throw std::invalid_argument("Option identifier '" +
//             node_id +
//             //                                 "' appears multiple times in
//             ");
//         }

//         auto arcs_json = case_json["arcs"];
//         std::filesystem::path arcs_csv_path = arcs_json["file"];
//         if(arcs_csv_path.is_relative())
//             arcs_csv_path = (instance_path.parent_path() / arcs_csv_path);
//         io::CSVReader<2> arcs_csv(arcs_csv_path);
//         detail::configure_csv_reader(arcs_csv, arcs_json, "arc_id",
//                                      "arc_probability");

//         std::string arc_id;
//         double arc_probability;
//         while(arcs_csv.read_row(arc_id, arc_probability)) {
//             // if(instance.containsOption(arc_id))
//             //     throw std::invalid_argument("Option identifier '" + arc_id
//             +
//             //                                 "' appears multiple times in
//             ");
//         }
//     }

//     return instance;
// }

Instance parse_instance(std::filesystem::path instance_path) {
    Instance instance;

    std::ifstream instance_stream(instance_path);
    nlohmann::json instance_json;
    instance_stream >> instance_json;

    detail::assert_json_properties(
        instance_json, {{"options", nlohmann::detail::value_t::object},
                        {"case", nlohmann::detail::value_t::object}});
    auto options_json = instance_json["options"];
    parse_options(options_json, instance_path.parent_path(), instance);

    auto case_json = instance_json["case"];
    detail::assert_json_properties(
        case_json, {{"nodes", nlohmann::detail::value_t::object},
                    {"arcs", nlohmann::detail::value_t::object}});
    instance.set_landscape(
        parse_landscape(case_json, instance_path.parent_path()));

    instance.set_node_options(
        parse_node_options_if_exists(case_json, instance_path.parent_path(),
                                     instance, instance.landscape()));

    instance.set_arc_options(
        parse_arc_options_if_exists(case_json, instance_path.parent_path(),
                                    instance, instance.landscape()));

    return instance;
}

}  // namespace fhamonic

#endif  // INSTANCE2_PARSER_HPP