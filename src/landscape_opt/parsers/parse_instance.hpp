#ifndef INSTANCE2_PARSER_HPP
#define INSTANCE2_PARSER_HPP

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>

#include "solvers/concept/instance.hpp"

#include <fast-cpp-csv-parser/csv.h>
#include <nlohmann/json.hpp>

namespace fhamonic {
namespace landscape_opt {
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
                          Columns... default_column_names) {
    auto get_column_name = [&](std::string && c) -> std::string {
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

Instance parse_instance(std::filesystem::path instance_path) {
    Instance instance;

    std::ifstream instance_stream(instance_path);
    nlohmann::json instance_json;
    instance_stream >> instance_json;

    detail::assert_json_properties(
        instance_json, {{"options", nlohmann::detail::value_t::object},
                        {"cases", nlohmann::detail::value_t::array}});
    auto options_json = instance_json["options"];
    detail::assert_json_properties(
        options_json, {{"file", nlohmann::detail::value_t::string}});

    {
        std::filesystem::path options_csv_path = options_json["file"];
        if(options_csv_path.is_relative())
            options_csv_path = (instance_path.parent_path() / options_csv_path);
        io::CSVReader<2> options_csv(options_csv_path);
        detail::configure_csv_reader(options_csv, options_json, "option_id",
                                     "cost");

        std::string option_id;
        double option_cost;
        while(options_csv.read_row(option_id, option_cost)) {
            if(instance.containsOption(option_id))
                throw std::invalid_argument("Option identifier '" + option_id +
                                            "' appears multiple times in ");
            instance.addOption(option_id, option_cost);
        }
    }

    for(auto case_json : instance_json["cases"]) {
        detail::assert_json_properties(
            case_json, {{"name", nlohmann::detail::value_t::string},
                        {"nodes", nlohmann::detail::value_t::object},
                        {"arcs", nlohmann::detail::value_t::object}});

        InstanceCase & instance_case = instance.emplaceCase();

        auto nodes_json = case_json["nodes"];
        std::filesystem::path nodes_csv_path = nodes_json["file"];
        if(nodes_csv_path.is_relative())
            nodes_csv_path = (instance_path.parent_path() / nodes_csv_path);
        io::CSVReader<2> nodes_csv(nodes_csv_path);
        detail::configure_csv_reader(nodes_csv, nodes_json, "node_id",
                                     "node_quality");

        std::string node_id;
        double node_quality;
        while(nodes_csv.read_row(node_id, node_quality)) {
            // if(instance.containsOption(node_id))
            //     throw std::invalid_argument("Option identifier '" + node_id +
            //                                 "' appears multiple times in ");
        }

        auto arcs_json = case_json["arcs"];
        std::filesystem::path arcs_csv_path = arcs_json["file"];
        if(arcs_csv_path.is_relative())
            arcs_csv_path = (instance_path.parent_path() / arcs_csv_path);
        io::CSVReader<2> arcs_csv(arcs_csv_path);
        detail::configure_csv_reader(arcs_csv, arcs_json, "arc_id",
                                     "arc_probability");

        std::string arc_id;
        double arc_probability;
        while(arcs_csv.read_row(arc_id, arc_probability)) {
            // if(instance.containsOption(arc_id))
            //     throw std::invalid_argument("Option identifier '" + arc_id +
            //                                 "' appears multiple times in ");
        }
    }

    return instance;
}
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // INSTANCE2_PARSER_HPP