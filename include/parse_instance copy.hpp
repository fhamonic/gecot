#ifndef INSTANCE2_PARSER_HPP
#define INSTANCE2_PARSER_HPP

#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>

#include <fast-cpp-csv-parser/csv.h>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>

#include "melon/utility/static_digraph_builder.hpp"

#include "instance.hpp"

namespace fhamonic {
namespace detail {

static nlohmann::json::json instance_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "type": "object",
    "properties": {
        "options": {
            "oneOf": [
                {
                    "type": "object",
                    "properties": {
                        "csv_file": {
                            "type": "string"
                        },
                        "csv_columns": {
                            "type": "object",
                            "properties": {
                                "id": {
                                    "type": "string"
                                },
                                "cost": {
                                    "type": "string"
                                }
                            },
                            "required": [
                                "id",
                                "cost"
                            ]
                        }
                    },
                    "required": [
                        "csv_file",
                        "csv_columns"
                    ],
                    "additionalProperties": false
                },
                {
                    "type": "array",
                    "contains": {
                        "type": "object",
                        "properties": {
                            "id": {
                                "type": "string"
                            },
                            "cost": {
                                "type": "number",
                                "minimum": 0
                            }
                        }
                    }
                }
            ]
        },
        "cases": {
            "type": "object",
            "patternProperties": {
                "^.*$": {
                    "type": "object",
                    "properties": {
                        "vertices": {
                            "oneOf": [
                                {
                                    "type": "object",
                                    "properties": {
                                        "csv_file": {
                                            "type": "string"
                                        },
                                        "csv_columns": {
                                            "type": "object",
                                            "properties": {
                                                "id": {
                                                    "type": "string"
                                                },
                                                "quality": {
                                                    "type": "string"
                                                }
                                            },
                                            "required": [
                                                "id",
                                                "quality"
                                            ]
                                        }
                                    },
                                    "required": [
                                        "csv_file",
                                        "csv_columns"
                                    ],
                                    "additionalProperties": false
                                },
                                {
                                    "type": "array",
                                    "contains": {
                                        "type": "object",
                                        "properties": {
                                            "id": {
                                                "type": "string"
                                            },
                                            "quality": {
                                                "type": "number",
                                                "minimum": 0
                                            }
                                        }
                                    }
                                }
                            ]
                        },
                        "arcs": {
                            "oneOf": [
                                {
                                    "type": "object",
                                    "properties": {
                                        "csv_file": {
                                            "type": "string"
                                        },
                                        "csv_columns": {
                                            "type": "object",
                                            "properties": {
                                                "id": {
                                                    "type": "string"
                                                },
                                                "from": {
                                                    "type": "string"
                                                },
                                                "to": {
                                                    "type": "string"
                                                },
                                                "probability": {
                                                    "type": "string"
                                                }
                                            },
                                            "required": [
                                                "id",
                                                "from",
                                                "to",
                                                "probability"
                                            ]
                                        }
                                    },
                                    "required": [
                                        "csv_file",
                                        "csv_columns"
                                    ],
                                    "additionalProperties": false
                                },
                                {
                                    "type": "array",
                                    "contains": {
                                        "type": "object",
                                        "properties": {
                                            "id": {
                                                "type": "string"
                                            },
                                            "from": {
                                                "type": "string"
                                            },
                                            "to": {
                                                "type": "string"
                                            },
                                            "probability": {
                                                "type": "number",
                                                "minimum": 0,
                                                "maximum": 1
                                            }
                                        }
                                    }
                                }
                            ]
                        },
                        "vertices_improvements": {
                            "oneOf": [
                                {
                                    "type": "object",
                                    "properties": {
                                        "csv_file": {
                                            "type": "string"
                                        },
                                        "csv_columns": {
                                            "type": "object",
                                            "properties": {
                                                "vertex_id": {
                                                    "type": "string"
                                                },
                                                "option_id": {
                                                    "type": "string"
                                                },
                                                "quality_gain": {
                                                    "type": "string"
                                                }
                                            },
                                            "required": [
                                                "vertex_id",
                                                "option_id",
                                                "quality_gain"
                                            ]
                                        }
                                    },
                                    "required": [
                                        "csv_file",
                                        "csv_columns"
                                    ],
                                    "additionalProperties": false
                                },
                                {
                                    "type": "array",
                                    "contains": {
                                        "type": "object",
                                        "properties": {
                                            "vertex_id": {
                                                "type": "string"
                                            },
                                            "option_id": {
                                                "type": "string"
                                            },
                                            "quality_gain": {
                                                "type": "number",
                                                "minimum": 0
                                            }
                                        }
                                    }
                                }
                            ]
                        },
                        "arcs_improvements": {
                            "oneOf": [
                                {
                                    "type": "object",
                                    "properties": {
                                        "csv_file": {
                                            "type": "string"
                                        },
                                        "csv_columns": {
                                            "type": "object",
                                            "properties": {
                                                "arc_id": {
                                                    "type": "string"
                                                },
                                                "option_id": {
                                                    "type": "string"
                                                },
                                                "improved_probability": {
                                                    "type": "string"
                                                }
                                            },
                                            "required": [
                                                "arc_id",
                                                "option_id",
                                                "improved_probability"
                                            ]
                                        }
                                    },
                                    "required": [
                                        "csv_file",
                                        "csv_columns"
                                    ],
                                    "additionalProperties": false
                                },
                                {
                                    "type": "array",
                                    "contains": {
                                        "type": "object",
                                        "properties": {
                                            "arc_id": {
                                                "type": "string"
                                            },
                                            "option_id": {
                                                "type": "string"
                                            },
                                            "improved_probability": {
                                                "type": "number",
                                                "minimum": 0,
                                                "maximum": 1
                                            }
                                        }
                                    }
                                }
                            ]
                        }
                    },
                    "required": [
                        "vertices",
                        "arcs"
                    ],
                    "anyOf": [
                        {
                            "required": [
                                "vertices_improvements"
                            ]
                        },
                        {
                            "required": [
                                "arcs_improvements"
                            ]
                        }
                    ]
                }
            },
            "additionalProperties": false
        }
    },
    "required": [
        "options",
        "cases"
    ]
}

)"_json;

template <typename T, typename I>
void parse_options(T json_object, std::filesystem::path parent_path,
                   I & instance) {
    if(json_object.is_object()) {
        std::filesystem::path options_csv_path = json_object["csv_file"];
        if(options_csv_path.is_relative())
            options_csv_path = (parent_path / options_csv_path);
        io::CSVReader<2> options_csv(options_csv_path);
        auto columns = json_object["csv_columns"];
        options_csv.read_header(io::ignore_extra_column, columns["id"],
                                columns["cost"]);

        std::string option_id;
        double option_cost;
        while(options_csv.read_row(option_id, option_cost)) {
            if(instance.contains_option(option_id))
                throw std::invalid_argument("Option identifier '" + option_id +
                                            "' appears multiple times in ");
            instance.add_option(option_id, option_cost);
        }
    } else {
        for(auto && option : json_object) {
            if(instance.contains_option(option["id"]))
                throw std::invalid_argument("Option identifier '" +
                                            option["id"] +
                                            "' appears multiple times in ");
            instance.add_option(option["id"], option["cost"]);
        }
    }
}

template <typename T>
StaticLandscape parse_landscape(T json_object,
                                std::filesystem::path parent_path) {
    std::vector<double> vertex_quality_map;
    std::vector<std::string> vertex_names;
    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        name_to_vertex_map;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        name_to_arc_map;

    auto vertices_json = json_object["vertices"];

    auto add_vertex = [&](auto && id, double quality) {
        vertex_quality_map.emplace_back(quality);
        vertex_names.emplace_back(id);
        name_to_vertex_map[id] =
            static_cast<melon::vertex_t<melon::static_digraph>>(
                vertex_names.size() - 1);
    };

    if(vertices_json.is_object()) {
        std::filesystem::path vertices_csv_path = vertices_json["csv_file"];
        if(vertices_csv_path.is_relative())
            vertices_csv_path = (parent_path / vertices_csv_path);
        io::CSVReader<2> vertices_csv(vertices_csv_path);
        auto vertices_columns = vertices_json["csv_columns"];
        vertices_csv.read_header(io::ignore_extra_column,
                                 vertices_columns["id"],
                                 vertices_columns["quality"]);
        std::string vertex_id;
        double vertex_quality;
        while(vertices_csv.read_row(vertex_id, vertex_quality)) {
            add_vertex(vertex_id, vertex_quality);
        }
    } else {
        for(auto && vertex : vertices_json) {
            add_vertex(vertex["id"], vertex["quality"].get<double>());
        }
    }

    melon::static_digraph_builder<melon::static_digraph, double, std::string>
        builder(vertex_quality_map.size());

    auto add_arc = [&](auto && id, auto && from, auto && to,
                       double probability) {
        if(probability < 0 || probability > 1)
            throw std::invalid_argument('\'' + std::to_string(probability) +
                                        "' is not a valid probability");
        builder.add_arc(name_to_vertex_map[from], name_to_vertex_map[to],
                        probability, id);
    };

    auto arcs_json = json_object["arcs"];
    if(arcs_json.is_object()) {
        std::filesystem::path arcs_csv_path = arcs_json["file"];
        if(arcs_csv_path.is_relative())
            arcs_csv_path = (parent_path / arcs_csv_path);
        io::CSVReader<4> arcs_csv(arcs_csv_path);
        auto arcs_columns = arcs_json["csv_columns"];
        arcs_csv.read_header(io::ignore_extra_column, arcs_columns["id"],
                             arcs_columns["from"], arcs_columns["to"],
                             arcs_columns["probability"]);

        std::string arc_id, from, to;
        double arc_probability;
        while(arcs_csv.read_row(arc_id, from, to, arc_probability)) {
            add_arc(arc_id, from, to, arc_probability);
        }
    } else {
        for(auto && arc : arcs_json) {
            add_arc(arc["id"], arc["from"], arc["to"],
                    arc["probability"].get<double>());
        }
    }

    auto [graph, arc_probability_map, arc_names] = builder.build();

    for(auto && a : melon::arcs(graph)) {
        name_to_arc_map[arc_names[a]] = a;
    }

    return StaticLandscape(graph, vertex_quality_map, arc_probability_map,
                           vertex_names, name_to_vertex_map, arc_names,
                           name_to_arc_map);
}

template <typename T, typename I>
std::vector<std::vector<std::pair<double, Instance::Option>>>
parse_vertex_options_if_exists(T json_object, std::filesystem::path parent_path,
                               const I & instance,
                               const StaticLandscape & landscape) {
    std::vector<std::vector<std::pair<double, Instance::Option>>>
        vertex_options(landscape.graph().nb_vertices());

    if(json_object.contains("vertex_options")) {
        auto vertex_options_json = json_object["vertex_options"];
        std::filesystem::path vertex_options_csv_path =
            vertex_options_json["file"];
        if(vertex_options_csv_path.is_relative())
            vertex_options_csv_path = (parent_path / vertex_options_csv_path);
        io::CSVReader<3> vertex_options_csv(vertex_options_csv_path);
        detail::configure_csv_reader(vertex_options_csv, vertex_options_json,
                                     "option_id", "vertex_id", "quality_gain");

        std::size_t line_no = 2;
        try {
            std::string option_id, vertex_id;
            double quality_gain;
            while(vertex_options_csv.read_row(option_id, vertex_id,
                                              quality_gain)) {
                vertex_options[landscape.vertex_from_name(vertex_id)]
                    .emplace_back(quality_gain,
                                  instance.option_from_name(option_id));
                ++line_no;
            }
        } catch(const std::invalid_argument & e) {
            throw std::invalid_argument(
                vertex_options_csv_path.filename().string() + " line " +
                std::to_string(line_no) + ": " + e.what());
        }
    }
    return vertex_options;
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
//                         {"vertices", nlohmann::detail::value_t::object},
//                         {"arcs", nlohmann::detail::value_t::object}});

//         InstanceCase & instance_case = instance.emplaceCase();

//         auto vertices_json = case_json["vertices"];
//         std::filesystem::path vertices_csv_path = vertices_json["file"];
//         if(vertices_csv_path.is_relative())
//             vertices_csv_path = (instance_path.parent_path() /
//             vertices_csv_path);
//         io::CSVReader<2> vertices_csv(vertices_csv_path);
//         detail::configure_csv_reader(vertices_csv, vertices_json,
//         "vertex_id",
//                                      "vertex_quality");

//         std::string vertex_id;
//         double vertex_quality;
//         while(vertices_csv.read_row(vertex_id, vertex_quality)) {
//             // if(instance.containsOption(vertex_id))
//             //     throw std::invalid_argument("Option identifier '" +
//             vertex_id +
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

Instance parse_instance_json(const nlohmann::json & instance_json) {
    Instance instance;

    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(detail::instance_schema);
    validator.validate(instance_json);

    auto options_json = instance_json["options"];
    parse_options(options_json, instance_path.parent_path(), instance);

    for(auto && case_json : instance_json["cases"]) {
        InstanceCase & instance_case = instance.add_case();

        instance_case.set_landscape(
            parse_landscape(case_json, instance_path.parent_path()));
        instance_case.set_vertex_options(parse_vertex_options_if_exists(
            case_json, instance_path.parent_path(), instance_case,
            instance_case.landscape()));
        instance_case.set_arc_options(
            parse_arc_options_if_exists(case_json, instance_path.parent_path(),
                                        instance_case, instance_case.landscape()));
    }

    return instance;
}

Instance parse_instance(std::filesystem::path instance_path) {
    std::ifstream instance_stream(instance_path);
    nlohmann::json instance_json;
    instance_stream >> instance_json;

    return parse_instance_json(Instance_json);
}

}  // namespace fhamonic

#endif  // INSTANCE2_PARSER_HPP