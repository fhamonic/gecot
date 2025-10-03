#include "parse_instance.hpp"

namespace fhamonic {

static const nlohmann::json instance_schema = R"(
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
                            "additionalProperties": false
                        }
                    },
                    "required": [
                        "csv_file"
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
                                                "source_quality": {
                                                    "type": "string"
                                                },
                                                "target_quality": {
                                                    "type": "string"
                                                }
                                            },
                                            "additionalProperties": false
                                        }
                                    },
                                    "required": [
                                        "csv_file"
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
                                            "source_quality": {
                                                "type": "number",
                                                "minimum": 0
                                            },
                                            "target_quality": {
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
                                            "additionalProperties": false
                                        }
                                    },
                                    "required": [
                                        "csv_file"
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
                                                "source_quality_gain": {
                                                    "type": "string"
                                                },
                                                "target_quality_gain": {
                                                    "type": "string"
                                                }
                                            },
                                            "additionalProperties": false
                                        }
                                    },
                                    "required": [
                                        "csv_file"
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
                                            "source_quality_gain": {
                                                "type": "number",
                                                "minimum": 0
                                            },
                                            "target_quality_gain": {
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
                                            "additionalProperties": false
                                        }
                                    },
                                    "required": [
                                        "csv_file"
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
        },
        "criterion": {
            "$ref": "#/properties/criterion/definitions/formula",
            "definitions": {
                "constant": {
                    "type": "number"
                },
                "variable": {
                    "type": "string"
                },
                "linear_term": {
                    "type": "array",
                    "items": [
                        {
                            "$ref": "#/properties/criterion/definitions/constant"
                        },
                        {
                            "$ref": "#/properties/criterion/definitions/formula"
                        }
                    ],
                    "additionalItems": false
                },
                "operation": {
                    "type": "object",
                    "properties": {
                        "operation": {
                            "enum": [
                                "sum",
                                "product",
                                "min"
                            ]
                        },
                        "values": {
                            "type": "array",
                            "contains": {
                                "$ref": "#/properties/criterion/definitions/formula"
                            }
                        }
                    },
                    "required": [
                        "operation",
                        "values"
                    ],
                    "additionalProperties": false
                },
                "formula": {
                    "oneOf": [
                        {
                            "$ref": "#/properties/criterion/definitions/constant"
                        },
                        {
                            "$ref": "#/properties/criterion/definitions/variable"
                        },
                        {
                            "$ref": "#/properties/criterion/definitions/linear_term"
                        },
                        {
                            "$ref": "#/properties/criterion/definitions/operation"
                        }
                    ]
                }
            }
        }
    },
    "required": [
        "options",
        "cases"
    ],
    "additionalProperties": false
}
)"_json;

void parse_columns_aliases(
    const nlohmann::json & json_object,
    std::initializer_list<std::reference_wrapper<std::string>> column_names) {
    if(!json_object.contains("csv_columns")) return;
    auto columns = json_object["csv_columns"];
    for(auto & s : column_names) {
        if(columns.contains(s)) {
            s.get() = columns[s];
        }
    }
}

void parse_options(Instance & instance, const nlohmann::json & json_object,
                   const std::filesystem::path & parent_path) {
    auto add_option = [&](const std::string & option_id,
                          const double option_cost) {
        if(instance.contains_option(option_id))
            throw std::invalid_argument("Option identifier '" + option_id +
                                        "' appears multiple times in ");
        instance.add_option(option_id, option_cost);
    };

    if(json_object.is_object()) {
        std::filesystem::path options_csv_path = json_object["csv_file"];
        if(options_csv_path.is_relative())
            options_csv_path = (parent_path / options_csv_path);
        std::string id_column = "id";
        std::string cost_column = "cost";
        parse_columns_aliases(json_object, {id_column, cost_column});

        csv::CSVReader reader(options_csv_path.string());
        for(auto & row : reader) {
            add_option(row[id_column].get<std::string>(),
                       row[cost_column].get<double>());
        }
    } else {
        for(auto && option : json_object) {
            add_option(option["id"], option["cost"]);
        }
    }
}

void parse_vertices_options(InstanceCase & instance_case,
                            const nlohmann::json & json_object,
                            const std::filesystem::path & parent_path,
                            const Instance & instance) {
    auto vertex_options = melon::create_vertex_map<
        std::vector<std::tuple<double, double, gecot::option_t>>>(
        instance_case.graph(), {});

    auto add_vertex_option = [&](const std::string & option_id,
                                 const std::string & vertex_id,
                                 const double source_quality_gain,
                                 const double target_quality_gain) {
        vertex_options[instance_case.vertex_from_name(vertex_id)].emplace_back(
            source_quality_gain, target_quality_gain,
            instance.option_from_name(option_id));
    };

    if(json_object.contains("vertices_improvements")) {
        auto vertex_options_json = json_object["vertices_improvements"];
        if(vertex_options_json.is_object()) {
            std::filesystem::path vertex_options_csv_path =
                vertex_options_json["csv_file"];
            if(vertex_options_csv_path.is_relative())
                vertex_options_csv_path =
                    (parent_path / vertex_options_csv_path);
            std::string option_id_column = "option_id";
            std::string vertex_id_column = "vertex_id";
            std::string source_quality_gain_column = "source_quality_gain";
            std::string target_quality_gain_column = "target_quality_gain";
            parse_columns_aliases(
                vertex_options_json,
                {option_id_column, vertex_id_column, source_quality_gain_column,
                 target_quality_gain_column});

            csv::CSVReader reader(vertex_options_csv_path.string());
            std::size_t line_no = 2;
            try {
                for(auto & row : reader) {
                    add_vertex_option(
                        row[option_id_column].get<std::string>(),
                        row[vertex_id_column].get<std::string>(),
                        row[source_quality_gain_column].get<double>(),
                        row[target_quality_gain_column].get<double>());
                    ++line_no;
                }
            } catch(const std::invalid_argument & e) {
                throw std::invalid_argument(
                    vertex_options_csv_path.filename().string() + " line " +
                    std::to_string(line_no) + ": " + e.what());
            }
        } else {
            for(auto && vertex_option : vertex_options_json) {
                add_vertex_option(vertex_option["option_id"],
                                  vertex_option["vertex_id"],
                                  vertex_option["source_quality_gain"],
                                  vertex_option["target_quality_gain"]);
            }
        }
    }
    instance_case.set_vertex_options_map(std::move(vertex_options));
}

void parse_arcs_options(InstanceCase & instance_case,
                        const nlohmann::json & json_object,
                        const std::filesystem::path & parent_path,
                        const Instance & instance) {
    auto arc_options =
        melon::create_arc_map<std::vector<std::pair<double, gecot::option_t>>>(
            instance_case.graph(), {});

    auto add_arc_option = [&](const std::string & option_id,
                              const std::string & arc_id,
                              const double improved_probability) {
        arc_options[instance_case.arc_from_name(arc_id)].emplace_back(
            improved_probability, instance.option_from_name(option_id));
    };

    if(json_object.contains("arcs_improvements")) {
        auto arc_options_json = json_object["arcs_improvements"];
        if(arc_options_json.is_object()) {
            std::filesystem::path arc_options_csv_path =
                arc_options_json["csv_file"];
            if(arc_options_csv_path.is_relative())
                arc_options_csv_path = (parent_path / arc_options_csv_path);
            std::string option_id_column = "option_id";
            std::string arc_id_column = "arc_id";
            std::string improved_probability_column = "improved_probability";
            parse_columns_aliases(
                arc_options_json,
                {option_id_column, arc_id_column, improved_probability_column});

            csv::CSVReader reader(arc_options_csv_path.string());
            std::size_t line_no = 2;
            try {
                for(auto & row : reader) {
                    add_arc_option(
                        row[option_id_column].get<std::string>(),
                        row[arc_id_column].get<std::string>(),
                        row[improved_probability_column].get<double>());
                    ++line_no;
                }
            } catch(const std::invalid_argument & e) {
                throw std::invalid_argument(
                    arc_options_csv_path.filename().string() + " line " +
                    std::to_string(line_no) + ": " + e.what());
            }
        } else {
            for(auto && arc_option : arc_options_json) {
                add_arc_option(arc_option["option_id"], arc_option["arc_id"],
                               arc_option["improved_probability"]);
            }
        }
    }
    instance_case.set_arc_options_map(std::move(arc_options));
}

InstanceCase & parse_instance_case(const nlohmann::json & json_object,
                                   std::string case_name,
                                   const std::filesystem::path & parent_path,
                                   Instance & instance) {
    std::vector<double> source_quality_map;
    std::vector<double> target_quality_map;
    std::vector<std::string> vertex_names;
    phmap::node_hash_map<std::string, melon::vertex_t<melon::static_digraph>>
        vertex_name_to_id_map;
    phmap::node_hash_map<std::string, melon::arc_t<melon::static_digraph>>
        arc_name_to_id_map;

    auto vertices_json = json_object["vertices"];

    auto add_vertex = [&](const std::string & id, double source_quality,
                          double target_quality) {
        source_quality_map.emplace_back(source_quality);
        target_quality_map.emplace_back(target_quality);
        vertex_names.emplace_back(id);
        vertex_name_to_id_map[id] =
            static_cast<melon::vertex_t<melon::static_digraph>>(
                vertex_names.size() - 1);
    };

    if(vertices_json.is_object()) {
        std::filesystem::path vertices_csv_path = vertices_json["csv_file"];
        if(vertices_csv_path.is_relative())
            vertices_csv_path = (parent_path / vertices_csv_path);
        std::string id_column = "id";
        std::string source_quality_column = "source_quality";
        std::string target_quality_column = "target_quality";
        parse_columns_aliases(vertices_json, {id_column, source_quality_column,
                                              target_quality_column});

        csv::CSVReader reader(vertices_csv_path.string());
        std::size_t line_no = 2;
        try {
            for(auto & row : reader) {
                add_vertex(row[id_column].get<std::string>(),
                           row[source_quality_column].get<double>(),
                           row[target_quality_column].get<double>());
                ++line_no;
            }
        } catch(const std::runtime_error & e) {
            throw std::invalid_argument(vertices_csv_path.filename().string() +
                                        " line " + std::to_string(line_no) +
                                        ": " + e.what());
        }
    } else {
        for(auto && vertex : vertices_json) {
            add_vertex(vertex["id"], vertex["source_quality"],
                       vertex["target_quality"]);
        }
    }

    melon::static_digraph_builder<melon::static_digraph, double, std::string>
        builder(source_quality_map.size());

    auto add_arc = [&](const std::string & id, const std::string & from,
                       const std::string & to, double probability) {
        if(probability < 0 || probability > 1)
            throw std::invalid_argument('\'' + std::to_string(probability) +
                                        "' is not a valid probability");
        builder.add_arc(vertex_name_to_id_map[from], vertex_name_to_id_map[to],
                        probability, id);
    };

    auto arcs_json = json_object["arcs"];
    if(arcs_json.is_object()) {
        std::filesystem::path arcs_csv_path = arcs_json["csv_file"];
        if(arcs_csv_path.is_relative())
            arcs_csv_path = (parent_path / arcs_csv_path);
        std::string id_column = "id";
        std::string from_column = "from";
        std::string to_column = "to";
        std::string probability_column = "probability";
        parse_columns_aliases(
            arcs_json, {id_column, from_column, to_column, probability_column});

        csv::CSVReader reader(arcs_csv_path.string());
        std::size_t line_no = 2;
        try {
            for(auto & row : reader) {
                add_arc(row[id_column].get<std::string>(),
                        row[from_column].get<std::string>(),
                        row[to_column].get<std::string>(),
                        row[probability_column].get<double>());
                ++line_no;
            }
        } catch(const std::runtime_error & e) {
            throw std::invalid_argument(arcs_csv_path.filename().string() +
                                        " line " + std::to_string(line_no) +
                                        ": " + e.what());
        }
    } else {
        for(auto && arc : arcs_json) {
            add_arc(arc["id"], arc["from"], arc["to"], arc["probability"]);
        }
    }

    auto [graph, arc_probability_map, arc_names] = builder.build();

    for(auto && a : melon::arcs(graph)) {
        arc_name_to_id_map[arc_names[a]] = a;
    }

    return instance.emplace_case(
        std::move(case_name), std::move(graph), std::move(source_quality_map),
        std::move(target_quality_map), std::move(arc_probability_map),
        std::move(vertex_names), std::move(vertex_name_to_id_map),
        std::move(arc_names), std::move(arc_name_to_id_map));
}

criterion_formula parse_formula(Instance & instance,
                                const nlohmann::json & json_object) {
    if(json_object.is_number()) {
        return criterion_constant{json_object.template get<double>()};
    } else if(json_object.is_string()) {
        return criterion_var{instance.case_id_from_name(
            json_object.template get<std::string>())};
    } else if(json_object.is_array()) {
        return criterion_product{{json_object[0].template get<double>(),
                                  parse_formula(instance, json_object[1])}};
    } else if(json_object.is_object()) {
        auto operation_type =
            json_object["operation"].template get<std::string>();
        std::vector<criterion_formula> values;
        for(auto && value_json : json_object["values"]) {
            values.emplace_back(parse_formula(instance, value_json));
        }
        if(operation_type == "sum") return criterion_sum{std::move(values)};
        if(operation_type == "product")
            return criterion_product{std::move(values)};
        if(operation_type == "min") return criterion_min{std::move(values)};
    }
    throw std::invalid_argument(
        "Unexpected criterion format : Update the JSON schema !");
}

Instance parse_instance_json(const nlohmann::json & instance_json,
                             const std::filesystem::path & relative_path) {
    Instance instance;

    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(instance_schema);
    validator.validate(instance_json);

    auto options_json = instance_json["options"];
    parse_options(instance, options_json, relative_path);

    for(auto && [case_name, case_json] : instance_json["cases"].items()) {
        InstanceCase & instance_case =
            parse_instance_case(case_json, case_name, relative_path, instance);
        parse_vertices_options(instance_case, case_json, relative_path,
                               instance);
        parse_arcs_options(instance_case, case_json, relative_path, instance);
    }

    if(instance_json.contains("criterion")) {
        instance.set_criterion(
            parse_formula(instance, instance_json["criterion"]));
    } else {
        criterion_sum s;
        for(auto instance_case : instance.cases())
            s.values.emplace_back(instance_case.id());
        instance.set_criterion(criterion_formula{s});
    }

    return instance;
}

Instance parse_instance(const std::filesystem::path & instance_path) {
    if(!std::filesystem::exists(instance_path))
        throw std::invalid_argument("File '" + instance_path.string() +
                                    "' does not exists");
    if(std::filesystem::is_directory(instance_path))
        throw std::invalid_argument("'" + instance_path.string() +
                                    "' is a directory");

    std::ifstream instance_stream(instance_path);
    nlohmann::json instance_json;
    instance_stream >> instance_json;

    return parse_instance_json(instance_json, instance_path.parent_path());
}

}  // namespace fhamonic