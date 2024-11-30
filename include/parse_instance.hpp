#ifndef PARSE_INSTANCE_HPP
#define PARSE_INSTANCE_HPP

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

void parse_columns_aliases(
    const nlohmann::json & json_object,
    std::initializer_list<std::reference_wrapper<std::string>> column_names);

void parse_options(Instance & instance, const nlohmann::json & json_object,
                   const std::filesystem::path & parent_path);
void parse_vertices_options(InstanceCase & instance_case,
                            const nlohmann::json & json_object,
                            const std::filesystem::path & parent_path,
                            const Instance & instance);
void parse_arcs_options(InstanceCase & instance_case,
                        const nlohmann::json & json_object,
                        const std::filesystem::path & parent_path,
                        const Instance & instance);
InstanceCase & parse_instance_case(const nlohmann::json & json_object,
                                   std::string case_name,
                                   const std::filesystem::path & parent_path,
                                   Instance & instance);
criterion_formula parse_formula(Instance & instance,
                                const nlohmann::json & json_object);
Instance parse_instance_json(const nlohmann::json & instance_json,
                             const std::filesystem::path & relative_path = "");
Instance parse_instance(const std::filesystem::path & instance_path);

}  // namespace fhamonic

#endif  // PARSE_INSTANCE_HPP