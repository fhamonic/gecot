#ifndef IO_HELPER_HPP
#define IO_HELPER_HPP

#include <iostream>
#include <ostream>
#include <string>

#include <fmt/os.h>

#include <spdlog/spdlog.h>

#include "gecot/utils/log_table.hpp"

template <spdlog::level::level_enum LVL>
void print_instance_size(auto && instance, const std::string & instance_name) {
    spdlog::log(LVL, "{}: {} options, {} species", instance_name,
                instance.num_options(), instance.cases().size());
    gecot::log_table<LVL, 0, 11, 8, 8, 10, 7, 10> sizes(
        {"species"}, {"vertices"}, {"habitats"}, {"improvable vertices"},
        {"arcs"}, {"improvable arcs"});
    for(const auto & instance_case : instance.cases()) {
        auto && graph = instance_case.graph();
        auto && vertex_options_map = instance_case.vertex_options_map();
        auto && source_quality_map = instance_case.source_quality_map();
        auto && target_quality_map = instance_case.target_quality_map();
        int num_improvable_vertices = 0;
        int num_habitat_vertices = 0;
        for(auto && v : melon::vertices(graph)) {
            num_improvable_vertices += vertex_options_map[v].size() > 0;
            num_habitat_vertices +=
                source_quality_map[v] > 0 || target_quality_map[v] > 0;
        }
        int num_improvable_arcs = 0;
        for(auto && a : melon::arcs(graph))
            num_improvable_arcs +=
                instance_case.arc_options_map()[a].size() > 0;
        sizes.row(instance_case.name(), graph.num_vertices(),
                  num_habitat_vertices, num_improvable_vertices,
                  graph.num_arcs(), num_improvable_arcs);
    }
}

inline void print_paragraph(std::size_t offset, std::size_t column_width,
                            const std::string & str) {
    std::size_t line_start = 0;
    while(str.size() - line_start > column_width) {
        std::size_t n = str.rfind(' ', line_start + column_width);
        if(n <= line_start) {
            fmt::print("{:<{}}\n{: ^{}}",
                       str.substr(line_start, column_width - 1),
                       column_width - offset, "", offset);
            line_start += column_width - 1;
        } else {
            fmt::print("{:<{}}\n{: ^{}}",
                       str.substr(line_start, n - line_start),
                       column_width - offset, "", offset);
            line_start = n + 1;
        }
    }
    fmt::println("{:<{}}", str.substr(line_start), column_width - offset);
}

#endif  // IO_HELPER_HPP