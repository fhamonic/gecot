#ifndef IO_HELPER_HPP
#define IO_HELPER_HPP

#include <iostream>
#include <ostream>
#include <string>

#include <spdlog/spdlog.h>

namespace fhamonic {

template <spdlog::level::level_enum lvl>
void print_instance_size(auto && instance, const std::string & instance_name  = "Instance") {    
    spdlog::log(lvl, "{} has {} options and {} species graph:", instance_name, instance.nb_options(), instance.cases().size());
    for(auto instance_case : instance.cases()) {
        auto && graph = instance_case.graph();
        int nb_improvable_vertices = 0;
        for(auto && v : melon::vertices(graph))
            nb_improvable_vertices += instance_case.vertex_options_map()[v].size() > 0;
        int nb_improvable_arcs = 0;
        for(auto && a : melon::arcs(graph))
            nb_improvable_arcs += instance_case.arc_options_map()[a].size() > 0;
        spdlog::log(lvl, "  {}: {} vertices ({} improvable), {} arcs ({} improvable)", instance_case.name(), graph.nb_vertices(), nb_improvable_vertices, graph.nb_arcs(), nb_improvable_arcs);
    }
}

void print_paragraph(std::ostream & os, std::size_t offset,
                     std::size_t column_width, const std::string & str) {
    std::size_t line_start = 0;
    while(str.size() - line_start > column_width) {
        std::size_t n = str.rfind(' ', line_start + column_width);
        if(n <= line_start) {
            os << str.substr(line_start, column_width - 1) << '-' << '\n'
               << std::string(offset, ' ');
            line_start += column_width - 1;
        } else {
            os << str.substr(line_start, n - line_start) << '\n'
               << std::string(offset, ' ');
            line_start = n + 1;
        }
    }
    os << str.substr(line_start) << '\n';
}

}  // namespace fhamonic

#endif  // IO_HELPER_HPP