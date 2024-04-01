#ifndef IO_HELPER_HPP
#define IO_HELPER_HPP

#include <iostream>
#include <ostream>
#include <string>

#include <fmt/os.h>

#include <spdlog/spdlog.h>

namespace fhamonic {

template <spdlog::level::level_enum lvl>
void print_instance_size(auto && instance,
                         const std::string & instance_name = "Instance") {
    spdlog::log(lvl, "{} has {} options and {} species graph:", instance_name,
                instance.nb_options(), instance.cases().size());
    const std::size_t instance_name_max_length =
        std::ranges::max(std::ranges::views::transform(
            instance.cases(), [&](auto && c) { return c.name().size(); }));
    for(auto instance_case : instance.cases()) {
        auto && graph = instance_case.graph();
        int nb_improvable_vertices = 0;
        int nb_habitat_vertices = 0;
        for(auto && v : melon::vertices(graph)) {
            nb_improvable_vertices +=
                instance_case.vertex_options_map()[v].size() > 0;
            nb_habitat_vertices += instance_case.vertex_quality_map()[v] > 0;
        }
        int nb_improvable_arcs = 0;
        for(auto && a : melon::arcs(graph))
            nb_improvable_arcs += instance_case.arc_options_map()[a].size() > 0;
        spdlog::log(lvl, "    {:>{}}: {:>6} vertices ({} improvable, {} habitats)",
                    instance_case.name(), instance_name_max_length,
                    graph.nb_vertices(), nb_improvable_vertices,
                    nb_habitat_vertices);
        spdlog::log(lvl, "    {:>{}}{:>8} arcs     ({} improvable)", "",
                    instance_name_max_length, graph.nb_arcs(),
                    nb_improvable_arcs);
    }
}

void print_paragraph(std::size_t offset, std::size_t column_width,
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

}  // namespace fhamonic

#endif  // IO_HELPER_HPP