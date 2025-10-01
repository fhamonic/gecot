#ifndef IO_HELPER_HPP
#define IO_HELPER_HPP

#include <iostream>
#include <ostream>
#include <string>

#include <fmt/os.h>

#include <spdlog/spdlog.h>

namespace fhamonic {

template <spdlog::level::level_enum LVL>
void print_instance_size(auto && instance,
                         const std::string & instance_name = "Instance") {
    spdlog::log(LVL, "{} has {} options and {} species graph:", instance_name,
                instance.num_options(), instance.cases().size());
    const std::size_t instance_name_max_length =
        std::ranges::max(std::ranges::views::transform(
            instance.cases(), [&](auto && c) { return c.name().size(); }));
    for(auto instance_case : instance.cases()) {
        auto && graph = instance_case.graph();
        int num_improvable_vertices = 0;
        int num_habitat_vertices = 0;
        for(auto && v : melon::vertices(graph)) {
            num_improvable_vertices +=
                instance_case.vertex_options_map()[v].size() > 0;
            num_habitat_vertices += instance_case.source_quality_map()[v] > 0;
        }
        int num_improvable_arcs = 0;
        for(auto && a : melon::arcs(graph))
            num_improvable_arcs += instance_case.arc_options_map()[a].size() > 0;
        spdlog::log(
            LVL, "    {:>{}}: {:>6} vertices ({} improvable, {} habitats)",
            instance_case.name(), instance_name_max_length, graph.num_vertices(),
            num_improvable_vertices, num_habitat_vertices);
        spdlog::log(LVL, "    {:>{}}{:>8} arcs     ({} improvable)", "",
                    instance_name_max_length, graph.num_arcs(),
                    num_improvable_arcs);
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

template <spdlog::level::level_enum LVL, std::size_t BAR_LENGTH>
class progress_bar {
private:
    std::atomic<std::size_t> _current_ticks;
    std::size_t _final_ticks;

private:
    void _print_bar(std::size_t progress) {
        fmt::print("[{0:=^{1}}>{0: ^{2}}] {3:>3}%\r", "", progress,
                   BAR_LENGTH - progress, 100 * progress / BAR_LENGTH);
        fflush(stdout);
    }

public:
    progress_bar(const std::size_t num_ticks)
        : _current_ticks(0), _final_ticks(num_ticks) {
        if(spdlog::get_level() > LVL) return;
        _print_bar(0);
    }
    ~progress_bar() {
        if(spdlog::get_level() > LVL) return;
        fmt::print("{0: ^{1}}\r", "", BAR_LENGTH + 8);
        fflush(stdout);
    }
    void tick(const std::size_t incr = 1) {
        const std::size_t ticks = (_current_ticks += incr);
        if(spdlog::get_level() > LVL) return;
        if((BAR_LENGTH * ticks) % _final_ticks <= BAR_LENGTH) {
            _print_bar(BAR_LENGTH * ticks / _final_ticks);
        }
    }
    void reset() { _current_ticks = 0; }
};

}  // namespace fhamonic

#endif  // IO_HELPER_HPP