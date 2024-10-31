#ifndef SVG_WRITER_HPP
#define SVG_WRITER_HPP

#include <optional>

#include <fmt/core.h>
#include <fmt/os.h>

#include "geometries.hpp"

class svg_writer {
private:
    FILE * file;

public:
    svg_writer() {
        file = fopen("test/test.svg", "w");
        fmt::println(file, R"(<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="100%" height="100%" version="1.1"
xmlns="http://www.w3.org/2000/svg"
xmlns:xlink="http://www.w3.org/1999/xlink">)");
    }

    svg_writer & add_class(std::string class_name, std::string class_style) {
        fmt::println(file, R"(<style>
    .{} {{
        {}
    }}      
</style>)",
                     class_name, class_style);
    }

    svg_writer & add_shape(
        box_2d box, std::optional<std::string> class_name = std::nullopt) {
        fmt::print(file,
                   "<rect x=\"{:.4f}\" y=\"{:.4f}\" width=\"{:.4f}\" "
                   "height=\"{:.4f}\"",
                   box.min_corner().x(), box.min_corner().y(),
                   (box.max_corner().x() - box.min_corner().x()),
                   (box.max_corner().y() - box.min_corner().y()));
        if(class_name.has_value())
            fmt::print(file, " class=\"{}\"", class_name.value());
        fmt::println(file, "/>");
        return *this;
    }

    svg_writer & add_shape(
        ring_2d ring, std::optional<std::string> class_name = std::nullopt) {
        fmt::print(file, "<polygon points=\"");
        for(auto p : ring) fmt::print(file, "{:.4f},{:.4f} ", p.x(), p.y());
        fmt::print(file, "\"");
        if(class_name.has_value())
            fmt::print(file, " class=\"{}\"", class_name.value());
        fmt::println(file, "/>");
        return *this;
    }
};

#endif  // SVG_WRITER_HPP