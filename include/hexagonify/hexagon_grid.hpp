#ifndef HEXAGON_GRID_HPP
#define HEXAGON_GRID_HPP

#include <ranges>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/zip.hpp>

#include "hexagonify/geometries.hpp"

struct hexagon_grid {
public:
    std::size_t raster_width;
    std::size_t raster_height;
    double hexagon_height;
    double hexagon_side_length;
    double hexagon_width;
    double hexagon_area;
    std::vector<point_2d> hexagon_centers;
    std::size_t num_hexagons_per_line;
    std::size_t num_hexagon_lines;
    std::size_t num_hexagons;

    hexagon_grid(int raster_width, int raster_height, double hexagon_height) {
        this->raster_width = raster_width;
        this->raster_height = raster_height;
        this->hexagon_height = hexagon_height;
        hexagon_side_length = hexagon_height / 2;
        hexagon_width = std::sqrt(3) * hexagon_side_length;
        hexagon_area = 3 * std::sqrt(3) / 8 * hexagon_height * hexagon_height;

        num_hexagons_per_line =
            (raster_width - hexagon_width / 2.0) / hexagon_width;
        num_hexagon_lines =
            1 + (raster_height - hexagon_height) / (0.75f * hexagon_height);
        num_hexagons = num_hexagon_lines * num_hexagons_per_line;

        hexagon_centers.reserve(num_hexagons);
        bool x_offset_b = false;
        double y = hexagon_height / 2;
        for(std::size_t j = 0; j < num_hexagon_lines; ++j) {
            double x = x_offset_b ? hexagon_width : hexagon_width / 2;
            for(std::size_t i = 0; i < num_hexagons_per_line; ++i) {
                hexagon_centers.emplace_back(x, y);
                x += hexagon_width;
            }
            x_offset_b = !x_offset_b;
            y += 0.75 * hexagon_height;
        }
    }

    ring_2d hexagon_ring() {
        ring_2d ring;
        ring.reserve(7);
        ring.emplace_back(0, -hexagon_height / 2);
        ring.emplace_back(hexagon_width / 2, -hexagon_height / 4);
        ring.emplace_back(hexagon_width / 2, hexagon_height / 4);
        ring.emplace_back(0, hexagon_height / 2);
        ring.emplace_back(-hexagon_width / 2, hexagon_height / 4);
        ring.emplace_back(-hexagon_width / 2, -hexagon_height / 4);
        ring.emplace_back(0, -hexagon_height / 2);
        return ring;
    }

    std::vector<box_2d> hexagon_boxes(int discretisation = 1) {
        const double side_length = hexagon_height / 2;
        const std::size_t num_boxes = 1 + 2 * discretisation;

        std::vector<box_2d> boxes;
        boxes.reserve(num_boxes);

        if(discretisation == 0) {
            boxes.emplace_back(point_2d(-hexagon_width / 2,
                                        -side_length / 2 - hexagon_height / 8),
                               point_2d(hexagon_width / 2,
                                        side_length / 2 + hexagon_height / 8));
        } else {
            const double step_width = hexagon_width / 2 / (discretisation + 1);
            const double step_height = hexagon_height / 4 / discretisation;
            for(int i = 0; i < discretisation; ++i) {
                boxes.emplace_back(
                    point_2d(-step_width * (i + 1),
                             -hexagon_height / 2 + step_height * i),
                    point_2d(step_width * (i + 1),
                             -hexagon_height / 2 + step_height * (i + 1)));
            }
            boxes.emplace_back(point_2d(-hexagon_width / 2, -side_length / 2),
                               point_2d(hexagon_width / 2, side_length / 2));
            for(int i = discretisation - 1; i >= 0; --i) {
                boxes.emplace_back(
                    point_2d(-step_width * (i + 1),
                             hexagon_height / 2 - step_height * (i + 1)),
                    point_2d(step_width * (i + 1),
                             hexagon_height / 2 - step_height * i));
            }
        }

        return boxes;
    }

    auto discretize_polygon(const std::vector<std::vector<double>> & bands) {
        const std::size_t num_bands = bands.size();
        std::vector<std::vector<double>> hex_bands(num_bands);
        for(auto & hex_band : hex_bands) {
            hex_band.resize(hexagon_centers.size(), 0.0);
        }

        const ring_2d ring = hexagon_ring();

        for(auto && [hex_num, hex_center] :
            ranges::views::enumerate(hexagon_centers)) {
            ring_2d hex = translated(ring, hex_center);

            box_2d b;
            boost::geometry::envelope(hex, b);
            const int first_i = static_cast<int>(b.min_corner().x());
            const int last_i = static_cast<int>(std::ceil(b.max_corner().x()));
            const int first_j = static_cast<int>(b.min_corner().y());
            const int last_j = static_cast<int>(std::ceil(b.max_corner().y()));

            for(int i = first_i; i <= last_i; ++i) {
                for(int j = first_j; j <= last_j; ++j) {
                    auto cell_rect =
                        box_2d(point_2d(i, j), point_2d(i + 1, j + 1));
                    ring_2d intersection;
                    boost::geometry::intersection(hex, cell_rect, intersection);
                    const double intersection_area =
                        boost::geometry::area(intersection);

                    for(std::size_t k = 0; k < num_bands; ++k) {
                        hex_bands[k][hex_num] +=
                            intersection_area * bands[k][j * raster_width + i];
                    }
                }
            }
        }

        for(auto & hex_band : hex_bands) {
            for(double & hex_value : hex_band) hex_value /= hexagon_area;
        }

        return hex_bands;
    }

    auto discretize_boxes(const std::vector<std::vector<double>> & bands,
                          std::size_t discretization = 0) {
        const std::size_t num_bands = bands.size();
        std::vector<std::vector<double>> hex_bands(num_bands);
        for(auto & hex_band : hex_bands) {
            hex_band.resize(hexagon_centers.size(), 0.0);
        }

        auto && rectangles = hexagon_boxes(discretization);
        for(auto && [hex_num, hex_center] :
            ranges::views::enumerate(hexagon_centers)) {
            for(const box_2d & b : rectangles) {
                box_2d r = translated(b, hex_center);

                const int first_i = static_cast<int>(r.min_corner().x());
                const int last_i =
                    static_cast<int>(std::ceil(r.max_corner().x()));
                const int first_j = static_cast<int>(r.min_corner().y());
                const int last_j =
                    static_cast<int>(std::ceil(r.max_corner().y()));

                for(int i = first_i; i <= last_i; ++i) {
                    for(int j = first_j; j <= last_j; ++j) {
                        auto cell_rect =
                            box_2d(point_2d(i, j), point_2d(i + 1, j + 1));
                        const float intersection_area =
                            boost::geometry::area(intersect(r, cell_rect));

                        for(std::size_t k = 0; k < num_bands; ++k) {
                            hex_bands[k][hex_num] +=
                                intersection_area *
                                bands[k][j * raster_width + i];
                        }
                    }
                }
            }
        }

        for(auto & hex_band : hex_bands) {
            for(double & hex_value : hex_band) hex_value /= hexagon_area;
        }

        return hex_bands;
    }
};

#endif  // HEXAGON_GRID_HPP