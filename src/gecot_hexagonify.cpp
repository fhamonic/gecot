#include <cmath>
#include <fstream>
#include <iostream>
#include <ranges>

#include <fmt/core.h>
#include <fmt/os.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <gdal_priv.h>

#include <boost/geometry/io/svg/svg_mapper.hpp>

#include "hexagonify/hexagon_grid.hpp"

int main() {
    GDALAllRegister();

    const char * filename = "test/PC1.tif";
    GDALDataset * dataset = (GDALDataset *)GDALOpen(filename, GA_ReadOnly);
    if(dataset == nullptr) {
        std::cerr << "GDALOpen failed - " << CPLGetLastErrorMsg() << std::endl;
        return EXIT_FAILURE;
    }

    const int raster_width = dataset->GetRasterXSize();
    const int raster_height = dataset->GetRasterYSize();
    const int num_bands = dataset->GetRasterCount();

    fmt::println("{}x{} raster with {} bands", raster_width, raster_height,
                 num_bands);

    std::vector<std::vector<double>> raster_bands;

    for(int band_num = 1; band_num <= num_bands; ++band_num) {
        GDALRasterBand * band = dataset->GetRasterBand(band_num);
        GDALDataType type = band->GetRasterDataType();

        fmt::println("DataType = {}", (int)type);

        raster_bands.emplace_back(raster_width * raster_height);
        if(band->RasterIO(GF_Read, 0, 0, raster_width, raster_height,
                          raster_bands.back().data(), raster_width,
                          raster_height, GDT_Float64, band_num,
                          0) == CE_Failure) {
            std::cerr << "RasterIO failed - " << CPLGetLastErrorMsg()
                      << std::endl;
            GDALClose(dataset);
            return EXIT_FAILURE;
        }
    }

    const double hexagon_height = 1.0f;
    hexagon_grid hex_grid(raster_width, raster_height, hexagon_height);

    auto hex_bands = hex_grid.discretize_boxes(raster_bands, 10);
    // auto hex_bands = hex_grid.discretize_polygon(raster_bands);

    std::cout << raster_bands[0][0] << std::endl;
    std::cout << hex_bands[0][0] << std::endl;

    //
    //
    //
    auto svg = fopen("test/test.svg", "w");
    fmt::println(svg, R"(<?xml version="1.0" standalone="no"?>
    <!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
    "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
    <svg width="100%" height="100%" version="1.1"
    xmlns="http://www.w3.org/2000/svg"
    xmlns:xlink="http://www.w3.org/1999/xlink">
    <style>
        .a {{
            fill: black;
            stroke: red;
            stroke-width: 0.02;
        }}
    </style>)");

    int cpt = 0;
    // for(auto && [x, y] : hexagon_centers) {
        // for(const box_2d & b : rectangles) {
        //     fmt::println(svg,
        //                  "<rect x=\"{:.4f}\" y=\"{:.4f}\" width=\"{:.4f}\" "
        //                  "height=\"{:.4f}\" class=\"a\"/>",
        //                  b.min_corner().x() + x, b.min_corner().y() + y,
        //                  (b.max_corner().x() - b.min_corner().x()),
        //                  (b.max_corner().y() - b.min_corner().y()));
        // }

        // fmt::print(svg, "<polygon points=\"");
        // for(auto p : ring) {
        //     fmt::print(svg, "{:.4f},{:.4f} ", p.x() + x, p.y() + y);
        // }
        // fmt::println(svg, "\" class=\"a\"/>");

        // if(++cpt >= 10) break;
    // }

    fmt::println(svg, R"(</svg>)");

    GDALClose(dataset);
    return EXIT_SUCCESS;
}
