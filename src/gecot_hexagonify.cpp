#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <mutex>
#include <print>
#include <ranges>
#include <stdexcept>
#include <vector>

#include <tiffio.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "melon/algorithm/dijkstra.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

auto parse_tiff(const std::string & filename) {
    uint32_t width;
    uint32_t height;
    std::vector<uint32_t> data;

    TIFF * tif = TIFFOpen(filename.c_str(), "r");
    if(!tif) throw std::runtime_error("Failed to open TIFF file.");

    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    uint16_t samplesPerPixel;
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel);
    if(samplesPerPixel != 1)
        throw std::runtime_error("Only 1 sample per peixel is supported.");
    uint16_t bitsPerSample;
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
    if(bitsPerSample != 32)
        throw std::runtime_error("Only 32-bit TIFF files are supported.");

    data.resize(width * height);
    for(uint32_t row = 0; row < height; ++row) {
        if(TIFFReadScanline(tif, data.data() + row * width, row, 0) < 0) {
            TIFFClose(tif);
            throw std::runtime_error("Error reading scanline.");
        }
    }
    TIFFClose(tif);

    return std::make_tuple(width, height, data);
}

using namespace fhamonic::melon;

auto compute_graph(int i, double ratio) {
    auto [width, height, data] = parse_tiff(std::format(
        "/home/plaiseek/Projects/rs/Paysages_ECA/paysage_{}.tif", i));

    // for(uint32_t y = 0; y < height; ++y) {
    //     for(uint32_t x = 0; x < width; ++x) {
    //         std::cout << data[cell(x, y)];
    //     }
    //     std::cout << std::endl;
    // }

    static_digraph_builder<static_digraph, double> builder(height * width);
    auto cell = [width](uint32_t x, uint32_t y) { return y * width + x; };

    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) {
            if(y > 0) builder.add_arc(cell(x, y), cell(x, y - 1), 1.0);
            if(x > 0) builder.add_arc(cell(x, y), cell(x - 1, y), 1.0);
            if(y < height - 1) builder.add_arc(cell(x, y), cell(x, y + 1), 1.0);
            if(x < width - 1) builder.add_arc(cell(x, y), cell(x + 1, y), 1.0);

            if(y > 0 && x > 0)
                builder.add_arc(cell(x, y), cell(x - 1, y - 1), std::sqrt(2));
            if(y > 0 && x < width - 1)
                builder.add_arc(cell(x, y), cell(x + 1, y - 1), std::sqrt(2));
            if(y < height - 1 && x > 0)
                builder.add_arc(cell(x, y), cell(x - 1, y + 1), std::sqrt(2));
            if(y < height - 1 && x < width - 1)
                builder.add_arc(cell(x, y), cell(x + 1, y + 1), std::sqrt(2));
        }
    }

    auto [graph, dist] = builder.build();

    // std::cout << num_vertices(graph) << std::endl;
    // std::cout << num_arcs(graph) << std::endl;

    for(const auto & [a, p] : arcs_entries(graph)) {
        const auto & [u, v] = p;
        dist[a] *= 30 * ((data[u] ? ratio : 1) + (data[v] ? ratio : 1)) / 2;
    }

    return std::make_tuple(graph, data, dist);
}

struct dijkstra_traits {
    using semiring = shortest_path_semiring<double>;
    using heap =
        updatable_d_ary_heap<4, std::pair<vertex_t<static_digraph>, double>,
                             typename semiring::less_t,
                             vertex_map_t<static_digraph, std::size_t>,
                             views::element_map<1>, views::element_map<0>>;

    static constexpr bool store_paths = false;
    static constexpr bool store_distances = false;
};

int main() {
    auto landscapes = std::ranges::to<std::vector>(std::views::iota(1, 52));
    std::vector<double> ratios = {1, 2, 3, 6, 12, 20};
    std::vector<double> matrix_alphas = {200, 400, 900, 1200, 1600, 2000, 2200};

    std::print("i,ratio,alpha,pc_num_sum\n");

    std::mutex print_mutex;
    tbb::parallel_for(
        tbb::blocked_range2d(landscapes.begin(), landscapes.end(),
                             ratios.begin(), ratios.end()),
        [&](const tbb::blocked_range2d<decltype(landscapes.begin()),
                                       decltype(ratios.begin())> &
                landscapes_ratios) {
            for(const auto & i : landscapes_ratios.rows()) {
                for(const auto & ratio : landscapes_ratios.cols()) {
                    const auto [graph, habitat_map, length_map] =
                        compute_graph(i, ratio);
                    auto algo = dijkstra(dijkstra_traits{}, graph, length_map);
                    auto dists = std::make_unique_for_overwrite<double[]>(
                        num_vertices(graph) * num_vertices(graph));
                    auto cursor = dists.get();
                    for(const auto & s : vertices(graph)) {
                        if(habitat_map[s] == 0) continue;
                        double sum = 0.0;
                        algo.reset();
                        algo.add_source(s);
                        for(const auto & [u, dist] : algo) {
                            *cursor = dist;
                            cursor += habitat_map[u];
                        }
                    }

                    for(double alpha : matrix_alphas) {
                        double pc_num_sum = 0;
                        for(auto dist : std::span(dists.get(), cursor))
                            pc_num_sum += std::exp(-dist / alpha);
                        pc_num_sum = (30 * 30) * std::sqrt(pc_num_sum) / 10000;

                        std::lock_guard<std::mutex> guard(print_mutex);
                        std::print("{},{},{},{}\n", i, ratio, alpha,
                                   pc_num_sum);
                    }
                }
            }
        });

    return 0;
}
