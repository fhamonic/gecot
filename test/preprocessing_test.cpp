#include <gtest/gtest.h>
#include <string>

#include "melon/utility/graphviz_printer.hpp"
#include "melon/utility/value_map.hpp"
#include "melon/views/reverse.hpp"

#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "landscape_opt/preprocessing/compute_generalized_flow_graph.hpp"
#include "landscape_opt/preprocessing/compute_strong_and_useless_arcs.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

GTEST_TEST(preprocessing, fuzzy_test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    instance_path.append("/test/instances/aude.json");

    Instance instance = parse_instance(instance_path);
    const auto & original_graph = instance.landscape().graph();
    const auto & original_quality_map = instance.landscape().quality_map();
    const auto & original_probability_map =
        instance.landscape().probability_map();

    const auto [graph, quality_map, vertex_options_map, probability_map,
                arc_option_map] =
        landscape_opt::compute_generalized_flow_graph(instance);

    const auto [strong_arcs_map, useless_arcs_map] =
        landscape_opt::compute_strong_and_useless_arcs(instance, true);

    int cpt = 0;
    int cpt_ok = 0;

    for(const auto & original_t : melon::vertices(original_graph)) {
        const auto [contracted_graph, contracted_quality_map,
                    contracted_vertex_options_map, contracted_arc_no_map,
                    contracted_probability_map, contracted_arc_option_map, t] =
            landscape_opt::compute_contracted_generalized_flow_graph(
                instance, strong_arcs_map, useless_arcs_map, original_t);

        const double contribution = landscape_opt::eca_vertex_contribution(
            melon::views::reverse(graph), quality_map, probability_map,
            original_t);
        const double contracted_contribution =
            landscape_opt::eca_vertex_contribution(
                melon::views::reverse(contracted_graph), contracted_quality_map,
                contracted_probability_map, t);

        std::cout << contribution << " " << contracted_contribution
                  << std::endl;

        cpt += 1;
        if(std::abs(contribution - contracted_contribution) <= 1e-6)
            cpt_ok += 1;

        if(std::ranges::distance(melon::out_arcs(contracted_graph, t)) == 0)
            continue;

        // melon::graphviz_printer printer(original_graph);
        // auto lower_length_map = original_probability_map;
        // const auto & arc_options_map = instance.arc_options_map();
        // for(const auto & a : melon::arcs(original_graph)) {
        //     for(const auto & [improved_prob, option] : arc_options_map[a]) {
        //         lower_length_map[a] =
        //             std::max(lower_length_map[a], improved_prob);
        //     }
        // }
        // printer
        //     .set_vertex_color_map(melon::views::map(
        //         [&](auto && v)
        //             -> std::tuple<unsigned char, unsigned char, unsigned char> {
        //             if(v == original_t)
        //                 return {255, 0, 0};
        //             else
        //                 return {255, 255, 255};
        //         }))
        //     .set_arc_label_map(melon::views::map([&](auto && a) {
        //         return "[" + std::to_string(original_probability_map[a]) + "," +
        //                std::to_string(lower_length_map[a]) + "]";
        //     }));
        // for(auto && a : strong_arcs_map[original_t]) {
        //     printer.set_arc_color(a, {255, 0, 0});
        // }
        // for(auto && a : useless_arcs_map[original_t]) {
        //     printer.set_arc_color(a, {0, 0, 255});
        // }
        // printer.print(std::ostream_iterator<char>(std::cout));



        // melon::graphviz_printer printer2(contracted_graph);
        // printer2
        //     .set_vertex_color_map(melon::views::map(
        //         [&](auto && v)
        //             -> std::tuple<unsigned char, unsigned char, unsigned char> {
        //             if(v == t)
        //                 return {255, 0, 0};
        //             else
        //                 return {255, 255, 255};
        //         }))
        //     .set_arc_label_map(melon::views::map([&](auto && a) {
        //         return "[" + std::to_string(probability_map[a]) + "]";
        //     }));
        // printer2.print(std::ostream_iterator<char>(std::cout));
        // ASSERT_EQ(contribution, contracted_contribution);
    }

    // ASSERT_TRUE(false);
    ASSERT_EQ(cpt, cpt_ok);
}
