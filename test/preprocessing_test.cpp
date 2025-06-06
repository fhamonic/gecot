#include <gtest/gtest.h>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "melon/mapping.hpp"
#include "melon/utility/graphviz_printer.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_generalized_flow_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

double compute_original_contribution(auto && graph, auto quality_map,
                                     auto && vertex_option_map,
                                     auto probability_map,
                                     auto && arc_option_map, auto && original_t,
                                     auto && solution) {
    for(auto && u : melon::vertices(graph)) {
        for(auto && [quality_gain, option] : vertex_option_map[u]) {
            if(!solution[option]) continue;
            quality_map[u] += quality_gain;
        }
    }
    for(auto && a : melon::arcs(graph)) {
        for(auto && [enhanced_prob, option] : arc_option_map[a]) {
            if(!solution[option]) continue;
            probability_map[a] = std::max(probability_map[a], enhanced_prob);
        }
    }

    return gecot::pc_num_vertex_contribution(
        melon::views::reverse(graph), quality_map, probability_map, original_t);
}

GTEST_TEST(preprocessing, fuzzy_test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    // instance_path.append("/test/instances/aude.json");
    instance_path.append(
        "/test/instances/Quebec_438_RASY/instance.json");
    // instance_path.append("/test/instances/biorevaix_N1.json");
    Instance instance = parse_instance(instance_path);
    double budget = 0;
    for(auto && o : instance.options()) budget += instance.option_cost(o);
    budget /= 10;

    int cpt = 0;
    int cpt_ok = 0;
    std::size_t contracted_num_variables = 0;
    std::size_t contracted_num_constraints = 0;
    std::size_t num_variables = 0;
    std::size_t num_constraints = 0;

    for(auto && instance_case : instance.cases()) {
        const auto & original_graph = instance_case.graph();
        const auto & original_quality_map = instance_case.vertex_quality_map();
        const auto & original_probability_map =
            instance_case.arc_probability_map();

        const auto [graph, quality_map, vertex_options_map, probability_map,
                    arc_option_map] =
            gecot::compute_generalized_flow_graph(instance_case);

        //*
        const auto [strong_arcs_map, useless_arcs_map] =
            gecot::compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget,
                [&instance, budget](const gecot::option_t & o) {
                    return instance.option_cost(o) <= budget;
                });
        /*/
        const auto [strong_arcs_map, useless_arcs_map] =
            gecot::compute_strong_and_useless_arcs(
                instance_case,
                [&instance, budget](const gecot::option_t & o) {
                    return instance.option_cost(o) <= budget;
                });
        //*/

        for(const auto & original_t : melon::vertices(original_graph)) {
            const double contribution = gecot::pc_num_vertex_contribution(
                melon::views::reverse(graph), quality_map, probability_map,
                original_t);

            const auto [contracted_graph, contracted_quality_map,
                        contracted_vertex_options_map, contracted_arc_no_map,
                        contracted_probability_map, contracted_arc_option_map,
                        t] =
                gecot::compute_contracted_generalized_flow_graph(
                    instance_case, strong_arcs_map[original_t],
                    useless_arcs_map[original_t], original_t);

            const double contracted_contribution =
                gecot::pc_num_vertex_contribution(
                    melon::views::reverse(contracted_graph),
                    contracted_quality_map, contracted_probability_map, t);

            cpt += 1;
            if(std::abs(contribution - contracted_contribution) <=
               1e-6 * std::min(contribution, contracted_contribution)) {
                cpt_ok += 1;
            } else {
                std::cout << original_t << "\t" << melon::num_vertices(graph)
                          << "\t" << melon::num_arcs(graph) << "\t"
                          << contribution << "\t"
                          << melon::num_vertices(contracted_graph) << "\t"
                          << melon::num_arcs(contracted_graph) << "\t"
                          << contracted_contribution << std::endl;
            }

            if(original_quality_map[original_t] > 0 ||
               vertex_options_map[original_t].size() > 0) {
                num_variables += melon::num_arcs(graph);
                num_constraints += melon::num_vertices(graph);
                contracted_num_variables += melon::num_arcs(contracted_graph);
                contracted_num_constraints +=
                    melon::num_vertices(contracted_graph);
            }
            // if(std::ranges::distance(melon::out_arcs(contracted_graph, t)) ==
            // 0)
            //     continue;

            // melon::graphviz_printer printer(original_graph);
            // auto lower_length_map = original_probability_map;
            // const auto & arc_options_map = instance.arc_options_map();
            // for(const auto & a : melon::arcs(original_graph)) {
            //     for(const auto & [improved_prob, option] :
            //     arc_options_map[a]) {
            //         lower_length_map[a] =
            //             std::max(lower_length_map[a], improved_prob);
            //     }
            // }
            // printer
            //     .set_vertex_color_map(melon::views::map(
            //         [&](auto && v)
            //             -> std::tuple<unsigned char, unsigned char, unsigned
            //             char> { if(v == original_t)
            //                 return {255, 0, 0};
            //             else
            //                 return {255, 255, 255};
            //         }))
            //     .set_arc_label_map(melon::views::map([&](auto && a) {
            //         return "[" + std::to_string(original_probability_map[a])
            //         + "," +
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
            //             -> std::tuple<unsigned char, unsigned char, unsigned
            //             char> { if(v == t)
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
    }

    std::cout << "all ok ? : " << cpt << " vs " << cpt_ok << std::endl;
    std::cout << "num_variables: " << num_variables << std::endl;
    std::cout << "num_constraints: " << num_constraints << std::endl;
    std::cout << "contracted_num_variables: " << contracted_num_variables
              << std::endl;
    std::cout << "contracted_num_constraints: " << contracted_num_constraints
              << std::endl;
    ASSERT_EQ(cpt, cpt_ok);
}
