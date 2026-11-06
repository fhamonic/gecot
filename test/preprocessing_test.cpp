#include <gtest/gtest.h>
#include <algorithm>
#include <random>
#include <string>
#include <tuple>
#include <vector>

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

double compute_solution_contribution(auto && graph, auto source_quality_map,
                                     auto && vertex_options_map,
                                     auto probability_map,
                                     auto && arc_option_map, auto && t,
                                     auto && solution) {
    for(auto && u : melon::vertices(graph)) {
        for(auto && option_entry : vertex_options_map[u]) {
            if(!solution[std::get<gecot::option_t>(option_entry)]) continue;
            source_quality_map[u] += std::get<0>(option_entry);
        }
    }
    for(auto && a : melon::arcs(graph)) {
        const auto & option = arc_option_map[a];
        if(option.has_value() && !solution[option.value()])
            probability_map[a] = 0.0;
    }
    return gecot::pc_num_vertex_in_flow(graph, source_quality_map,
                                        probability_map, t);
}

GTEST_TEST(preprocessing, fuzzy_test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    // instance_path.append("/test/instances/aude.json");
    instance_path.append("/test/instances/Quebec_438_RASY/instance.json");
    // instance_path.append("/test/instances/biorevaix_N1.json");
    Instance instance = parse_instance(instance_path);
    double budget = 0;
    for(auto && o : instance.options()) budget += instance.option_cost(o);
    budget /= 10;
    const auto option_predicate = [&](const gecot::option_t & o) {
        return instance.option_cost(o) <= budget;
    };

    int cpt = 0;
    int cpt_ok = 0;
    std::size_t contracted_num_variables = 0;
    std::size_t contracted_num_constraints = 0;
    std::size_t num_variables = 0;
    std::size_t num_constraints = 0;

    for(auto && instance_case : instance.cases()) {
        const auto & original_graph = instance_case.graph();
        const auto & original_source_quality_map =
            instance_case.source_quality_map();
        const auto & original_probability_map =
            instance_case.arc_probability_map();

        const auto [graph, source_quality_map, target_quality_map,
                    vertex_options_map, probability_map, arc_option_map] =
            gecot::compute_generalized_flow_graph(instance_case);

        for(const bool constrained : {false, true}) {
            const auto [strong_arcs_map, useless_arcs_map] =
                constrained
                    ? gecot::compute_constrained_strong_and_useless_arcs(
                          instance, instance_case, budget, option_predicate,
                          1e-7)
                    : gecot::compute_strong_and_useless_arcs(instance_case,
                                                             option_predicate);

            for(const auto & original_t : melon::vertices(original_graph)) {
                const double contribution = gecot::pc_num_vertex_in_flow(
                    graph, source_quality_map, probability_map, original_t);

                const auto [contracted_graph, contracted_source_quality_map,
                            contracted_vertex_options_map,
                            contracted_arc_no_map, contracted_probability_map,
                            contracted_arc_option_map, t] =
                    gecot::compute_contracted_generalized_flow_graph(
                        instance_case, strong_arcs_map[original_t],
                        useless_arcs_map[original_t], original_t);

                std::cout << "<contracted>" << std::endl;
                const double contracted_contribution =
                    gecot::pc_num_vertex_in_flow(contracted_graph,
                                                 contracted_source_quality_map,
                                                 contracted_probability_map, t);
                std::cout << "</contracted>" << std::endl;

                cpt += 1;
                if(std::abs(contribution - contracted_contribution) <=
                   1e-6 * std::min(contribution, contracted_contribution)) {
                    cpt_ok += 1;
                } else {
                    std::cout << original_t << "\t"
                              << melon::num_vertices(graph) << "\t"
                              << melon::num_arcs(graph) << "\t" << contribution
                              << "\t" << melon::num_vertices(contracted_graph)
                              << "\t" << melon::num_arcs(contracted_graph)
                              << "\t" << contracted_contribution << std::endl;
                }

                if(original_source_quality_map[original_t] > 0 ||
                   vertex_options_map[original_t].size() > 0) {
                    num_variables += melon::num_arcs(graph);
                    num_constraints += melon::num_vertices(graph);
                    contracted_num_variables +=
                        melon::num_arcs(contracted_graph);
                    contracted_num_constraints +=
                        melon::num_vertices(contracted_graph);
                }
                // if(std::ranges::distance(melon::out_arcs(contracted_graph,
                // t)) == 0)
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
                //     .set_vertex_color_map(melon::maps::function(
                //         [&](auto && v)
                //             -> std::tuple<unsigned char, unsigned char,
                //             unsigned char> { if(v == original_t)
                //                 return {255, 0, 0};
                //             else
                //                 return {255, 255, 255};
                //         }))
                //     .set_arc_label_map(melon::maps::function([&](auto && a) {
                //         return "[" +
                //         std::to_string(original_probability_map[a])
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
                //     .set_vertex_color_map(melon::maps::function(
                //         [&](auto && v)
                //             -> std::tuple<unsigned char, unsigned char,
                //             unsigned char> { if(v == t)
                //                 return {255, 0, 0};
                //             else
                //                 return {255, 255, 255};
                //         }))
                //     .set_arc_label_map(melon::maps::function([&](auto && a) {
                //         return "[" + std::to_string(probability_map[a]) +
                //         "]";
                //     }));
                // printer2.print(std::ostream_iterator<char>(std::cout));
                // ASSERT_EQ(contribution, contracted_contribution);
            }
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

GTEST_TEST(preprocessing, constrained_labels) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    instance_path.append("/test/instances/Quebec_438_RASY/instance.json");
    Instance instance = parse_instance(instance_path);
    const double budget = 2133;
    const auto option_predicate = [&](const gecot::option_t & o) {
        return instance.option_cost(o) <= budget;
    };

    std::vector<gecot::option_t> affordable_options;
    for(auto && o : instance.options())
        if(option_predicate(o)) affordable_options.push_back(o);
    std::vector<decltype(instance.create_option_map(false))> solutions;
    solutions.push_back(instance.create_option_map(false));
    std::mt19937 rng(0);
    for(int i = 0; i < 4; ++i) {
        std::ranges::shuffle(affordable_options, rng);
        auto & solution =
            solutions.emplace_back(instance.create_option_map(false));
        double cost = 0;
        for(auto && o : affordable_options) {
            if(cost + instance.option_cost(o) > budget) continue;
            solution[o] = true;
            cost += instance.option_cost(o);
        }
    }

    std::size_t num_unconstrained_labels = 0;
    std::size_t num_strong_labels = 0;
    std::size_t num_useless_labels = 0;
    std::size_t num_not_superset = 0;
    std::size_t num_not_nested = 0;
    std::size_t num_invalid = 0;
    for(auto && instance_case : instance.cases()) {
        const auto [unconstrained_strong_arcs_map,
                    unconstrained_useless_arcs_map] =
            gecot::compute_strong_and_useless_arcs(instance_case,
                                                   option_predicate, 1e-8);
        // 10 divides 20: the multipliers of 10 are a subsequence of those of
        // 20, which is what makes its labels a subset
        const auto [strong_arcs_map_10, useless_arcs_map_10] =
            gecot::compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget, option_predicate, 1e-8, 10,
                true);
        const auto [strong_arcs_map, useless_arcs_map] =
            gecot::compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget, option_predicate, 1e-8, 20,
                true);
        const auto [graph, source_quality_map, target_quality_map,
                    vertex_options_map, probability_map, arc_option_map] =
            gecot::compute_generalized_flow_graph(instance_case);

        for(const auto & t : melon::vertices(instance_case.graph())) {
            num_unconstrained_labels +=
                unconstrained_strong_arcs_map[t].size() +
                unconstrained_useless_arcs_map[t].size();
            num_strong_labels += strong_arcs_map[t].size();
            num_useless_labels += useless_arcs_map[t].size();
            num_not_superset += !std::ranges::includes(
                strong_arcs_map[t], unconstrained_strong_arcs_map[t]);
            num_not_superset += !std::ranges::includes(
                useless_arcs_map[t], unconstrained_useless_arcs_map[t]);
            num_not_nested += !std::ranges::includes(strong_arcs_map[t],
                                                     strong_arcs_map_10[t]);
            num_not_nested += !std::ranges::includes(useless_arcs_map[t],
                                                     useless_arcs_map_10[t]);

            const auto [contracted_graph, contracted_source_quality_map,
                        contracted_vertex_options_map, contracted_arc_no_map,
                        contracted_probability_map, contracted_arc_option_map,
                        contracted_t] =
                gecot::compute_contracted_generalized_flow_graph(
                    instance_case, strong_arcs_map[t], useless_arcs_map[t], t);
            for(auto && solution : solutions) {
                const double contribution = compute_solution_contribution(
                    graph, source_quality_map, vertex_options_map,
                    probability_map, arc_option_map, t, solution);
                const double contracted_contribution =
                    compute_solution_contribution(
                        contracted_graph, contracted_source_quality_map,
                        contracted_vertex_options_map,
                        contracted_probability_map, contracted_arc_option_map,
                        contracted_t, solution);
                num_invalid +=
                    std::abs(contribution - contracted_contribution) >
                    1e-6 * std::min(contribution, contracted_contribution);
            }
        }
    }
    EXPECT_EQ(num_not_superset, std::size_t{0});
    EXPECT_EQ(num_not_nested, std::size_t{0});
    EXPECT_EQ(num_invalid, std::size_t{0});
    EXPECT_GT(num_strong_labels + num_useless_labels, num_unconstrained_labels);
    // exact at any thread count; they move with the instance, the budget, the
    // resolution and the multiplier grid, and drop when a pass loses labels
    EXPECT_EQ(num_strong_labels, std::size_t{160083});
    EXPECT_EQ(num_useless_labels, std::size_t{483907});
}
