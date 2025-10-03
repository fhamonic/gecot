#include <gtest/gtest.h>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "melon/mapping.hpp"
#include "melon/utility/graphviz_printer.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

// GTEST_TEST(parse_instance, json_test) {
//     static const nlohmann::json instance_schema = R"(
// {

// }
// )"_json;
//     std::string instance_path = PROJECT_SOURCE_DIR;
//     // instance_path.append("/test/instances/aude.json");
//     instance_path.append("/test/instances/quebec_438_RASY.json");
//     // instance_path.append("/test/instances/biorevaix_N1.json");
//     Instance instance = parse_instance(instance_path);
//     double budget = 0;
//     for(auto && o : instance.options()) budget += instance.option_cost(o);
// }

GTEST_TEST(parse_instance, test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    // instance_path.append("/test/instances/aude.json");
    instance_path.append("/test/instances/Quebec_438_RASY/instance.json");
    Instance instance = parse_instance(instance_path);

    ASSERT_EQ(std::ranges::size(instance.options()), 260);
    double budget = 0;
    for(auto && o : instance.options()) budget += instance.option_cost(o);
    ASSERT_DOUBLE_EQ(budget, 426654.0);
    ASSERT_EQ(budget,
              gecot::compute_solution_cost(instance, melon::views::true_map{}));

    ASSERT_EQ(std::ranges::size(instance.cases()), 1);
    InstanceCase squirell = *std::ranges::begin(instance.cases());
    auto && graph = squirell.graph();
    ASSERT_EQ(melon::num_vertices(graph), 518);
    ASSERT_EQ(melon::num_arcs(graph), 1814);

    double quality_sum = 0.0;
    auto && source_quality_map = squirell.source_quality_map();
    auto && target_quality_map = squirell.target_quality_map();
    ASSERT_EQ(source_quality_map.size(), 518);
    ASSERT_EQ(target_quality_map.size(), 518);
    for(auto && v : melon::vertices(graph)) {
        ASSERT_EQ(source_quality_map[v], target_quality_map[v]);
        quality_sum += source_quality_map[v];
    }
    ASSERT_DOUBLE_EQ(quality_sum, 1673676);

    double probability_sum = 0.0;
    auto && probability_map = squirell.arc_probability_map();
    ASSERT_EQ(probability_map.size(), 1814);
    for(auto && a : melon::arcs(graph)) {
        probability_sum += probability_map[a];
    }
    ASSERT_DOUBLE_EQ(probability_sum, 613.66037479604245);

    auto && pc_value = gecot::pc_num(graph, source_quality_map,
                                     target_quality_map, probability_map);
    ASSERT_DOUBLE_EQ(pc_value, 117149420274.53568);
    ASSERT_DOUBLE_EQ(pc_value, gecot::compute_base_score(instance));
    ASSERT_DOUBLE_EQ(pc_value, gecot::compute_solution_cases_pc_num(
                                   instance, melon::views::false_map{})[0]);

    auto && vertex_options_map = squirell.vertex_options_map();
    ASSERT_EQ(vertex_options_map.size(), 518);
    for(auto && u : melon::vertices(graph)) {
        for(auto && [source_quality_gain, target_quality_gain, option] :
            vertex_options_map[u]) {
            std::cout << source_quality_gain << " " << target_quality_gain
                      << " " << option << std::endl;
        }
    }

    auto && arc_options_map = squirell.arc_options_map();
    ASSERT_EQ(arc_options_map.size(), 1814);
    for(auto && a : melon::arcs(graph)) {
        for(auto && [improved_probability, option] : arc_options_map[a]) {
            std::cout << improved_probability << " " << option << std::endl;
        }
    }

    const auto cases_maximum_pc_num = gecot::compute_solution_cases_pc_num(
        instance, melon::views::true_map{});
    auto && maximum_pc_value = instance.eval_criterion(cases_maximum_pc_num);
    ASSERT_DOUBLE_EQ(maximum_pc_value, 150608123477.23215);
}
