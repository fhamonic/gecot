#include <gtest/gtest.h>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "melon/mapping.hpp"
#include "melon/maps/constant.hpp"
#include "melon/utility/graphviz_printer.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

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
              gecot::compute_solution_cost(instance, melon::maps::true_map{}));

    ASSERT_EQ(std::ranges::size(instance.cases()), 1);
    const auto & squirell = *std::ranges::begin(instance.cases());
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
                                   instance, melon::maps::false_map{})[0]);

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

    const auto cases_maximum_pc_num =
        gecot::compute_solution_cases_pc_num(instance, melon::maps::true_map{});
    auto && maximum_pc_value = instance.eval_criterion(cases_maximum_pc_num);
    ASSERT_DOUBLE_EQ(maximum_pc_value, 150608123477.23215);
}

namespace {

const char * three_woods_cases = R"({
    "squirrel": {
        "vertices": [
            { "id": "wood_a", "source_quality": 2, "target_quality": 2 },
            { "id": "wood_b", "source_quality": 1, "target_quality": 1 },
            { "id": "wood_c", "source_quality": 3, "target_quality": 3 }
        ],
        "arcs": [
            { "id": "a_b", "from": "wood_a", "to": "wood_b", "probability": 0.5 },
            { "id": "b_a", "from": "wood_b", "to": "wood_a", "probability": 0.5 },
            { "id": "b_c", "from": "wood_b", "to": "wood_c", "probability": 0.2 },
            { "id": "c_b", "from": "wood_c", "to": "wood_b", "probability": 0.2 }
        ],
        "vertices_improvements": [
            { "vertex_id": "wood_b", "option_id": "plantation",
              "source_quality_gain": 1, "target_quality_gain": 1 }
        ],
        "arcs_improvements": [
            { "arc_id": "a_b", "option_id": "hedgerow", "improved_probability": 0.9 },
            { "arc_id": "b_a", "option_id": "hedgerow", "improved_probability": 0.9 },
            { "arc_id": "b_c", "option_id": "underpass", "improved_probability": 0.8 },
            { "arc_id": "c_b", "option_id": "underpass", "improved_probability": 0.8 }
        ]
    }
})";

// three woods, with the given options and conflicts tables
Instance three_woods(const char * options, const char * conflicts = nullptr) {
    nlohmann::json instance_json;
    instance_json["options"] = nlohmann::json::parse(options);
    if(conflicts) instance_json["conflicts"] = nlohmann::json::parse(conflicts);
    instance_json["cases"] = nlohmann::json::parse(three_woods_cases);
    return parse_instance_json(instance_json);
}

const char * named_costs = R"([
    { "id": "hedgerow", "cost": { "money": 2, "area": 1 } },
    { "id": "underpass", "cost": { "money": 3, "area": 0 } },
    { "id": "plantation", "cost": { "money": 2, "area": 4 } }
])";
const char * conflicts = R"([
    { "set_id": "site", "option_id": "underpass" },
    { "set_id": "site", "option_id": "plantation" },
    { "set_id": "pair", "option_id": "hedgerow" },
    { "set_id": "pair", "option_id": "underpass" }
])";

// the parsed instances of test/instances/conflicts and of its inline twin
void expect_three_woods_conflicts(const Instance & instance) {
    ASSERT_EQ(instance.num_cost_dimensions(), 2u);
    // nlohmann's objects order the dimensions by name
    EXPECT_EQ(instance.cost_dimension_name(0), "area");
    EXPECT_EQ(instance.cost_dimension_name(1), "money");
    const auto hedgerow = instance.option_from_name("hedgerow");
    const auto underpass = instance.option_from_name("underpass");
    const auto plantation = instance.option_from_name("plantation");
    EXPECT_EQ(instance.option_cost(hedgerow, 0), 1.0);
    EXPECT_EQ(instance.option_cost(hedgerow, 1), 2.0);
    EXPECT_EQ(instance.option_cost(underpass, 0), 0.0);
    EXPECT_EQ(instance.option_cost(plantation, 1), 2.0);
    EXPECT_THROW(std::ignore = instance.option_cost(hedgerow),
                 std::logic_error);
    using sets = std::vector<std::vector<gecot::option_t>>;
    EXPECT_EQ(instance.conflict_sets(),
              (sets{{underpass, plantation}, {hedgerow, underpass}}));
    EXPECT_EQ(instance.conflict_set_name(0), "site");
    EXPECT_EQ(instance.conflict_set_name(1), "pair");
}

}  // namespace

GTEST_TEST(parse_instance, cost_dimensions_and_conflicts) {
    expect_three_woods_conflicts(three_woods(named_costs, conflicts));
    expect_three_woods_conflicts(parse_instance(
        PROJECT_SOURCE_DIR "/test/instances/conflicts/instance.json"));

    // a plain cost is one dimension named cost, and no conflict set
    const Instance plain = three_woods(R"([
        { "id": "hedgerow", "cost": 2 },
        { "id": "underpass", "cost": 3 },
        { "id": "plantation", "cost": 2 }
    ])");
    ASSERT_EQ(plain.num_cost_dimensions(), 1u);
    EXPECT_EQ(plain.cost_dimension_name(0), "cost");
    EXPECT_EQ(plain.option_cost(1), 3.0);
    EXPECT_EQ(plain.option_cost(1, 0), 3.0);
    EXPECT_TRUE(plain.conflict_sets().empty());
}

GTEST_TEST(parse_instance, cost_dimensions_and_conflicts_errors) {
    // a number after named costs, and the other way around
    EXPECT_THROW(three_woods(R"([
        { "id": "hedgerow", "cost": { "money": 2 } },
        { "id": "underpass", "cost": 3 },
        { "id": "plantation", "cost": { "money": 2 } }
    ])"),
                 std::invalid_argument);
    EXPECT_THROW(three_woods(R"([
        { "id": "hedgerow", "cost": 2 },
        { "id": "underpass", "cost": { "money": 3 } },
        { "id": "plantation", "cost": 2 }
    ])"),
                 std::invalid_argument);
    // a dimension missing, then one too many
    EXPECT_THROW(three_woods(R"([
        { "id": "hedgerow", "cost": { "money": 2, "area": 1 } },
        { "id": "underpass", "cost": { "money": 3 } },
        { "id": "plantation", "cost": { "money": 2, "area": 4 } }
    ])"),
                 std::invalid_argument);
    EXPECT_THROW(three_woods(R"([
        { "id": "hedgerow", "cost": { "money": 2 } },
        { "id": "underpass", "cost": { "money": 3, "area": 0 } },
        { "id": "plantation", "cost": { "money": 2 } }
    ])"),
                 std::invalid_argument);
    // a dimension whose name the budget syntax cannot carry
    EXPECT_ANY_THROW(three_woods(R"([
        { "id": "hedgerow", "cost": { "a=b": 2 } },
        { "id": "underpass", "cost": { "a=b": 3 } },
        { "id": "plantation", "cost": { "a=b": 2 } }
    ])"));
    // an option twice in a set, and an unknown option
    EXPECT_THROW(three_woods(named_costs, R"([
        { "set_id": "site", "option_id": "underpass" },
        { "set_id": "site", "option_id": "underpass" }
    ])"),
                 std::invalid_argument);
    EXPECT_THROW(three_woods(named_costs, R"([
        { "set_id": "site", "option_id": "bridge" }
    ])"),
                 std::invalid_argument);
}

GTEST_TEST(parse_instance, budget) {
    const Instance plain = three_woods(R"([
        { "id": "hedgerow", "cost": 2 },
        { "id": "underpass", "cost": 3 },
        { "id": "plantation", "cost": 2 }
    ])");
    EXPECT_EQ(parse_budget(plain, "10").values, std::vector<double>{10.0});
    EXPECT_EQ(parse_budget(plain, " 2.5 ").values, std::vector<double>{2.5});
    EXPECT_EQ(parse_budget(plain, "1e2").values, std::vector<double>{100.0});
    EXPECT_EQ(parse_budget(plain, "cost=3").values, std::vector<double>{3.0});
    // CLI11 read these as 105, 1000 and 25
    for(const char * text : {"10,5", "1,000", "2,5", "-1", "abc", "1e400",
                             "nan", "", "cost=", "price=3"})
        EXPECT_THROW(parse_budget(plain, text), std::invalid_argument) << text;

    const Instance named = three_woods(named_costs, conflicts);
    EXPECT_EQ(parse_budget(named, "money=5,area=3").values,
              (std::vector<double>{3.0, 5.0}));
    for(const char * text : {"5", "money=5", "money=5,area=3,money=1",
                             "money=5,area=3,cash=1", "money=5;area=3"})
        EXPECT_THROW(parse_budget(named, text), std::invalid_argument) << text;
}

GTEST_TEST(parse_instance, is_feasible) {
    const Instance instance = three_woods(named_costs, conflicts);
    auto solution = [&](std::vector<std::string> names) {
        auto s = instance.create_option_map<bool>(false);
        for(auto && name : names) s[instance.option_from_name(name)] = true;
        return s;
    };
    const gecot::budget_t budget{{5.0, 4.0}};  // area, money
    EXPECT_TRUE(gecot::is_feasible(instance, solution({}), budget, 0.0));
    EXPECT_TRUE(gecot::is_feasible(
        instance, solution({"hedgerow", "plantation"}), budget, 0.0));
    EXPECT_FALSE(gecot::is_feasible(
        instance, solution({"hedgerow", "plantation"}), {{4.0, 4.0}}, 0.0));
    EXPECT_TRUE(gecot::is_feasible(
        instance, solution({"hedgerow", "plantation"}), {{4.0, 4.0}}, 1.0));
    // within both budgets, but two options of the set 'site'
    EXPECT_FALSE(gecot::is_feasible(
        instance, solution({"underpass", "plantation"}), {{9.0, 9.0}}, 0.0));
    EXPECT_EQ(gecot::compute_solution_cost(
                  instance, solution({"hedgerow", "plantation"}), 1),
              4.0);
}
