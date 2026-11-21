#include <gtest/gtest.h>
#include <cmath>
#include <ranges>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <tbb/global_control.h>

#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/solvers/greedy.hpp"
#include "gecot/solvers/multiflow_target_block.hpp"
#include "gecot/solvers/static.hpp"

#include "mippp/model_entities.hpp"

#include "conflict_instances.hpp"
#include "instance.hpp"
#include "instance_builder.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

using option_names = std::set<std::string>;

// Mirrors gecot optimize: the options bought statically are listed too.
option_names solve_file(auto solver, const std::string & path,
                        const std::string & budget_text) {
    const Instance raw_instance = parse_instance(path);
    const gecot::budget_t budget = parse_budget(raw_instance, budget_text);
    const auto reformulated =
        trivial_reformulate_instance(raw_instance, budget);
    const Instance & instance = reformulated.instance;
    const auto solution = solver.solve(instance, budget);
    option_names taken;
    for(auto && option : raw_instance.options()) {
        const auto & name = raw_instance.option_name(option);
        if(reformulated.static_options[option] ||
           (instance.contains_option(name) &&
            solution[instance.option_from_name(name)]))
            taken.insert(name);
    }
    return taken;
}

option_names solve_aude(auto solver, double budget) {
    return solve_file(solver,
                      PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json",
                      std::to_string(budget));
}

// On every thread of the machine: where the work is cut must not show in
// the last digits of a score, that choose between two options tied for a
// heuristic.
GTEST_TEST(solvers, a_score_is_the_same_at_every_evaluation) {
    const Instance instance = parse_instance(
        PROJECT_SOURCE_DIR "/test/instances/Aix/instance_30x30/instance.json");
    const auto & squirrel = *instance.cases().begin();
    const auto score = [&squirrel] {
        return gecot::parallel_pc_num(
            squirrel.graph(), squirrel.source_quality_map(),
            squirrel.target_quality_map(), squirrel.arc_probability_map());
    };
    const double first = score();
    for(int evaluation = 0; evaluation < 50; ++evaluation)
        ASSERT_EQ(score(), first);
}

// 32 cases of one vertex of quality 0.1, each with two options, of gains 0.3
// and 0.5. Restored by the inverse operation, (0.1 + 0.3) - 0.3 is not 0.1:
// the second option of a case, evaluated after the first in a block of the
// parallel loop, would be scored from that quality, so its score would
// follow how the threads cut the loop. With few options a block holds one,
// and in a case of several vertices the sum of the score hides the digit.
GTEST_TEST(solvers, an_option_scores_the_same_after_any_other) {
    const std::size_t num_cases = 32;
    std::vector<case_spec> cases;
    for(gecot::option_t i = 0; i < 2 * num_cases; i += 2)
        cases.push_back(case_spec{
            1, {}, {}, {0.1}, {0.1}, {{{0.3, 0.3, i}, {0.5, 0.5, i + 1}}}, {}});
    criterion_sum criterion;
    for(gecot::case_id_t k = 0; k < num_cases; ++k)
        criterion.values.emplace_back(k);
    const Instance instance = build_instance(
        std::vector<double>(2 * num_cases, 1.0), cases, criterion);
    const tbb::global_control one_thread(
        tbb::global_control::max_allowed_parallelism, 1);
    const auto all = std::ranges::to<std::vector>(instance.options());
    const auto solution = instance.create_option_map(true);
    gecot::solvers::heuristic_landscape landscape(instance);
    auto scores = [&](const bool with,
                      const std::vector<gecot::option_t> & options) {
        auto options_cases_pc_num =
            instance.create_option_map(instance.create_case_map<double>());
        if(with)
            landscape.scores_with(options, options_cases_pc_num);
        else
            landscape.scores_without(solution, options, options_cases_pc_num);
        return options_cases_pc_num;
    };
    for(const bool with : {true, false}) {
        if(with)
            landscape.set(std::vector<gecot::option_t>{});
        else
            landscape.set(all);
        const auto together = scores(with, all);
        for(const gecot::option_t i : all)
            EXPECT_EQ(together[i][i / 2], scores(with, {i})[i][i / 2])
                << (with ? "with " : "without ") << i;
    }
}

// Two cases of two woods, each with the option that improves its arcs, and
// the criterion z_0 - z_1: the option of the case 1 lowers the score, and
// both fit in the budget.
GTEST_TEST(solvers, no_option_that_lowers_the_score) {
    auto improvable = [](const gecot::option_t i) {
        return case_spec{
            2,  {{0, 1}, {1, 0}},        {0.5, 0.5}, {1.0, 1.0}, {1.0, 1.0},
            {}, {{{0.9, i}}, {{0.9, i}}}};
    };
    const Instance instance = build_instance(
        {1.0, 1.0}, {improvable(0), improvable(1)},
        criterion_sum{
            {criterion_var{0}, criterion_product{{-1.0, criterion_var{1}}}}});
    const gecot::budget_t budget{{2}};
    auto bought = [&](const auto & solver) {
        const auto solution = solver.solve(instance, budget);
        return std::vector<bool>{solution[0], solution[1]};
    };
    using gecot::solvers::Greedy, gecot::solvers::Static;
    const std::vector<bool> first_only{true, false};
    EXPECT_EQ(bought(Greedy{}), first_only);
    EXPECT_EQ(bought(Static{}), first_only);
    // the cost of both fits: the second is removed for the score
    EXPECT_EQ(bought(Greedy{.decremental = true}), first_only);
    EXPECT_EQ(bought(Static{.decremental = true}), first_only);
    EXPECT_EQ(bought(Greedy{.decremental = true, .only_dec = true}),
              first_only);
    EXPECT_EQ(bought(Static{.decremental = true, .only_dec = true}),
              first_only);
    // the reference of the worst choices spends the budget
    EXPECT_EQ(bought(Greedy{.worst = true}), (std::vector<bool>{true, true}));
}

// The criterion z_0 - z_1, z_0 of 1e6. Option 0 (cost 1000) adds 0.04 to
// z_1, option 1 (cost 0.01) 5e-7 to z_0, option 2 (cost 1e4) 0.4 to z_0:
// the ratio of 0 is tied with that of 1, the best, whose window grows with
// 1 / its cost. The tie takes the lowest id, 0, which lowers the score.
GTEST_TEST(solvers, greedy_takes_no_tie_that_lowers_the_score) {
    const Instance instance = build_instance(
        {1000.0, 0.01, 1e4},
        {case_spec{1,
                   {},
                   {},
                   {1000.0},
                   {1000.0},
                   {{{2.5e-10, 2.5e-10, 1}, {2e-4, 2e-4, 2}}},
                   {}},
         case_spec{1, {}, {}, {0.0}, {0.0}, {{{0.2, 0.2, 0}}}, {}}},
        criterion_sum{
            {criterion_var{0}, criterion_product{{-1.0, criterion_var{1}}}}});
    const auto solution =
        gecot::solvers::Greedy{}.solve(instance, gecot::budget_t{{20000}});
    EXPECT_EQ((std::vector<bool>{solution[0], solution[1], solution[2]}),
              (std::vector<bool>{false, true, true}));
}

// The criterion z_0 - z_1, z_0 of 1e6, and z_1 the sum of two vertices:
// option 0 (cost 5e-7) adds 9e-7 to it, option 1 (cost 1) adds 1. Both fit.
// The smallest loss per unit of cost is that of 0, -1.8, inside the tie
// window of the score: removing it raises the score by less than the window,
// removing 1 by 1.
GTEST_TEST(solvers, decremental_removes_what_raises_the_score) {
    const Instance instance = build_instance(
        {5e-7, 1.0},
        {case_spec{1, {}, {}, {1000.0}, {1000.0}, {}, {}},
         case_spec{2,
                   {},
                   {},
                   {0.0, 0.0},
                   {0.0, 0.0},
                   {{{std::sqrt(9e-7), std::sqrt(9e-7), 0}}, {{1.0, 1.0, 1}}},
                   {}}},
        criterion_sum{
            {criterion_var{0}, criterion_product{{-1.0, criterion_var{1}}}}});
    const gecot::budget_t budget{{2}};
    auto bought = [&](const auto & solver) {
        const auto solution = solver.solve(instance, budget);
        return std::vector<bool>{solution[0], solution[1]};
    };
    using gecot::solvers::Greedy, gecot::solvers::Static;
    const std::vector<bool> first_only{true, false};
    EXPECT_EQ(bought(Greedy{.decremental = true, .only_dec = true}),
              first_only);
    EXPECT_EQ(bought(Static{.decremental = true, .only_dec = true}),
              first_only);
}

GTEST_TEST(solvers, static_incremental) {
    const gecot::solvers::Static s;
    EXPECT_EQ(solve_aude(s, 3),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou"}));
    EXPECT_EQ(solve_aude(s, 7),
              (option_names{"AGU-Sou", "AIG-Pou", "AUD-Car_BRU-Uss",
                            "AUD-Far_AUD-Fou", "AUD-Puy_AUD-Car", "BRU-Uss",
                            "QUE-Mas_QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AGU-Sou", "AIG-Pou", "AUD-Car_BRU-Uss",
                            "AUD-Far_AUD-Fou", "AUD-Fou_AUD-Ser_1",
                            "AUD-Fou_AUD-Ser_2", "AUD-Puy_AUD-Car", "BRU-Uss",
                            "QUE-Mas_QUE-Ria", "QUE-Ria", "RT-Lau"}));
}

GTEST_TEST(solvers, static_decremental) {
    const gecot::solvers::Static s{.decremental = true};
    EXPECT_EQ(solve_aude(s, 3),
              (option_names{"AUD-Far_AUD-Fou", "BRU-Uss", "QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 7),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou",
                            "AUD-Fou_AUD-Ser_1", "AUD-Fou_AUD-Ser_2", "BRU-Uss",
                            "QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou",
                            "AUD-Fou_AUD-Ser_1", "AUD-Fou_AUD-Ser_2",
                            "BRU-Mij_1", "BRU-Mij_2", "BRU-Mij_3", "BRU-Uss",
                            "QUE-Mas_QUE-Ria", "QUE-Ria"}));
}

GTEST_TEST(solvers, greedy_incremental) {
    const gecot::solvers::Greedy s;
    EXPECT_EQ(solve_aude(s, 3),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou"}));
    EXPECT_EQ(solve_aude(s, 7),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou",
                            "AUD-Puy_AUD-Car", "BRU-Uss", "QUE-Mas_QUE-Ria",
                            "QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AGU-Sou", "AIG-Pou", "AUD-Car_BRU-Uss",
                            "AUD-Far_AUD-Fou", "AUD-Fou_AUD-Ser_1",
                            "AUD-Puy_AUD-Car", "BRU-Uss", "QUE-Mas_QUE-Ria",
                            "QUE-Ria", "RT-Lau", "RT-Lau_QUE-Ria"}));
}

GTEST_TEST(solvers, greedy_decremental) {
    const gecot::solvers::Greedy s{.decremental = true};
    EXPECT_EQ(solve_aude(s, 3), (option_names{"AIG-Pou", "AUD-Fou_AUD-Ser_1",
                                              "AUD-Fou_AUD-Ser_2"}));
    EXPECT_EQ(solve_aude(s, 7),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou",
                            "AUD-Fou_AUD-Ser_1", "AUD-Fou_AUD-Ser_2", "BRU-Uss",
                            "QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AIG-Pou", "AUD-Car_BRU-Uss", "AUD-Far_AUD-Fou",
                            "AUD-Fou_AUD-Ser_1", "AUD-Fou_AUD-Ser_2",
                            "BRU-Mij_1", "BRU-Mij_2", "BRU-Mij_3", "BRU-Uss",
                            "QUE-Mas_QUE-Ria", "QUE-Ria"}));
}

// without decremental, only_dec must not skip the purchases
GTEST_TEST(solvers, only_dec_without_decremental) {
    EXPECT_EQ(solve_aude(gecot::solvers::Static{.only_dec = true}, 7),
              solve_aude(gecot::solvers::Static{}, 7));
    EXPECT_EQ(solve_aude(gecot::solvers::Greedy{.only_dec = true}, 7),
              solve_aude(gecot::solvers::Greedy{}, 7));
}

// the two worst greedy first differ at budget 11
GTEST_TEST(solvers, worst_greedy_incremental) {
    const gecot::solvers::Greedy s{.worst = true};
    EXPECT_EQ(solve_aude(s, 3),
              (option_names{"BRU-Mij_2", "BRU-Mij_3", "RT-Lau_QUE-Ria"}));
    EXPECT_EQ(
        solve_aude(s, 7),
        (option_names{"AGU-Sou", "AUD-Fou_AUD-Ser_1", "BRU-Mij_1", "BRU-Mij_2",
                      "BRU-Mij_3", "RT-Lau", "RT-Lau_QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AGU-Sou", "AUD-Fou_AUD-Ser_1", "AUD-Fou_AUD-Ser_2",
                            "AUD-Puy_AUD-Car", "BRU-Mij_1", "BRU-Mij_2",
                            "BRU-Mij_3", "BRU-Uss", "QUE-Mas_QUE-Ria", "RT-Lau",
                            "RT-Lau_QUE-Ria"}));
}

GTEST_TEST(solvers, worst_greedy_decremental) {
    const gecot::solvers::Greedy s{.decremental = true, .worst = true};
    EXPECT_EQ(solve_aude(s, 3),
              (option_names{"BRU-Mij_2", "BRU-Mij_3", "RT-Lau_QUE-Ria"}));
    EXPECT_EQ(
        solve_aude(s, 7),
        (option_names{"AGU-Sou", "AUD-Fou_AUD-Ser_1", "BRU-Mij_1", "BRU-Mij_2",
                      "BRU-Mij_3", "RT-Lau", "RT-Lau_QUE-Ria"}));
    EXPECT_EQ(solve_aude(s, 11),
              (option_names{"AGU-Sou", "AUD-Car_BRU-Uss", "AUD-Fou_AUD-Ser_1",
                            "AUD-Fou_AUD-Ser_2", "AUD-Puy_AUD-Car", "BRU-Mij_1",
                            "BRU-Mij_2", "BRU-Mij_3", "QUE-Mas_QUE-Ria",
                            "RT-Lau", "RT-Lau_QUE-Ria"}));
}

namespace {

// Three vertices of quality 1 and the arcs of probability 0.1 from the last
// two to the first, that o0 and o1, of cost 1, improve to the given
// probabilities: the score is 3.2 before them.
Instance two_arcs(const double o0_probability, const double o1_probability) {
    const case_spec c{3,
                      {{1, 0}, {2, 0}},
                      {0.1, 0.1},
                      {1, 1, 1},
                      {1, 1, 1},
                      {},
                      {{{o0_probability, 0}}, {{o1_probability, 1}}}};
    return build_instance({1, 1}, {c}, criterion_var{0});
}

std::vector<gecot::option_t> bought(const gecot::solvers::Greedy & solver,
                                    const Instance & instance) {
    const auto solution = solver.solve(instance, gecot::budget_t{{1}});
    std::vector<gecot::option_t> options;
    for(auto && o : instance.options())
        if(solution[o]) options.push_back(o);
    return options;
}

}  // namespace

// The gains differ by 5e-14, which a strict comparison sees at every run on
// these scores of 3.2: the size of the rounding that, on the scores of a
// landscape, makes it choose one option or the other from run to run.
GTEST_TEST(solvers, greedy_gives_a_tie_to_the_first_option) {
    using gecot::solvers::Greedy;
    using options = std::vector<gecot::option_t>;
    const double more = 0.5 * (1 + 1e-13), less = 0.5 * (1 - 1e-13);
    EXPECT_EQ(bought(Greedy{}, two_arcs(0.5, more)), options{0});
    EXPECT_EQ(bought(Greedy{.worst = true}, two_arcs(0.5, less)), options{0});
    // removed first: o1 stays
    EXPECT_EQ(bought(Greedy{.decremental = true}, two_arcs(0.5, less)),
              options{1});
    EXPECT_EQ(
        bought(Greedy{.decremental = true, .worst = true}, two_arcs(0.5, more)),
        options{1});
    // o0 gains nothing, under the probability of its arc: a window relative
    // to the ratios would have no width here
    EXPECT_EQ(bought(Greedy{}, two_arcs(0.05, 0.1 + 1e-14)), options{0});
    // one millionth more is not a tie
    EXPECT_EQ(bought(Greedy{}, two_arcs(0.5, 0.5 * (1 + 1e-6))), options{1});
}

// Captured from main e2585fc, before the heuristics took several costs and
// conflict sets: unequal costs, where a change of the ratios would show.
GTEST_TEST(solvers, selections_with_unequal_costs) {
    using gecot::solvers::Greedy;
    using gecot::solvers::Static;
    const std::string aix =
        PROJECT_SOURCE_DIR "/test/instances/Aix/instance_30x30/instance.json";
    EXPECT_EQ(solve_file(Static{}, aix, "4"),
              (option_names{"112", "113", "122", "210", "214", "216"}));
    EXPECT_EQ(solve_file(Static{.decremental = true}, aix, "4"),
              (option_names{"122", "201", "209", "210", "216"}));
    EXPECT_EQ(solve_file(Greedy{}, aix, "4"),
              (option_names{"113", "122", "201", "210", "216"}));
    EXPECT_EQ(solve_file(Greedy{.decremental = true}, aix, "4"),
              (option_names{"112", "120", "121", "122"}));
    EXPECT_EQ(solve_file(Greedy{.worst = true}, aix, "4"),
              (option_names{"127", "131", "132", "137"}));
    EXPECT_EQ(solve_file(Greedy{.decremental = true, .worst = true}, aix, "4"),
              (option_names{"112", "113", "131", "137"}));
    const std::string woods =
        PROJECT_SOURCE_DIR "/docs/examples/three-woods.json";
    EXPECT_EQ(solve_file(Greedy{}, woods, "3"), option_names{"plantation"});
    EXPECT_EQ(solve_file(Greedy{.worst = true}, woods, "3"),
              option_names{"hedgerow"});
    EXPECT_EQ(solve_file(Static{}, woods, "4"),
              (option_names{"hedgerow", "plantation"}));
    EXPECT_EQ(
        solve_file(Greedy{.decremental = true, .only_dec = true}, woods, "4"),
        option_names{"plantation"});
    EXPECT_EQ(solve_file(Greedy{.worst = true}, woods, "5"),
              (option_names{"hedgerow", "underpass"}));
}

GTEST_TEST(solvers, decremental_refuses_conflict_sets_and_cost_dimensions) {
    using gecot::solvers::Greedy;
    using gecot::solvers::Static;
    const Instance conflict = three_woods_conflict();
    const Instance two_costs = parse_instance(
        PROJECT_SOURCE_DIR "/test/instances/conflicts/instance.json");
    for(const Instance * instance : {&conflict, &two_costs}) {
        const gecot::budget_t budget{
            std::vector<double>(instance->num_cost_dimensions(), 5.0)};
        for(const Greedy greedy :
            {Greedy{.decremental = true},
             Greedy{.decremental = true, .only_dec = true},
             Greedy{.decremental = true, .worst = true}})
            EXPECT_THROW(std::ignore = greedy.solve(*instance, budget),
                         std::invalid_argument);
        EXPECT_THROW(
            std::ignore = Static{.decremental = true}.solve(*instance, budget),
            std::invalid_argument);
        EXPECT_NO_THROW(std::ignore = Greedy{}.solve(*instance, budget));
        EXPECT_NO_THROW(std::ignore = Static{}.solve(*instance, budget));
    }
    // the underpass over the budget, the set constrains nothing
    const Instance vacuous =
        trivial_reformulate_instance(conflict, gecot::budget_t{{2}}).instance;
    EXPECT_NO_THROW(std::ignore = Greedy{.decremental = true}.solve(
                        vacuous, gecot::budget_t{{2}}));
}

GTEST_TEST(solvers, incremental_with_conflict_sets_and_cost_dimensions) {
    using gecot::solvers::Greedy;
    using gecot::solvers::Static;
    const std::string examples = PROJECT_SOURCE_DIR "/docs/examples/";
    // greedy gives the harm back through a swap, static keeps the wood
    EXPECT_EQ(solve_file(Greedy{}, examples + "three-woods-works.json", "7"),
              (option_names{"hedgerow", "plantation", "underpass"}));
    EXPECT_EQ(solve_file(Static{}, examples + "three-woods-works.json", "7"),
              (option_names{"hedgerow", "plantation", "wood_c_intact"}));
    EXPECT_EQ(
        solve_file(Greedy{}, examples + "three-woods-alternatives.json", "7"),
        (option_names{"forest", "underpass"}));
    EXPECT_EQ(solve_file(Static{}, examples + "three-woods-resources.json",
                         "money=7,area=2,crossings=1"),
              (option_names{"plantation", "underpass"}));
    // single swaps miss {hedgerow, plantation}, that --worst finds
    const std::string conflicts =
        PROJECT_SOURCE_DIR "/test/instances/conflicts/instance.json";
    EXPECT_EQ(solve_file(Greedy{}, conflicts, "money=7,area=5"),
              option_names{"underpass"});
    EXPECT_EQ(solve_file(Greedy{.worst = true}, conflicts, "money=7,area=5"),
              (option_names{"hedgerow", "plantation"}));
    // a budget of 0 in the first dimension: the costs are in money
    EXPECT_NO_THROW(solve_file(Greedy{}, conflicts, "money=7,area=0"));
}

namespace {
// a free option b in a set with each of a1 and a2, which add up; or, with
// layered, two free options in a set each with a single paid option
Instance free_member_instance(bool layered) {
    case_spec spec{3, {}, {}, {1, 0, 0}, {1, 0, 0}, {{}, {}, {}}, {}};
    if(!layered) {
        spec.vertex_options = {
            {{1.0, 1.0, 0}}, {{3.0, 3.0, 1}}, {{3.0, 3.0, 2}}};
        return build_instance({"cost"}, {{0}, {1}, {1}}, {{0, 1}, {0, 2}},
                              {spec}, criterion_var{0});
    }
    spec.vertex_options = {
        {{1.0, 1.0, 0}, {1.0, 1.0, 1}}, {{10.0, 10.0, 2}}, {}};
    return build_instance({"cost"}, {{0}, {0}, {1}}, {{0, 2}, {1, 2}}, {spec},
                          criterion_var{0});
}
std::vector<gecot::option_t> greedy_bought(const Instance & instance,
                                           double budget) {
    const auto solution =
        gecot::solvers::Greedy{}.solve(instance, gecot::budget_t{{budget}});
    std::vector<gecot::option_t> bought;
    for(auto && o : instance.options())
        if(solution[o]) bought.push_back(o);
    return bought;
}
}  // namespace

GTEST_TEST(solvers, greedy_swaps_then_buys_again) {
    using options = std::vector<gecot::option_t>;
    // b first, as it is free, then swapped for a1, which frees a2
    const Instance instance = free_member_instance(false);
    EXPECT_EQ(greedy_bought(instance, 2), (options{1, 2}));
    EXPECT_DOUBLE_EQ(brute_force_optimum(instance, 2), 19);
}

// a limit of single swaps: the paid option is in a set with each of two
// bought free options, and greedy never reaches it
GTEST_TEST(solvers, greedy_misses_an_option_behind_two_free_ones) {
    using options = std::vector<gecot::option_t>;
    const Instance instance = free_member_instance(true);
    EXPECT_EQ(greedy_bought(instance, 1), (options{0, 1}));
    EXPECT_DOUBLE_EQ(brute_force_optimum(instance, 1), 101);
}

GTEST_TEST(solvers, incremental_heuristics_stay_feasible_and_maximal) {
    std::mt19937 rng(29);
    int free_members = 0, zero_budgets = 0;
    for(int round = 0; round < 300; ++round) {
        auto [raw, budget] = random_conflicts_instance(rng);
        const Instance instance =
            trivial_reformulate_instance(raw, budget).instance;
        for(std::size_t r = 0; r < budget.size(); ++r)
            zero_budgets += budget[r] == 0;
        for(auto && set : instance.conflict_sets())
            for(auto && o : set)
                free_members += instance.option_cost(o, 0) == 0 &&
                                instance.option_cost(o, 1) == 0;
        const double optimum = brute_force_optimum(instance, budget);
        const auto score = [&](const auto & solution) {
            return instance.eval_criterion(
                gecot::compute_solution_cases_pc_num(instance, solution));
        };
        const auto blocked = [&](const auto & solution, gecot::option_t o) {
            return std::ranges::any_of(
                instance.conflict_sets(), [&](auto && set) {
                    return std::ranges::contains(set, o) &&
                           std::ranges::any_of(set, [&](auto && m) {
                               return m != o && solution[m];
                           });
                });
        };
        const auto fits = [&](const auto & solution, gecot::option_t o) {
            for(std::size_t r = 0; r < budget.size(); ++r)
                if(gecot::compute_solution_cost(instance, solution, r) +
                       instance.option_cost(o, r) >
                   budget[r] + 1e-7)
                    return false;
            return true;
        };
        const auto check = [&](const auto & solver, bool swaps) {
            const auto solution = solver.solve(instance, budget);
            ASSERT_TRUE(gecot::is_feasible(instance, solution, budget, 1e-7))
                << "round " << round;
            const double s = score(solution);
            EXPECT_LE(s, optimum + 1e-9 * std::max(1.0, optimum));
            for(auto && o : instance.options())
                EXPECT_TRUE(solution[o] || blocked(solution, o) ||
                            !fits(solution, o))
                    << "round " << round << ": option " << o << " fits";
            if(!swaps) return;
            auto swapped = solution;
            for(auto && set : instance.conflict_sets())
                for(auto && o : set)
                    for(auto && m : set) {
                        if(!solution[o] || solution[m]) continue;
                        swapped[o] = false;
                        swapped[m] = true;
                        if(gecot::is_feasible(instance, swapped, budget,
                                              1e-7)) {
                            EXPECT_LE(score(swapped),
                                      s + 1e-9 * std::max(1.0, std::abs(s)))
                                << "round " << round << ": swap " << o
                                << " for " << m;
                        }
                        swapped[o] = true;
                        swapped[m] = false;
                    }
        };
        check(gecot::solvers::Greedy{.feasibility_tol = 1e-7}, true);
        check(gecot::solvers::Greedy{.feasibility_tol = 1e-7, .worst = true},
              false);
        check(gecot::solvers::Static{.feasibility_tol = 1e-7}, false);
    }
    EXPECT_GT(free_members, 0);
    EXPECT_GT(zero_budgets, 0);
}

namespace {
// The cut that the block of each vertex of each case separates at each
// scenario and at two fractional points is at least the flow that reaches
// the vertex in every scenario, and its value is that of the cut at the
// point separated. At a scenario that value is the flow. Returns how many of
// the cuts stopped the flow of a source at the limit of its pushes.
template <std::size_t MaxPushes>
std::size_t expect_valid_cuts(const std::string & path, const bool rebuild) {
    const Instance instance = parse_instance(path);
    const std::size_t n = instance.options().size();
    const std::size_t num_scenarios = std::size_t{1} << n;
    double all_costs = 0;
    for(auto && i : instance.options()) all_costs += instance.option_cost(i);
    const gecot::budget_t budget{{all_costs}};  // every scenario fits
    auto point = [n](const std::size_t scenario) {
        std::vector<double> y(n);
        for(std::size_t i = 0; i < n; ++i)
            y[i] = static_cast<double>(scenario >> i & 1);
        return y;
    };
    std::vector<std::vector<double>> points;
    for(std::size_t scenario = 0; scenario < num_scenarios; ++scenario)
        points.push_back(point(scenario));
    points.emplace_back(n, 0.5);
    points.emplace_back(n);
    for(std::size_t i = 0; i < n; ++i)
        points.back()[i] =
            static_cast<double>(i + 1) / static_cast<double>(n + 1);
    std::size_t num_unsettled = 0;
    for(const auto & instance_case : instance.cases()) {
        const auto & graph = instance_case.graph();
        const auto [strong_arcs_map, useless_arcs_map] =
            gecot::compute_strong_and_useless_arcs(
                instance_case, [](const gecot::option_t &) { return true; },
                1e-8, true);
        for(const auto & t : melon::vertices(graph)) {
            // the flow that reaches t in each scenario, on the case's graph
            std::vector<double> flows;
            for(std::size_t scenario = 0; scenario < num_scenarios;
                ++scenario) {
                auto source_quality_map = instance_case.source_quality_map();
                auto probability_map = instance_case.arc_probability_map();
                for(const auto & u : melon::vertices(graph))
                    for(const auto & [source_gain, target_gain, i] :
                        instance_case.vertex_options_map()[u])
                        if(scenario >> i & 1)
                            source_quality_map[u] += source_gain;
                for(const auto & a : melon::arcs(graph))
                    for(const auto & [improved_probability, i] :
                        instance_case.arc_options_map()[a])
                        if(scenario >> i & 1)
                            probability_map[a] = std::max(probability_map[a],
                                                          improved_probability);
                flows.push_back(gecot::pc_num_vertex_in_flow(
                    graph, source_quality_map, probability_map, t));
            }
            gecot::solvers::multiflow_target_block<
                mippp::model_variable<int, double>, MaxPushes>
                block(instance_case, t,
                      gecot::compute_contracted_flow_graph<false>(
                          instance, instance_case, budget, t,
                          strong_arcs_map[t], useless_arcs_map[t],
                          gecot::big_M_kind::plain, 1e-7, true));
            block.compute_sources(!rebuild);
            for(std::size_t p = 0; p < points.size(); ++p) {
                const auto cut = block.separate(points[p], rebuild);
                num_unsettled += !cut.settled;
                auto cut_at = [&](const std::vector<double> & x) {
                    double value = cut.constant;
                    for(std::size_t i = 0; i < n; ++i)
                        value += cut.coefficients[i] * x[i];
                    return value;
                };
                const double tol = 1e-9 * std::max(1.0, flows.back());
                for(std::size_t scenario = 0; scenario < num_scenarios;
                    ++scenario)
                    EXPECT_GE(cut_at(point(scenario)), flows[scenario] - tol)
                        << path << ": vertex " << t << ", point " << p
                        << ", scenario " << scenario;
                if(p < num_scenarios) {
                    EXPECT_NEAR(cut.value, flows[p], tol)
                        << path << ": vertex " << t << ", scenario " << p;
                }
                EXPECT_NEAR(cut_at(points[p]), cut.value, tol)
                    << path << ": vertex " << t << ", point " << p;
            }
        }
    }
    return num_unsettled;
}
}  // namespace

// MaxPushes 0: every source that has something to send stops before its
// first push, and 1: every source whose flow takes two paths, which only a
// fractional point asks for. No run of the benchmark reaches that limit.
GTEST_TEST(solvers, multiflow_cuts_are_valid_and_tight) {
    std::size_t gave_up_at_0 = 0, gave_up_at_1 = 0;
    for(const std::string name : {"/test/instances/test/instance.json",
                                  "/docs/examples/three-woods.json",
                                  "/docs/examples/two-species.json"}) {
        const std::string path = PROJECT_SOURCE_DIR + name;
        for(const bool rebuild : {false, true}) {
            EXPECT_EQ(expect_valid_cuts<100000>(path, rebuild), 0u) << path;
            gave_up_at_0 += expect_valid_cuts<0>(path, rebuild);
            gave_up_at_1 += expect_valid_cuts<1>(path, rebuild);
        }
    }
    EXPECT_GT(gave_up_at_0, 0u);
    EXPECT_GT(gave_up_at_1, 0u);
}

// make_groups splits the sources not affine into ⌈n / size⌉ groups, none
// empty, each source in one group or affine, each group within the sum of
// the bounds of its sources; and the parts of the groups with the fixed part
// add up to the cut, at the point separated and at another; the fixed part
// is the target's own quality with the flows of the affine sources, those of
// their parts in the cut. Voronoi puts together sources close in G_t: twice
// as many pairs of a group as random are adjacent or share a neighbor other
// than t. Aude is a river network; Quebec, under a budget of 1000, has
// sources with no improved copy, some of quality gains; on cover-40 every
// way has probability 1, the arcs of a way to a kernel then decide its cell,
// and no voronoi cell takes half the sources of the target.
GTEST_TEST(solvers, multiflow_source_groups_partition_the_sources) {
    using block_t =
        gecot::solvers::multiflow_target_block<mippp::model_variable<int, double>>;
    using grouping = gecot::solvers::source_grouping;
    // the paths that the test must cover
    std::size_t num_voronoi = 0, num_shared = 0, num_with_affine = 0;
    struct tested {
        std::string name;
        double budget;  // 0: every option
        bool balanced;  // voronoi cells under half the sources
    };
    for(const tested & tested_instance :
        {tested{"/test/instances/Aude/aude.json", 0, false},
         tested{"/test/instances/Quebec_438_RASY/instance.json", 1000, false},
         tested{"/benchmark/instances/cover-40/instance.json", 0, true}}) {
        const std::string path = PROJECT_SOURCE_DIR + tested_instance.name;
        const Instance instance = parse_instance(path);
        const std::size_t n = instance.options().size();
        double all_costs = 0;
        for(auto && i : instance.options()) all_costs += instance.option_cost(i);
        const gecot::budget_t budget{
            {tested_instance.budget > 0 ? tested_instance.budget : all_costs}};
        std::vector<double> x_hat(n), other(n);
        for(std::size_t i = 0; i < n; ++i) {
            x_hat[i] = static_cast<double>(i + 1) / static_cast<double>(n + 1);
            other[i] = static_cast<double>(i % 2);
        }
        // of each grouping, the pairs of a group close in G_t
        std::array<std::size_t, 3> close_pairs{};
        for(const auto & instance_case : instance.cases()) {
            const auto & graph = instance_case.graph();
            const auto [strong_arcs_map, useless_arcs_map] =
                gecot::compute_strong_and_useless_arcs(
                    instance_case,
                    [](const gecot::option_t &) { return true; }, 1e-8, true);
            for(const auto & t : melon::vertices(graph)) {
                block_t block(instance_case, t,
                              gecot::compute_contracted_flow_graph<false>(
                                  instance, instance_case, budget, t,
                                  strong_arcs_map[t], useless_arcs_map[t],
                                  gecot::big_M_kind::plain, 1e-7, true));
                block.compute_sources(true);
                std::size_t num_affine = 0;
                for(const auto & source : block.sources)
                    num_affine += source.improved_copies.empty();
                const std::size_t num_grouped =
                    block.sources.size() - num_affine;
                const auto cut = block.separate(x_hat, false, true);
                // each source with its neighbors other than t, sorted
                std::vector<std::vector<std::size_t>> around(
                    block.sources.size());
                for(std::size_t k = 0; k < block.sources.size(); ++k) {
                    const auto s = block.sources[k].s;
                    around[k].push_back(s);
                    for(const auto a : melon::out_arcs(block.graph, s))
                        around[k].push_back(melon::arc_target(block.graph, a));
                    for(const auto a : melon::in_arcs(block.graph, s))
                        around[k].push_back(melon::arc_source(block.graph, a));
                    std::erase(around[k], block.t);
                    std::ranges::sort(around[k]);
                }
                auto close = [&](const std::size_t k1, const std::size_t k2) {
                    std::vector<std::size_t> both;
                    std::ranges::set_intersection(around[k1], around[k2],
                                                  std::back_inserter(both));
                    return !both.empty();
                };
                for(const grouping method :
                    {grouping::voronoi, grouping::consecutive,
                     grouping::random})
                    for(const std::size_t size :
                        std::initializer_list<std::size_t>{1, 2, 3, 5, 16}) {
                        std::mt19937_64 rng{size};
                        block.make_groups(size, method, true, rng);
                        const std::size_t expected_groups =
                            (num_grouped + size - 1) / size;
                        if(expected_groups <= 1) {
                            EXPECT_TRUE(block.groups.empty());
                            EXPECT_TRUE(block.affine_sources.empty());
                            continue;
                        }
                        num_voronoi += method == grouping::voronoi &&
                                       expected_groups < num_grouped;
                        num_with_affine += num_affine > 0;
                        ASSERT_EQ(block.groups.size(), expected_groups);
                        EXPECT_EQ(block.affine_sources.size(), num_affine);
                        std::vector<int> seen(block.sources.size(), 0);
                        for(const std::size_t k : block.affine_sources) {
                            EXPECT_TRUE(
                                block.sources[k].improved_copies.empty());
                            ++seen[k];
                        }
                        std::size_t largest = 0;
                        for(const auto & group : block.groups) {
                            EXPECT_FALSE(group.members.empty());
                            for(std::size_t m1 = 0; m1 < group.members.size();
                                ++m1)
                                for(std::size_t m2 = m1 + 1;
                                    m2 < group.members.size(); ++m2)
                                    close_pairs[static_cast<std::size_t>(
                                        method)] += close(group.members[m1],
                                                          group.members[m2]);
                            num_shared += group.members.size() > 1;
                            largest = std::max(largest, group.members.size());
                            double bound = 0;
                            for(const std::size_t k : group.members) {
                                ++seen[k];
                                bound +=
                                    block.source_flow_bound(block.sources[k]);
                            }
                            EXPECT_NEAR(group.bound, bound,
                                        1e-12 * std::max(1.0, bound));
                        }
                        for(const int count : seen) EXPECT_EQ(count, 1);
                        if(method != grouping::voronoi) {
                            EXPECT_LE(largest, size);
                        }
                        if(tested_instance.balanced &&
                           method == grouping::voronoi) {
                            EXPECT_LT(2 * largest, num_grouped)
                                << path << ": size " << size;
                        }
                        const auto [constant, terms] = block.fixed_part();
                        // the target's own quality and the parts of the
                        // affine sources, option by option
                        double expected_constant =
                            block.source_quality_map[block.t];
                        std::vector<double> expected(n, 0.0), fixed(n, 0.0);
                        for(const auto & [source_gain, i] :
                            block.source_options_map[block.t])
                            expected[i] += source_gain;
                        for(const std::size_t k : block.affine_sources) {
                            expected_constant += cut.sources[k].constant;
                            for(const auto & [i, coefficient] :
                                cut.sources[k].coefficients)
                                expected[i] += coefficient;
                        }
                        for(const auto & [i, coefficient] : terms)
                            fixed[i] += coefficient;
                        EXPECT_NEAR(constant, expected_constant,
                                    1e-9 * std::abs(expected_constant) + 1e-12);
                        for(std::size_t i = 0; i < n; ++i)
                            EXPECT_NEAR(fixed[i], expected[i],
                                        1e-9 * std::abs(expected[i]) + 1e-12)
                                << path << ": vertex " << t << ", option "
                                << i;
                        for(const auto & y : {x_hat, other}) {
                            auto at = [&](const gecot::option_t i) {
                                return y[i];
                            };
                            double sum = constant;
                            for(const auto & [i, coefficient] : terms)
                                sum += coefficient * y[i];
                            for(std::size_t g = 0; g < block.groups.size();
                                ++g)
                                sum += block.group_part(g, cut).at(at);
                            double cut_at = cut.constant;
                            for(std::size_t i = 0; i < n; ++i)
                                cut_at += cut.coefficients[i] * y[i];
                            EXPECT_NEAR(sum, cut_at,
                                        1e-9 * std::max(1.0, cut_at))
                                << path << ": vertex " << t;
                        }
                    }
            }
        }
        EXPECT_GT(close_pairs[static_cast<std::size_t>(grouping::voronoi)],
                  2 * close_pairs[static_cast<std::size_t>(grouping::random)])
            << path;
    }
    EXPECT_GT(num_voronoi, 0u);
    EXPECT_GT(num_shared, 0u);
    EXPECT_GT(num_with_affine, 0u);
}
