#include <gtest/gtest.h>
#include <set>
#include <string>

#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/solvers/greedy.hpp"
#include "gecot/solvers/static.hpp"

#include "conflict_instances.hpp"
#include "instance.hpp"
#include "instance_builder.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

using option_names = std::set<std::string>;

// Mirrors gecot optimize, whose feasibility tolerance defaults to 1e-7: the
// options bought statically are listed too.
option_names solve_file(auto solver, const std::string & path,
                        const std::string & budget_text) {
    solver.feasibility_tol = 1e-7;
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
