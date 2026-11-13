#include <gtest/gtest.h>
#include <set>
#include <string>

#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/solvers/greedy.hpp"
#include "gecot/solvers/static.hpp"

#include "instance.hpp"
#include "instance_builder.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

using option_names = std::set<std::string>;

// Mirrors gecot optimize, whose feasibility tolerance defaults to 1e-7.
option_names solve_aude(auto solver, double budget) {
    solver.feasibility_tol = 1e-7;
    const Instance raw_instance =
        parse_instance(PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json");
    const Instance instance =
        trivial_reformulate_instance(raw_instance, budget).first;
    const auto solution = solver.solve(instance, budget);
    option_names taken;
    for(auto && option : raw_instance.options()) {
        const auto & name = raw_instance.option_name(option);
        if(instance.contains_option(name) &&
           solution[instance.option_from_name(name)])
            taken.insert(name);
    }
    return taken;
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
    const auto solution = solver.solve(instance, 1);
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
