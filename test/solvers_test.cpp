#include <gtest/gtest.h>
#include <set>
#include <string>

#include <tbb/global_control.h>

#include "gecot/solvers/greedy_decremental.hpp"
#include "gecot/solvers/greedy_incremental.hpp"
#include "gecot/solvers/static_decremental.hpp"
#include "gecot/solvers/static_incremental.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

using option_names = std::set<std::string>;

// Mirrors gecot_optimize, whose feasibility tolerance defaults to 1e-7.
// One thread: parallel reductions vary in the last ulp and can flip near-ties.
option_names solve_aude(auto solver, double budget) {
    const tbb::global_control single_thread(
        tbb::global_control::max_allowed_parallelism, 1);
    solver.feasibility_tol = 1e-7;
    const Instance raw_instance =
        parse_instance(PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json");
    const Instance instance =
        trivial_reformulate_instance(raw_instance, budget);
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

GTEST_TEST(solvers, static_incr) {
    const gecot::solvers::StaticIncremental s;
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

GTEST_TEST(solvers, static_decr) {
    const gecot::solvers::StaticDecremental s;
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

GTEST_TEST(solvers, greedy_incr) {
    const gecot::solvers::GreedyIncremental s;
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

GTEST_TEST(solvers, greedy_decr) {
    const gecot::solvers::GreedyDecremental s;
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

// worst_greedy_incr and worst_greedy_decr first differ at budget 11
GTEST_TEST(solvers, worst_greedy_incr) {
    const gecot::solvers::GreedyIncremental s{.worst = true};
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

GTEST_TEST(solvers, worst_greedy_decr) {
    const gecot::solvers::GreedyDecremental s{.worst = true};
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
