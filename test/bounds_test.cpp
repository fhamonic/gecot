#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "gecot/bounds/all_options.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/bounds/lp_knapsack.hpp"
#include "gecot/bounds/qkp_planes.hpp"
#include "gecot/bounds/split_lagrange.hpp"
#include "gecot/helper.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

#include "conflict_instances.hpp"
#include "instance_builder.hpp"

namespace {

Instance reformulated(const char * path, double budget) {
    return trivial_reformulate_instance(parse_instance(path),
                                        gecot::budget_t{{budget}})
        .instance;
}

// cases of one vertex and no arc: only the criterion matters
Instance trivial_cases_instance(std::size_t num_cases,
                                criterion_formula criterion) {
    std::vector<case_spec> cases(num_cases,
                                 case_spec{1, {}, {}, {1.0}, {1.0}, {}, {}});
    return build_instance({}, cases, std::move(criterion));
}

struct generated_instance {
    Instance instance;
    double budget;
};

// cells v_in -> v -> v_out on a grid, v_out -> u_in between neighbours with
// probability 1; most cells have an option improving their two arcs and
// adding quality at v
generated_instance hexagon_like(std::mt19937 & rng, bool quality_gains = true) {
    auto pick = [&rng](std::initializer_list<double> values) {
        std::uniform_int_distribution<std::size_t> d(0, values.size() - 1);
        return values.begin()[d(rng)];
    };
    std::uniform_int_distribution<unsigned int> side(2, 3);
    const unsigned int width = side(rng), height = side(rng);
    const unsigned int num_cells = width * height;
    case_spec c{
        3 * num_cells,
        {},
        {},
        std::vector<double>(3 * num_cells, 0.0),
        std::vector<double>(3 * num_cells, 0.0),
        std::vector<std::vector<std::tuple<double, double, gecot::option_t>>>(
            3 * num_cells),
        {}};
    std::vector<double> costs;
    const double base = pick({0.5, 0.8, 0.9554});
    double total_cost = 0;
    for(unsigned int k = 0; k < num_cells; ++k) {
        const unsigned int in = 3 * k, v = in + 1, out = in + 2;
        const double q = pick({0, 0, 0, 1, 2.5});
        c.source_qualities[v] = c.target_qualities[v] = q;
        const double p_in = pick({base, base, 0.3}),
                     p_out = pick({base, base, 0.6});
        c.arcs.push_back({in, v});
        c.probabilities.push_back(p_in);
        c.arcs.push_back({v, out});
        c.probabilities.push_back(p_out);
        c.arc_options.resize(c.arcs.size());
        if(std::uniform_real_distribution<double>(0, 1)(rng) < 0.8) {
            const auto o = static_cast<gecot::option_t>(costs.size());
            costs.push_back(pick({0.5, 1, 1, 2, 2.6}));
            total_cost += costs.back();
            const double improved = pick({0.977, 0.99, 1.0});
            c.arc_options[c.arcs.size() - 2].emplace_back(
                std::max(p_in, improved), o);
            c.arc_options[c.arcs.size() - 1].emplace_back(
                std::max(p_out, improved), o);
            const double gain =
                quality_gains ? pick({costs.back(), 1.0, 0.0}) : 0.0;
            if(gain > 0) c.vertex_options[v].emplace_back(gain, gain, o);
        }
    }
    for(unsigned int k = 0; k < num_cells; ++k)
        for(unsigned int k2 = 0; k2 < num_cells; ++k2) {
            const unsigned int i = k / height, j = k % height, i2 = k2 / height,
                               j2 = k2 % height;
            if((i > i2 ? i - i2 : i2 - i) + (j > j2 ? j - j2 : j2 - j) != 1)
                continue;
            c.arcs.push_back({3 * k + 2, 3 * k2});
            c.probabilities.push_back(1.0);
        }
    c.arc_options.resize(c.arcs.size());
    const double budget =
        std::uniform_real_distribution<double>(0.3, 0.8)(rng) * total_cost;
    return {build_instance(costs, {c}, criterion_var{0}), budget};
}

// random digraphs; options with gains on several vertices, arcs improved by
// several options, zero qualities and probabilities; one or two cases under
// a random criterion
generated_instance general_random(std::mt19937 & rng) {
    std::uniform_real_distribution<double> unit(0, 1);
    std::uniform_int_distribution<int> num_options_d(2, 9);
    const int num_options = num_options_d(rng);
    std::vector<double> costs;
    double total_cost = 0;
    for(int o = 0; o < num_options; ++o) {
        costs.push_back(std::uniform_int_distribution<int>(1, 3)(rng));
        total_cost += costs.back();
    }
    std::uniform_int_distribution<gecot::option_t> option_d(
        0, static_cast<gecot::option_t>(num_options - 1));
    const std::size_t num_cases = unit(rng) < 0.3 ? 2 : 1;
    std::vector<case_spec> cases;
    for(std::size_t k = 0; k < num_cases; ++k) {
        const unsigned int n =
            std::uniform_int_distribution<unsigned int>(3, 7)(rng);
        case_spec c{
            n,
            {},
            {},
            std::vector<double>(n),
            std::vector<double>(n),
            std::vector<
                std::vector<std::tuple<double, double, gecot::option_t>>>(n),
            {}};
        for(unsigned int v = 0; v < n; ++v) {
            c.source_qualities[v] = unit(rng) < 0.4 ? 0.0 : 3 * unit(rng);
            c.target_qualities[v] = unit(rng) < 0.4 ? 0.0 : 3 * unit(rng);
            for(unsigned int w = 0; w < n; ++w) {
                if(v == w || unit(rng) > 0.35) continue;
                c.arcs.push_back({v, w});
                c.probabilities.push_back(unit(rng) < 0.1 ? 0.0 : unit(rng));
            }
        }
        c.arc_options.resize(c.arcs.size());
        for(int e = 0; e < 2 * num_options; ++e) {
            const gecot::option_t o = option_d(rng);
            if(unit(rng) < 0.5 && !c.arcs.empty()) {
                const std::size_t a =
                    std::uniform_int_distribution<std::size_t>(
                        0, c.arcs.size() - 1)(rng);
                c.arc_options[a].emplace_back(
                    c.probabilities[a] + (1 - c.probabilities[a]) * unit(rng),
                    o);
            } else {
                const unsigned int v =
                    std::uniform_int_distribution<unsigned int>(0, n - 1)(rng);
                c.vertex_options[v].emplace_back(
                    unit(rng) < 0.3 ? 0.0 : 2 * unit(rng),
                    unit(rng) < 0.3 ? 0.0 : 2 * unit(rng), o);
            }
        }
        cases.push_back(std::move(c));
    }
    criterion_formula criterion = criterion_var{0};
    if(num_cases == 2) {
        const double r = unit(rng);
        if(r < 0.4)
            criterion = criterion_sum{
                {criterion_product{{0.5 + unit(rng), criterion_var{0}}},
                 criterion_product{{unit(rng), criterion_var{1}}}, 1.0}};
        else if(r < 0.7)
            criterion = criterion_min{{criterion_var{0}, criterion_var{1}}};
        else
            criterion = criterion_product{{criterion_var{0}, criterion_var{1}}};
    }
    const double budget =
        std::uniform_real_distribution<double>(0.2, 0.9)(rng) * total_cost;
    return {build_instance(costs, cases, std::move(criterion)), budget};
}

// two cases on the same hexagon-like cells and options, with other
// qualities and probabilities
generated_instance hexagon_like_two_cases(std::mt19937 & rng) {
    auto [instance, budget] = hexagon_like(rng);
    std::uniform_real_distribution<double> unit(0, 1);
    const InstanceCase & first = instance.cases()[0];
    const auto & graph = first.graph();
    case_spec second{graph.num_vertices(), {}, {}, {}, {}, {}, {}};
    for(auto && v : melon::vertices(graph)) {
        const double q = first.source_quality_map()[v];
        second.source_qualities.push_back(q > 0 ? q * (0.5 + unit(rng)) : 0.0);
        second.target_qualities.push_back(q > 0 ? q * (0.5 + unit(rng)) : 0.0);
        second.vertex_options.push_back(first.vertex_options_map()[v]);
    }
    for(auto && a : melon::arcs(graph)) {
        second.arcs.push_back(
            {melon::arc_source(graph, a), melon::arc_target(graph, a)});
        const double p =
            first.arc_probability_map()[a] * (0.7 + 0.3 * unit(rng));
        second.probabilities.push_back(p);
        second.arc_options.push_back(first.arc_options_map()[a]);
    }
    case_spec copy_of_first{
        graph.num_vertices(), second.arcs, {}, {}, {}, {}, {}};
    for(auto && v : melon::vertices(graph)) {
        copy_of_first.source_qualities.push_back(first.source_quality_map()[v]);
        copy_of_first.target_qualities.push_back(first.target_quality_map()[v]);
        copy_of_first.vertex_options.push_back(first.vertex_options_map()[v]);
    }
    for(auto && a : melon::arcs(graph)) {
        copy_of_first.probabilities.push_back(first.arc_probability_map()[a]);
        copy_of_first.arc_options.push_back(first.arc_options_map()[a]);
    }
    std::vector<double> costs;
    for(auto && o : instance.options())
        costs.push_back(instance.option_cost(o));
    criterion_formula criterion =
        unit(rng) < 0.5
            ? criterion_formula{criterion_sum{
                  {criterion_product{{0.5 + unit(rng), criterion_var{0}}},
                   criterion_product{{0.5 + unit(rng), criterion_var{1}}}}}}
            : criterion_formula{
                  criterion_min{{criterion_var{0}, criterion_var{1}}}};
    return {
        build_instance(costs, {copy_of_first, second}, std::move(criterion)),
        budget};
}

// options centred on a vertex, each improving some of its incident arcs, so
// a vertex can be entered by several improved arcs of one option; parallel
// arcs, probabilities 0, 1 and tied at 0.5, arcs improved to 1, entries that
// do not improve and weaker duplicates. Keeps R1 and R3, so the case passes
// split_lagrange::check_structure.
case_spec star_options_case(std::mt19937 & rng, unsigned int n,
                            unsigned int num_options) {
    std::uniform_real_distribution<double> unit(0, 1);
    case_spec c{
        n,
        {},
        {},
        std::vector<double>(n),
        std::vector<double>(n),
        std::vector<std::vector<std::tuple<double, double, gecot::option_t>>>(
            n),
        {}};
    auto probability = [&] {
        const double r = unit(rng);
        return r < 0.08  ? 0.0
               : r < 0.2 ? 1.0
               : r < 0.4 ? 0.5
                         : 0.05 + 0.9 * unit(rng);
    };
    for(unsigned int v = 0; v < n; ++v) {
        c.source_qualities[v] = unit(rng) < 0.3 ? 0.0 : 0.2 + 2 * unit(rng);
        c.target_qualities[v] = unit(rng) < 0.3 ? 0.0 : 0.2 + 2 * unit(rng);
        for(unsigned int w = 0; w < n; ++w) {
            if(v == w || unit(rng) > 0.45) continue;
            for(int copy = 0; copy < (unit(rng) < 0.2 ? 2 : 1); ++copy) {
                c.arcs.push_back({v, w});
                c.probabilities.push_back(probability());
            }
        }
    }
    c.arc_options.resize(c.arcs.size());
    std::vector<bool> improved_arc(c.arcs.size(), false);
    std::vector<int> entering(n, -1);
    for(gecot::option_t o = 0; o < num_options; ++o) {
        const unsigned int centre =
            std::uniform_int_distribution<unsigned int>(0, n - 1)(rng);
        for(std::size_t a = 0; a < c.arcs.size(); ++a) {
            const auto [x, y] = c.arcs[a];
            if((x != centre && y != centre) || unit(rng) < 0.25 ||
               improved_arc[a] ||
               (entering[y] != -1 && entering[y] != static_cast<int>(o)))
                continue;
            const double p = c.probabilities[a];
            const double r = unit(rng);
            const double improved = r < 0.1   ? p
                                    : r < 0.4 ? 1.0
                                              : p + (1 - p) * unit(rng);
            c.arc_options[a].emplace_back(improved, o);
            if(improved > p) {
                improved_arc[a] = true;
                entering[y] = static_cast<int>(o);
            }
            if(unit(rng) < 0.1)
                c.arc_options[a].emplace_back(p + (improved - p) * unit(rng),
                                              o);
        }
        if(unit(rng) < 0.6)
            c.vertex_options[centre].emplace_back(unit(rng), unit(rng), o);
    }
    return c;
}

// max over the simple s-t paths π and the options S of π of
// w P_π(S) - Σ_{o∈S} ν_o, 0 without any path
double brute_force_pair_value(const InstanceCase & instance_case,
                              unsigned int s, unsigned int t, double w,
                              std::span<const double> nu) {
    const auto & graph = instance_case.graph();
    double best = 0.0;
    std::vector<melon::arc_t<melon::static_digraph>> path;
    std::vector<bool> visited(graph.num_vertices(), false);
    auto improved = [&](auto a, gecot::option_t & option) {
        double p = instance_case.arc_probability_map()[a];
        option = std::numeric_limits<gecot::option_t>::max();
        for(auto && [ip, o] : instance_case.arc_options_map()[a])
            if(ip > instance_case.arc_probability_map()[a]) {
                p = std::max(p, ip);
                option = o;
            }
        return p;
    };
    auto evaluate = [&] {
        std::vector<gecot::option_t> options;
        for(auto a : path) {
            gecot::option_t o;
            improved(a, o);
            if(o != std::numeric_limits<gecot::option_t>::max() &&
               std::ranges::find(options, o) == options.end())
                options.push_back(o);
        }
        for(std::uint32_t mask = 0; mask < (std::uint32_t{1} << options.size());
            ++mask) {
            double probability = 1.0, charge = 0.0;
            for(std::size_t i = 0; i < options.size(); ++i)
                if((mask >> i) & 1) charge += nu[options[i]];
            for(auto a : path) {
                gecot::option_t o;
                const double ip = improved(a, o);
                const auto it = std::ranges::find(options, o);
                const bool bought =
                    it != options.end() &&
                    ((mask >> static_cast<std::size_t>(it - options.begin())) &
                     1);
                probability *=
                    bought ? ip : instance_case.arc_probability_map()[a];
            }
            best = std::max(best, w * probability - charge);
        }
    };
    auto dfs = [&](auto && self, unsigned int v) -> void {
        if(v == t) {
            evaluate();
            return;
        }
        visited[v] = true;
        for(auto a : melon::out_arcs(graph, v)) {
            const unsigned int u = melon::arc_target(graph, a);
            if(visited[u]) continue;
            path.push_back(a);
            self(self, u);
            path.pop_back();
        }
        visited[v] = false;
    };
    dfs(dfs, s);
    return best;
}

}  // namespace

GTEST_TEST(criterion_analysis, linear_forms) {
    const auto f = gecot::linear_form(
        criterion_sum{{criterion_product{{2.0, criterion_var{0}}}, 3.0,
                       criterion_var{1}}},
        2);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->constant, 3.0);
    EXPECT_EQ(f->weights, (std::vector<double>{2.0, 1.0}));

    // trivial_reformulate_instance's weight comes first, any order works
    const auto g = gecot::linear_form(
        criterion_product{{criterion_var{0}, 0.25, criterion_min{{4.0}}}}, 1);
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->constant, 0.0);
    EXPECT_EQ(g->weights, (std::vector<double>{1.0}));

    EXPECT_FALSE(gecot::linear_form(
        criterion_product{{criterion_var{0}, criterion_var{1}}}, 2));
    EXPECT_FALSE(gecot::linear_form(
        criterion_min{{criterion_var{0}, criterion_var{1}}}, 2));
}

GTEST_TEST(criterion_analysis, monotony) {
    using gecot::is_non_decreasing;
    EXPECT_TRUE(is_non_decreasing(criterion_var{0}));
    EXPECT_TRUE(is_non_decreasing(criterion_sum{{criterion_var{0}, -5.0}}));
    EXPECT_TRUE(is_non_decreasing(
        criterion_product{{criterion_var{0}, criterion_var{1}}}));
    EXPECT_TRUE(
        is_non_decreasing(criterion_min{{criterion_var{0}, criterion_var{1}}}));
    EXPECT_TRUE(
        is_non_decreasing(criterion_product{{-1.0, criterion_var{0}, -2.0}}));
    EXPECT_FALSE(
        is_non_decreasing(criterion_product{{-1.0, criterion_var{0}}}));
    // a constant nested in a sum still counts as a constant factor
    EXPECT_FALSE(is_non_decreasing(
        criterion_product{{criterion_sum{{-1.0}}, criterion_var{0}}}));
    // a factor that can be negative
    EXPECT_FALSE(is_non_decreasing(criterion_product{
        {criterion_min{{criterion_var{0}, -1.0}}, criterion_var{1}}}));
    EXPECT_TRUE(is_non_decreasing(criterion_product{
        {criterion_min{{criterion_var{0}, 1.0}}, criterion_var{1}}}));

    // a decreasing term under a sum or a min
    EXPECT_FALSE(is_non_decreasing(criterion_sum{
        {criterion_product{{-1.0, criterion_var{0}}}, criterion_var{1}}}));
    EXPECT_FALSE(is_non_decreasing(criterion_min{
        {criterion_product{{-1.0, criterion_var{0}}}, criterion_var{1}}}));
    // a sum that can be negative, as one factor of two or alone
    EXPECT_FALSE(is_non_decreasing(criterion_product{
        {criterion_sum{{criterion_var{0}, -5.0}}, criterion_var{1}}}));
    EXPECT_TRUE(is_non_decreasing(
        criterion_product{{2.0, criterion_sum{{criterion_var{0}, -1.0}}}}));
    EXPECT_TRUE(is_non_decreasing(
        criterion_product{{0.0, criterion_product{{-1.0, criterion_var{0}}}}}));

    EXPECT_EQ(gecot::criterion_cases(criterion_sum{
                  {criterion_var{2}, criterion_min{{criterion_var{0}, 1.0}},
                   criterion_var{2}}}),
              (std::vector<gecot::case_id_t>{0, 2}));
}

GTEST_TEST(criterion_analysis, non_decreasing_criterion_of_an_instance) {
    using gecot::has_non_decreasing_criterion;
    EXPECT_TRUE(has_non_decreasing_criterion(trivial_cases_instance(
        2, criterion_min{{criterion_var{0}, criterion_var{1}}})));
    // 2 z - z: its second term alone decreases
    const criterion_sum net_positive{
        {criterion_product{{2.0, criterion_var{0}}},
         criterion_product{{-1.0, criterion_var{0}}}}};
    EXPECT_FALSE(gecot::is_non_decreasing(net_positive));
    EXPECT_TRUE(
        has_non_decreasing_criterion(trivial_cases_instance(2, net_positive)));
    EXPECT_FALSE(has_non_decreasing_criterion(trivial_cases_instance(
        2, criterion_sum{{criterion_var{1},
                          criterion_product{{-1.0, criterion_var{0}}}}})));
    // a minimum is not linear: each of its terms must be non-decreasing
    EXPECT_FALSE(has_non_decreasing_criterion(trivial_cases_instance(
        2, criterion_min{{net_positive, criterion_var{1}}})));
}

GTEST_TEST(criterion_analysis, bound_criterion_calls) {
    std::vector<std::vector<double>> calls;
    auto weighted_bound = [&calls](std::span<const double> w) {
        calls.emplace_back(w.begin(), w.end());
        return 10.0 * w[0] + 20.0 * w[1];
    };

    const Instance linear = trivial_cases_instance(
        2, criterion_sum{{criterion_product{{2.0, criterion_var{0}}},
                          criterion_var{1}, 1.0}});
    EXPECT_EQ(gecot::bound_criterion(linear, "test", weighted_bound), 41.0);
    EXPECT_EQ(calls, (std::vector<std::vector<double>>{{2.0, 1.0}}));

    calls.clear();
    const Instance min = trivial_cases_instance(
        2, criterion_min{{criterion_var{0}, criterion_var{1}}});
    EXPECT_EQ(gecot::bound_criterion(min, "test", weighted_bound), 10.0);
    EXPECT_EQ(calls,
              (std::vector<std::vector<double>>{{1.0, 0.0}, {0.0, 1.0}}));

    const Instance decreasing =
        trivial_cases_instance(2, criterion_product{{-1.0, criterion_var{0}}});
    EXPECT_THROW(gecot::bound_criterion(decreasing, "test", weighted_bound),
                 std::invalid_argument);
}

GTEST_TEST(bounds, all_options_bounds_the_optimum) {
    struct test_case {
        const char * path;
        double budget;
    };
    for(auto && [path, budget] :
        {test_case{PROJECT_SOURCE_DIR "/test/instances/test/instance.json", 2},
         test_case{PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json", 3},
         test_case{PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json", 8},
         test_case{PROJECT_SOURCE_DIR "/test/instances/Aude/biobj_aude.json",
                   5}}) {
        const Instance instance = reformulated(path, budget);
        const double optimum = brute_force_optimum(instance, budget);
        EXPECT_GE(gecot::all_options_bound(instance).bound,
                  optimum * (1 - 1e-12))
            << path << " at " << budget;
    }
}

GTEST_TEST(bounds, all_options_is_exact_within_a_budget_for_all) {
    const Instance instance =
        reformulated(PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json", 15);
    EXPECT_NEAR(gecot::all_options_bound(instance).bound,
                brute_force_optimum(instance, 15),
                1e-9 * gecot::all_options_bound(instance).bound);
}

GTEST_TEST(bounds, all_options_refuses_a_decreasing_criterion) {
    Instance instance =
        reformulated(PROJECT_SOURCE_DIR "/test/instances/Aude/aude.json", 5);
    instance.set_criterion(criterion_product{{-1.0, criterion_var{0}}});
    EXPECT_THROW(gecot::all_options_bound(instance), std::invalid_argument);
}

GTEST_TEST(bounds, lp_knapsack) {
    std::vector<double> x(3);
    EXPECT_DOUBLE_EQ(gecot::lp_knapsack(std::vector{6.0, 10.0, 12.0},
                                        std::vector{1.0, 2.0, 3.0}, 5.0, x),
                     24.0);
    EXPECT_EQ(x, (std::vector<double>{1.0, 1.0, 2.0 / 3.0}));
    EXPECT_EQ(gecot::lp_knapsack(std::vector{-1.0, 0.0, 3.0},
                                 std::vector{1.0, 1.0, 1.0}, 2.0),
              3.0);
    EXPECT_EQ(gecot::lp_knapsack(std::vector{1.0}, std::vector{1.0}, 0.0), 0.0);
    // items of cost 0 fit even at capacity 0
    EXPECT_EQ(gecot::lp_knapsack(std::vector{0.5, 1.0}, std::vector{0.0, 1.0},
                                 0.0, std::span<double>(x).subspan(0, 2)),
              0.5);
    EXPECT_EQ(x[0], 1.0);
    EXPECT_EQ(x[1], 0.0);
}

GTEST_TEST(bounds, qkp_planes_bounds_the_optimum) {
    using enum gecot::qkp_planes::theta_start;
    std::mt19937 rng(20261105);
    int checked = 0;
    for(int round = 0; round < 300; ++round) {
        auto [instance, budget] =
            round % 2 ? general_random(rng) : hexagon_like(rng);
        const double optimum = brute_force_optimum(instance, budget);
        const double all_options =
            gecot::is_non_decreasing(instance.criterion())
                ? gecot::all_options_bound(instance).bound
                : std::numeric_limits<double>::infinity();
        const double tolerance = 1e-9 * std::max(1.0, std::abs(optimum));
        for(auto start : {best, half, target, source}) {
            for(std::size_t iterations : {std::size_t{0}, std::size_t{20}}) {
                const gecot::qkp_planes method{start, iterations, {}};
                const double bound = method.bound(instance, {{budget}}).bound;
                EXPECT_GE(bound, optimum - tolerance)
                    << "round " << round << " start " << static_cast<int>(start)
                    << " iterations " << iterations;
                EXPECT_LE(bound, all_options * (1 + 1e-12) + tolerance)
                    << "round " << round;
            }
        }
        ++checked;
    }
    EXPECT_EQ(checked, 300);
}

// on a reformulated instance, as gecot bound runs: all_options would also
// buy the options over the budget
GTEST_TEST(bounds, qkp_planes_without_quality_gains_is_all_options) {
    std::mt19937 rng(7);
    for(int round = 0; round < 20; ++round) {
        auto [raw_instance, budget] = hexagon_like(rng, false);
        const Instance instance = trivial_reformulate_instance(
                                      raw_instance, gecot::budget_t{{budget}})
                                      .instance;
        const double all_options = gecot::all_options_bound(instance).bound;
        EXPECT_NEAR(gecot::qkp_planes{}.bound(instance, {{budget}}).bound,
                    all_options, 1e-12 * all_options);
    }
}

GTEST_TEST(bounds, qkp_planes_joint_bound_is_at_most_the_case_bounds) {
    std::mt19937 rng(11);
    int multi_case = 0;
    while(multi_case < 30) {
        auto [instance, budget] = general_random(rng);
        if(instance.cases().size() != 2) continue;
        ++multi_case;
        // for a given split: the best of the three is not subadditive
        const gecot::qkp_planes method{
            gecot::qkp_planes::theta_start::half, 0, {}};
        const std::vector<double> w{0.7, 1.3};
        const double joint = method.weighted(instance, {{budget}}, w).bound;
        const double separate =
            0.7 * method.weighted(instance, {{budget}}, std::vector{1.0, 0.0})
                      .bound +
            1.3 * method.weighted(instance, {{budget}}, std::vector{0.0, 1.0})
                      .bound;
        EXPECT_LE(joint, separate * (1 + 1e-12));
    }
}

GTEST_TEST(bounds, qkp_planes_iterations_never_raise_the_bound) {
    std::mt19937 rng(13);
    for(int round = 0; round < 30; ++round) {
        auto [instance, budget] = hexagon_like(rng);
        using enum gecot::qkp_planes::theta_start;
        const gecot::qkp_planes iterated{best, 30, {}}, started{best, 0, {}};
        EXPECT_LE(iterated.bound(instance, {{budget}}).bound,
                  started.bound(instance, {{budget}}).bound);
    }
}

GTEST_TEST(bounds, split_lagrange_bounds_the_optimum) {
    std::mt19937 rng(20261106);
    int stepped = 0, below_qkp = 0, runs = 0;
    for(int round = 0; round < 120; ++round) {
        auto [instance, budget] = round % 3 == 0 ? hexagon_like(rng)
                                  : round % 3 == 1
                                      ? hexagon_like(rng, false)
                                      : hexagon_like_two_cases(rng);
        const double optimum = brute_force_optimum(instance, budget);
        const double tolerance = 1e-9 * std::max(1.0, std::abs(optimum));
        for(std::size_t iterations : {std::size_t{1}, std::size_t{40}}) {
            for(bool aim_at_optimum : {false, true}) {
                gecot::split_lagrange method;
                method.iterations = iterations;
                if(aim_at_optimum) method.target = optimum;
                const auto report = method.bound(instance, {{budget}});
                EXPECT_GE(report.bound, optimum - tolerance)
                    << "round " << round << " iterations " << iterations
                    << (aim_at_optimum ? " aiming at the optimum" : "");
                ++runs;
                stepped += report.stats.front().second > 1;
                below_qkp +=
                    report.bound <
                    gecot::qkp_planes{}.bound(instance, {{budget}}).bound *
                        (1 - 1e-9);
            }
        }
    }
    // the steps run and lower the bound below qkp_planes' on part of them
    EXPECT_GT(stepped, runs / 4);
    EXPECT_GT(below_qkp, runs / 10);
}

GTEST_TEST(bounds, split_lagrange_starts_from_qkp_planes) {
    using enum gecot::qkp_planes::theta_start;
    std::mt19937 rng(17);
    for(int round = 0; round < 20; ++round) {
        auto [instance, budget] = hexagon_like(rng);
        const gecot::qkp_planes vertex_side{half, 0, {}};
        gecot::split_lagrange method;
        method.vertex_side = vertex_side;
        method.iterations = 1;
        const std::vector<double> w{1.0};
        const double qkp = vertex_side.weighted(instance, {{budget}}, w).bound;
        // the first step's bound itself: the result is at most qkp_planes'
        const auto result = method.weighted(instance, {{budget}}, w);
        ASSERT_EQ(result.sweep_bounds.size(), 1u);
        EXPECT_NEAR(result.sweep_bounds[0], qkp, 1e-9 * qkp);
    }
}

GTEST_TEST(bounds, split_lagrange_prices_pairs_exactly) {
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> unit(0, 1);
    int pairs = 0;
    while(pairs < 300000) {
        const unsigned int n =
            std::uniform_int_distribution<unsigned int>(3, 7)(rng);
        const unsigned int num_options =
            std::uniform_int_distribution<unsigned int>(1, 6)(rng);
        const Instance instance = build_instance(
            std::vector<double>(num_options, 1.0),
            {star_options_case(rng, n, num_options)}, criterion_var{0});
        gecot::split_lagrange::check_structure(
            instance, {{static_cast<double>(num_options)}});
        const InstanceCase & c = instance.cases()[0];
        for(unsigned int s = 0; s < n; ++s)
            for(unsigned int t = 0; t < n; ++t) {
                if(s == t || c.source_quality_map()[s] == 0 ||
                   c.target_quality_map()[t] == 0)
                    continue;
                const double w =
                    c.source_quality_map()[s] * c.target_quality_map()[t];
                // prices of the pair's scale, where paying matters
                std::vector<double> nu(num_options);
                for(auto & v : nu) {
                    const double r = unit(rng);
                    v = r < 0.3 ? 0.0 : r < 0.5 ? 10 * w : 0.3 * w * unit(rng);
                }
                const double expected = brute_force_pair_value(c, s, t, w, nu);
                EXPECT_NEAR(gecot::split_lagrange::pair_value(
                                instance, c,
                                {{static_cast<double>(num_options)}}, s, t, nu),
                            expected, 1e-12 * std::max(1.0, expected))
                    << "pair " << s << "-" << t << " after " << pairs;
                ++pairs;
            }
    }
}

GTEST_TEST(bounds, split_lagrange_checks_the_priced_cases_only) {
    const case_spec fine{2, {{0, 1}}, {0.5}, {1, 1}, {1, 1}, {}, {{{0.9, 0}}}};
    const case_spec two_into_one{
        3,  {{0, 2}, {1, 2}},        {0.5, 0.5}, {1, 1, 1}, {1, 1, 1},
        {}, {{{0.9, 0}}, {{0.9, 1}}}};
    const Instance instance =
        build_instance({1, 1}, {fine, two_into_one}, criterion_var{0});
    EXPECT_THROW(gecot::split_lagrange::check_structure(instance, {{2}}),
                 std::invalid_argument);
    EXPECT_NO_THROW(gecot::split_lagrange{}.bound(instance, {{2}}));
}

GTEST_TEST(bounds, split_lagrange_refuses_unsupported_structures) {
    // 0 -> 2 by option 0 and 1 -> 2 by option 1: R3
    const case_spec two_into_one{
        3,  {{0, 2}, {1, 2}},        {0.5, 0.5}, {1, 1, 1}, {1, 1, 1},
        {}, {{{0.9, 0}}, {{0.9, 1}}}};
    // 0 -> 1 and 2 -> 3 by option 0: R2
    const case_spec disjoint{4,
                             {{0, 1}, {2, 3}, {1, 2}},
                             {0.5, 0.5, 0.5},
                             {1, 1, 1, 1},
                             {1, 1, 1, 1},
                             {},
                             {{{0.9, 0}}, {{0.9, 0}}, {}}};
    // 0 -> 1 by options 0 and 1: R1
    const case_spec shared{
        2, {{0, 1}}, {0.5}, {1, 1}, {1, 1}, {}, {{{0.9, 0}, {0.8, 1}}}};
    for(const case_spec & c : {two_into_one, disjoint, shared}) {
        const Instance instance = build_instance({1, 1}, {c}, criterion_var{0});
        EXPECT_THROW(gecot::split_lagrange::check_structure(instance, {{2}}),
                     std::invalid_argument);
        EXPECT_THROW(gecot::split_lagrange{}.bound(instance, {{2}}),
                     std::invalid_argument);
    }
    std::mt19937 rng(19);
    for(int round = 0; round < 20; ++round) {
        auto [instance, budget] = hexagon_like(rng);
        EXPECT_NO_THROW(
            gecot::split_lagrange::check_structure(instance, {{budget}}));
    }
}

GTEST_TEST(bounds, qkp_planes_and_split_lagrange_with_conflicts_and_costs) {
    std::mt19937 rng(29);
    int tightened = 0, priced = 0;
    for(int round = 0; round < 300; ++round) {
        auto [instance, budget] = random_conflicts_instance(rng);
        const double optimum = brute_force_optimum(instance, budget);
        const double tolerance = 1e-9 * std::max(1.0, std::abs(optimum));
        for(std::size_t iterations : {std::size_t{0}, std::size_t{20}}) {
            const gecot::qkp_planes method{
                gecot::qkp_planes::theta_start::best, iterations, {}};
            const double bound = method.bound(instance, budget).bound;
            EXPECT_GE(bound, optimum - tolerance) << "round " << round;
            // the second dimension is read
            tightened +=
                bound <
                method.bound(instance, {{budget[0], 1e9}}).bound * (1 - 1e-12);
        }
        try {
            gecot::split_lagrange::check_structure(instance, budget);
        } catch(const std::invalid_argument &) {
            continue;
        }
        EXPECT_GE(gecot::split_lagrange{}.bound(instance, budget).bound,
                  optimum - tolerance)
            << "round " << round;
        ++priced;
    }
    EXPECT_GT(tightened, 0);
    EXPECT_GT(priced, 0);
}

// two options of one vertex, each of a source and a target gain: a set
// forbids buying both, and so their product
GTEST_TEST(bounds, qkp_planes_drops_the_products_of_a_conflict_set) {
    const case_spec site{
        2, {{0, 1}}, {0.5}, {1, 1}, {1, 1}, {{{1, 1, 0}, {1, 1, 1}}, {}}, {{}}};
    const Instance both =
        build_instance({"cost"}, {{1}, {1}}, {}, {site}, criterion_var{0});
    const Instance one = build_instance({"cost"}, {{1}, {1}}, {{0, 1}}, {site},
                                        criterion_var{0});
    auto qkp = [](const Instance & instance) {
        return gecot::qkp_planes{}.bound(instance, {{2}}).bound;
    };
    EXPECT_LT(qkp(one), qkp(both));
    EXPECT_GE(qkp(one), brute_force_optimum(one, 2) - 1e-9);
}

// two options that fit alone in both budgets, together in the area only
GTEST_TEST(bounds, qkp_planes_takes_the_least_knapsack_of_the_dimensions) {
    const case_spec site{
        2, {{0, 1}}, {0.5}, {1, 1}, {1, 1}, {{{1, 1, 0}}, {{1, 1, 1}}}, {{}}};
    const Instance instance = build_instance(
        {"area", "money"}, {{1, 2}, {1, 2}}, {}, {site}, criterion_var{0});
    auto qkp = [&](double money) {
        return gecot::qkp_planes{}.bound(instance, {{2, money}}).bound;
    };
    EXPECT_LT(qkp(2), qkp(4));
    EXPECT_GE(qkp(2), brute_force_optimum(instance, {{2, 2}}) - 1e-9);
}

GTEST_TEST(trivial_reformulation, keeps_free_options_that_exclude_others) {
    const Instance raw = three_woods_conflict();
    // buying the free plantation statically would give the infeasible
    // {plantation, underpass} 35.40, deleting the underpass 28.76 at B=5
    EXPECT_NEAR(brute_force_optimum(raw, 3), 28.76, 1e-9);
    EXPECT_NEAR(brute_force_optimum(raw, 5), 31.04, 1e-9);
    for(const double budget : {3.0, 5.0}) {
        const auto reformulated =
            trivial_reformulate_instance(raw, gecot::budget_t{{budget}});
        EXPECT_TRUE(reformulated.instance.contains_option("o2"));
        EXPECT_FALSE(reformulated.static_options[2]);
        EXPECT_EQ(reformulated.instance.conflict_sets().size(), 1u);
        EXPECT_NEAR(brute_force_optimum(reformulated.instance, budget),
                    brute_force_optimum(raw, budget), 1e-9);
    }
    // the underpass over the budget, the set constrains nothing
    const auto reformulated =
        trivial_reformulate_instance(raw, gecot::budget_t{{2}});
    EXPECT_FALSE(reformulated.instance.contains_option("o2"));
    EXPECT_TRUE(reformulated.static_options[2]);
    EXPECT_TRUE(reformulated.instance.conflict_sets().empty());
    EXPECT_NEAR(brute_force_optimum(reformulated.instance, 2), 28.76, 1e-9);
}

GTEST_TEST(trivial_reformulation, keeps_the_optimum_with_conflicts_and_costs) {
    std::mt19937 rng(23);
    int static_free = 0, kept_free = 0, dropped = 0;
    for(int round = 0; round < 300; ++round) {
        auto [raw, budget] = random_conflicts_instance(rng);
        const auto reformulated = trivial_reformulate_instance(raw, budget);
        const double scale2 =
            reformulated.quality_scale * reformulated.quality_scale;
        const double optimum = brute_force_optimum(raw, budget);
        EXPECT_NEAR(brute_force_optimum(reformulated.instance, budget) * scale2,
                    optimum, 1e-9 * std::max(1.0, optimum))
            << "round " << round;
        for(auto && o : raw.options()) {
            const bool free =
                raw.option_cost(o, 0) == 0 && raw.option_cost(o, 1) == 0;
            static_free += free && reformulated.static_options[o];
            kept_free += free && !reformulated.static_options[o];
            dropped +=
                !reformulated.static_options[o] &&
                !reformulated.instance.contains_option(raw.option_name(o));
        }
    }
    // every rule was exercised
    EXPECT_GT(static_free, 0);
    EXPECT_GT(kept_free, 0);
    EXPECT_GT(dropped, 0);
}
