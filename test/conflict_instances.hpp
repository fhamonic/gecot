#ifndef GECOT_TEST_CONFLICT_INSTANCES_HPP
#define GECOT_TEST_CONFLICT_INSTANCES_HPP

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "gecot/helper.hpp"

#include "instance.hpp"
#include "instance_builder.hpp"

// the best score over every feasible subset of the options
inline double brute_force_optimum(const Instance & instance,
                                  const gecot::budget_t & budget) {
    const std::size_t num_options = instance.num_options();
    EXPECT_LE(num_options, 20u);
    const auto cases_vertex_options =
        gecot::compute_cases_vertex_options(instance);
    const auto cases_arc_options = gecot::compute_cases_arc_options(instance);
    auto solution = instance.create_option_map<bool>();
    double best = -std::numeric_limits<double>::infinity();
    for(std::uint32_t mask = 0; mask < (std::uint32_t{1} << num_options);
        ++mask) {
        for(auto && o : instance.options()) solution[o] = (mask >> o) & 1;
        if(!gecot::is_feasible(instance, solution, budget, 0.0)) continue;
        best = std::max(
            best,
            instance.eval_criterion(gecot::compute_solution_cases_pc_num(
                instance, solution, cases_vertex_options, cases_arc_options)));
    }
    return best;
}
inline double brute_force_optimum(const Instance & instance,
                                  const double budget) {
    return brute_force_optimum(instance, gecot::budget_t{{budget}});
}

// docs/examples/three-woods.json with a free plantation in conflict with the
// underpass: options hedgerow (o0, cost 2), underpass (o1, cost 3) and
// plantation (o2, cost 0)
inline Instance three_woods_conflict() {
    case_spec woods{3,
                    {{0, 1}, {1, 0}, {1, 2}, {2, 1}},
                    {0.5, 0.5, 0.2, 0.2},
                    {2, 1, 3},
                    {2, 1, 3},
                    {{}, {{1.0, 1.0, 2}}, {}},
                    {{{0.9, 0}}, {{0.9, 0}}, {{0.8, 1}}, {{0.8, 1}}}};
    return build_instance({"cost"}, {{2}, {3}, {0}}, {{1, 2}}, {woods},
                          criterion_var{0});
}

// options of two costs, some of them 0, overlapping conflict sets and
// probability 1 arcs for the merging
inline std::pair<Instance, gecot::budget_t> random_conflicts_instance(
    std::mt19937 & rng) {
    std::uniform_int_distribution<int> cost(0, 3), coin(0, 1), quality(0, 3);
    std::uniform_real_distribution<double> probability(0.1, 0.9);
    const unsigned num_options = 7, num_vertices = 5;
    std::uniform_int_distribution<unsigned> option(0, num_options - 1),
        vertex(0, num_vertices - 1);
    std::vector<std::vector<double>> costs;
    for(unsigned o = 0; o < num_options; ++o)
        costs.push_back({static_cast<double>(coin(rng) * cost(rng)),
                         static_cast<double>(coin(rng) * cost(rng))});
    std::vector<std::vector<gecot::option_t>> sets(3);
    for(auto & set : sets)
        while(set.size() < 2 + static_cast<std::size_t>(coin(rng))) {
            const auto o = option(rng);
            if(!std::ranges::contains(set, o)) set.push_back(o);
        }
    case_spec spec{num_vertices, {}, {}, {}, {}, {}, {}};
    spec.vertex_options.resize(num_vertices);
    for(unsigned v = 0; v < num_vertices; ++v) {
        spec.source_qualities.push_back(quality(rng));
        spec.target_qualities.push_back(quality(rng));
        if(coin(rng))
            spec.vertex_options[v].emplace_back(quality(rng), quality(rng),
                                                option(rng));
    }
    for(int i = 0; i < 9; ++i) {
        const auto u = vertex(rng), v = vertex(rng);
        if(u == v) continue;
        spec.arcs.emplace_back(u, v);
        spec.probabilities.push_back(i < 2 ? 1.0 : probability(rng));
        spec.arc_options.push_back({});
        if(i >= 2 && coin(rng))
            spec.arc_options.back().emplace_back(
                std::min(1.0, spec.probabilities.back() + 0.4), option(rng));
    }
    gecot::budget_t budget{{static_cast<double>(cost(rng) + cost(rng)),
                            static_cast<double>(cost(rng) + cost(rng))}};
    return {build_instance({"money", "area"}, costs, sets, {spec},
                           criterion_var{0}),
            budget};
}

#endif  // GECOT_TEST_CONFLICT_INSTANCES_HPP
