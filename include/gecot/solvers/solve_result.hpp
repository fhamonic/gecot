#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gecot/concepts/instance.hpp"

namespace gecot {

// the relative gap between a solution and the bound at which the MIP solvers
// stop: a bound within it of a score proves that score optimal
inline constexpr double mip_gap = 1e-10;

struct solve_report {
    std::string status = "completed";
    // best_bound and score_stats are in the solved instance's score unit,
    // which gecot optimize scales back
    std::optional<double> best_bound = {};  // proven upper bound on the score
    std::vector<std::pair<std::string, double>> stats = {};
    std::vector<std::pair<std::string, double>> score_stats = {};
};

template <instance_c I>
struct solve_result {
    instance_solution_t<I> solution;
    solve_report report = {};
};

}  // namespace gecot
