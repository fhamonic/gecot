#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gecot/concepts/instance.hpp"

namespace gecot {

struct solve_report {
    std::string status = "completed";
    // best_bound and score_stats are in the solved instance's score unit,
    // which gecot_optimize scales back
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
