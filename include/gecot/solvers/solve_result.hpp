#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gecot/concepts/instance.hpp"

namespace gecot {

struct solve_report {
    std::string status = "completed";
    std::optional<double> best_bound = {};  // proven upper bound on the score
    std::vector<std::pair<std::string, double>> stats = {};
};

template <instance_c I>
struct solve_result {
    instance_solution_t<I> solution;
    solve_report report = {};
};

}  // namespace gecot
