#ifndef GECOT_BOUNDS_BOUND_REPORT_HPP
#define GECOT_BOUNDS_BOUND_REPORT_HPP

#include <string>
#include <utility>
#include <vector>

namespace gecot {

// bound and score_stats are in the score unit of the bounded instance
struct bound_report {
    double bound;  // no solution within the budget scores more
    std::string status = "completed";
    std::vector<std::pair<std::string, double>> stats = {};
    std::vector<std::pair<std::string, double>> score_stats = {};
};

}  // namespace gecot

#endif  // GECOT_BOUNDS_BOUND_REPORT_HPP
