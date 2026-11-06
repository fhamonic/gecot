#ifndef GECOT_BOUNDS_LP_KNAPSACK_HPP
#define GECOT_BOUNDS_LP_KNAPSACK_HPP

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <span>
#include <vector>

namespace gecot {

// max{v·x : c·x <= capacity, 0 <= x <= 1} for costs c >= 0, by Dantzig's
// ratio order: items with v <= 0 stay out. x, if given, receives the
// solution.
inline double lp_knapsack(std::span<const double> values,
                          std::span<const double> costs, double capacity,
                          std::span<double> x = {}) {
    assert(values.size() == costs.size());
    assert(x.empty() || x.size() == values.size());
    std::ranges::fill(x, 0.0);
    std::vector<std::size_t> order;
    for(std::size_t i = 0; i < values.size(); ++i)
        if(values[i] > 0) order.push_back(i);
    // ties by index: the same solution on every run
    std::ranges::sort(order, [&](std::size_t i, std::size_t j) {
        const double ri = values[i] / costs[i], rj = values[j] / costs[j];
        return ri > rj || (ri == rj && i < j);
    });
    double value = 0.0;
    // the items of cost 0 come first, even at capacity 0
    for(const std::size_t i : order) {
        if(costs[i] <= capacity) {
            value += values[i];
            capacity -= costs[i];
            if(!x.empty()) x[i] = 1.0;
            continue;
        }
        if(capacity > 0) {
            const double fraction = capacity / costs[i];
            value += fraction * values[i];
            if(!x.empty()) x[i] = fraction;
        }
        break;
    }
    return value;
}

}  // namespace gecot

#endif  // GECOT_BOUNDS_LP_KNAPSACK_HPP
