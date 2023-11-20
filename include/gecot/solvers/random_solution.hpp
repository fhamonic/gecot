#ifndef GECOT_SOLVERS_RANDOM_SOLUTION_HPP
#define GECOT_SOLVERS_RANDOM_SOLUTION_HPP

#include "gecot/concepts/instance.hpp"
#include "gecot/utils/chronometer.hpp"
#include "gecot/utils/random_chooser.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct RandomSolution {
    int seed = 314159265;
    int time_ms = 0;

    template <instance_c I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        chronometer chrono;
        Solution solution = instance.create_solution();

        RandomChooser<Option> option_chooser(seed);
        for(Option i : instance.options()) option_chooser.add(i, 1);

        double purschaised = 0.0;
        while(option_chooser.canPick()) {
            Option i = option_chooser.pick();
            const double cost = instance.option_cost(i);
            if(purschaised + cost > budget) continue;
            purschaised += cost;
            solution[i] = 1;
        }

        time_ms = chrono.time_ms();

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_RANDOM_SOLUTION_HPP