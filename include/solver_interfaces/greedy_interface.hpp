#ifndef GECOT_GREEDY_INTERFACE_HPP
#define GECOT_GREEDY_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/solvers/greedy.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class GreedyInterface : public AbstractSolverInterface {
private:
    gecot::solvers::Greedy solver;

public:
    GreedyInterface() : AbstractSolverInterface("greedy") {
        add_feasibility_tol_option(solver.feasibility_tol);
        auto * decremental = app.add_flag(
            "--decremental", solver.decremental,
            "From the improved landscape, iteratively remove the option with "
            "the worst gain/cost ratio until the budget is met (Zonation "
            "Algorithm), then add back the options that fit");
        // --only-dec: its name in GECOT 1.0
        app.add_flag("--no-add-back,--only-dec", solver.only_dec,
                     "Do not add back, after the removals, the options that "
                     "still fit: the final purchases that spend the rest of "
                     "the budget")
            ->needs(decremental);
        app.add_flag("--worst", solver.worst,
                     "For comparison: the opposite choice at each step, to "
                     "minimize connectivity while spending the budget");
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override {
        // the flag read after the solve: a braced list runs in order
        return {solver.solve(instance, B),
                {gecot::stop_requested ? "interrupted" : "completed"}};
    };

    std::string description() const override {
        return "From the base landscape, iteratively add the option with the "
               "best gain/cost ratio; with --decremental, iteratively remove "
               "from the improved landscape the option with the worst one "
               "(Zonation Algorithm).";
    }
};

#endif  // GECOT_GREEDY_INTERFACE_HPP