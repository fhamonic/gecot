#pragma once

#include <CLI/CLI.hpp>

#include "gecot/solvers/greedy_incremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class WorstGreedyIncrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::GreedyIncremental solver{.worst = true};

public:
    WorstGreedyIncrementalInterface()
        : AbstractSolverInterface("worst_greedy_incr") {
        add_feasibility_tol_option(solver.feasibility_tol);
    }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override {
        return {solver.solve(instance, B)};
    };

    std::string description() const override {
        return "From the base landscape, iteratively add the option with the "
               "worst gain/cost ratio (to minimize connectivity while spending "
               "the budget).";
    }
};
