#pragma once

#include <CLI/CLI.hpp>

#include "gecot/solvers/greedy_decremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class WorstGreedyDecrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::GreedyDecremental solver{.worst = true};

public:
    WorstGreedyDecrementalInterface()
        : AbstractSolverInterface("worst_greedy_decr") {
        add_feasibility_tol_option(solver.feasibility_tol);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "From the improved landscape, iteratively remove the option "
               "with the best gain/cost ratio (to minimize connectivity while "
               "spending the budget).";
    }
};
