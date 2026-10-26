#ifndef GECOT_GREEDY_INCREMENTAL_INTERFACE_HPP
#define GECOT_GREEDY_INCREMENTAL_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/solvers/greedy_incremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class GreedyIncrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::GreedyIncremental solver;

public:
    GreedyIncrementalInterface() : AbstractSolverInterface("greedy_incr") {
        add_feasibility_tol_option(solver.feasibility_tol);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "From the base landscape, iteratively add the option with the "
               "best gain/cost ratio.";
    }
};

#endif  // GECOT_GREEDY_INCREMENTAL_INTERFACE_HPP