#ifndef GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP
#define GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/solvers/greedy_decremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class GreedyDecrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::GreedyDecremental solver;

public:
    GreedyDecrementalInterface() : AbstractSolverInterface("greedy_decr") {
        add_feasibility_tol_option(solver.feasibility_tol);
        app.add_flag("--only-dec", solver.only_dec,
                     "Do not perform the final incremental steps that ensure "
                     "that the entire budget is used");
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "From the improved landscape, iteratively remove the option "
               "with "
               "the worst gain/cost ratio (Zonation Algorithm).";
    }
};

#endif  // GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP