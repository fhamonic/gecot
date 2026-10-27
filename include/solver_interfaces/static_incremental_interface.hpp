#ifndef GECOT_STATIC_INCREMENTAL_INTERFACE_HPP
#define GECOT_STATIC_INCREMENTAL_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/solvers/static_incremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class StaticIncrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::StaticIncremental solver;

public:
    StaticIncrementalInterface() : AbstractSolverInterface("static_incr") {
        add_feasibility_tol_option(solver.feasibility_tol);
    }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override {
        return {solver.solve(instance, B)};
    };

    std::string description() const override {
        return "From the base landscape, add the options with the best "
               "gain/cost ratio.";
    }
};

#endif  // GECOT_STATIC_INCREMENTAL_INTERFACE_HPP