#ifndef GECOT_STATIC_DECREMENTAL_INTERFACE_HPP
#define GECOT_STATIC_DECREMENTAL_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/solvers/static_decremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class StaticDecrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::StaticDecremental solver;

public:
    StaticDecrementalInterface() : AbstractSolverInterface("static_decr") {
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
        return "From the improved landscape, remove the options with the worst "
               "gain/cost ratio.";
    }
};

#endif  // GECOT_STATIC_DECREMENTAL_INTERFACE_HPP