#ifndef GECOT_STATIC_INTERFACE_HPP
#define GECOT_STATIC_INTERFACE_HPP

#include <CLI/CLI.hpp>

#include "gecot/solvers/static.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class StaticInterface : public AbstractSolverInterface {
private:
    gecot::solvers::Static solver;

public:
    StaticInterface() : AbstractSolverInterface("static") {
        add_feasibility_tol_option(solver.feasibility_tol);
        auto * decremental = app.add_flag(
            "--decremental", solver.decremental,
            "From the improved landscape, remove the options with the worst "
            "gain/cost ratio until the budget is met, then add back those "
            "that fit");
        app.add_flag("--only-dec", solver.only_dec,
                     "Do not perform the final incremental steps that ensure "
                     "that the entire budget is used")
            ->needs(decremental);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override {
        return {solver.solve(instance, B)};
    };

    std::string description() const override {
        return "From the base landscape, add the options with the best "
               "gain/cost ratio; with --decremental, remove from the improved "
               "landscape those with the worst one.";
    }
};

#endif  // GECOT_STATIC_INTERFACE_HPP