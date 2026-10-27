#include "solver_interfaces/target_benders_interface.hpp"

gecot::solve_result<Instance> TargetBendersInterface::solve(
    const Instance & instance, const double B) const {
    return with_solver(
        [&]<typename LP, gecot::solvers::lazy_constraint_model MILP>() {
            return solver.template solve<MILP>(instance, B);
        });
}
