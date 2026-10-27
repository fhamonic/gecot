#include "solver_interfaces/flow_benders_interface.hpp"

gecot::solve_result<Instance> FlowBendersInterface::solve(
    const Instance & instance, const double B) const {
    return with_solver(
        [&]<typename LP, gecot::solvers::lazy_constraint_model MILP>() {
            return solver.template solve<MILP>(instance, B);
        });
}
