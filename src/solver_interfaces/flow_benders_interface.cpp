#include "solver_interfaces/flow_benders_interface.hpp"

gecot::solve_result<Instance> FlowBendersInterface::solve(
    const Instance & instance, const double B) const {
    const auto started = with_mip_start(solver, instance, B);
    return with_solver(
        [&]<typename LP, gecot::solvers::lazy_constraint_model MILP>() {
            return started.template solve<MILP>(instance, B);
        });
}
