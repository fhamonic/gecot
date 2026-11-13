#include "solver_interfaces/flow_benders_interface.hpp"

gecot::solve_result<Instance> FlowBendersInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    gecot::check_plain_budget(instance, name());
    const auto started = with_mip_start(solver, instance, B);
    return with_solver(
        [&]<typename LP, gecot::solvers::lazy_constraint_model MILP>() {
            return started.template solve<MILP>(instance, B.front());
        });
}
