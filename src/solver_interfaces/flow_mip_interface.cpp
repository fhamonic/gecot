#include "solver_interfaces/flow_mip_interface.hpp"

gecot::solve_result<Instance> FlowMipInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    gecot::check_plain_budget(instance, name());
    const auto started = with_mip_start(solver, instance, B);
    return with_solver([&]<typename LP, typename MILP>() {
        return started.template solve<MILP>(instance, B.front());
    });
}
