#include "solver_interfaces/flow_benders_interface.hpp"

gecot::solve_result<Instance> FlowBendersInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    const auto started = with_mip_start(solver, instance, B);
    return with_solver([&]<typename LP, mippp::has_column_generation MILP>() {
        return started.template solve<MILP>(instance, B);
    });
}
