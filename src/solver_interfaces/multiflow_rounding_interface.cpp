#include "solver_interfaces/multiflow_rounding_interface.hpp"

gecot::solve_result<Instance> MultiflowRoundingInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    auto started = with_mip_start(solver, instance, B);
    started.root_cut_purge_given = root_cut_purge_given();
    return with_solver([&]<typename LP, mippp::has_column_generation MILP>() {
        return started.template solve<MILP>(instance, B);
    });
}
