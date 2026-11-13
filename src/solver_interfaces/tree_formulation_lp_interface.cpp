#include "solver_interfaces/tree_formulation_lp_interface.hpp"

gecot::solve_result<Instance> TreeFormulationLPInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    const auto started = with_mip_start(solver, instance, B);
    return with_solver([&]<typename LP, typename MILP>() {
        return started.template solve<LP, MILP>(instance, B.front());
    });
}
