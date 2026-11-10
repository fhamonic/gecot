#include "solver_interfaces/path_formulation_lp_interface.hpp"

gecot::solve_result<Instance> PathFormulationLPInterface::solve(
    const Instance & instance, const double B) const {
    const auto started = with_mip_start(solver, instance, B);
    return with_solver([&]<typename LP, typename MILP>() {
        return started.template solve<LP>(instance, B);
    });
}
