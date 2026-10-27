#include "solver_interfaces/tree_formulation_rr_cm_interface.hpp"

gecot::solve_result<Instance> TreeFormulationRRCMInterface::solve(
    const Instance & instance, const double B) const {
    return with_solver([&]<typename LP, typename MILP>() {
        return solver.template solve<LP, MILP>(instance, B);
    });
}
