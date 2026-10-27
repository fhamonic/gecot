#include "solver_interfaces/tree_formulation_rr_interface.hpp"

gecot::solve_result<Instance> TreeFormulationRRInterface::solve(
    const Instance & instance, const double B) const {
    return with_solver([&]<typename LP, typename MILP>() {
        return solver.template solve<LP, MILP>(instance, B);
    });
}
