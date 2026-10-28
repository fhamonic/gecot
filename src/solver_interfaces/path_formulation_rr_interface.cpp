#include "solver_interfaces/path_formulation_rr_interface.hpp"

gecot::solve_result<Instance> PathFormulationRRInterface::solve(
    const Instance & instance, const double B) const {
    return with_solver([&]<typename LP, typename MILP>() {
        return solver.template solve<LP>(instance, B);
    });
}
