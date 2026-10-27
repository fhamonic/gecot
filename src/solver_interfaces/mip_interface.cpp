#include "solver_interfaces/mip_interface.hpp"

gecot::solve_result<Instance> mipInterface::solve(const Instance & instance,
                                                  const double B) const {
    return with_solver([&]<typename LP, typename MILP>() {
        return solver.template solve<MILP>(instance, B);
    });
}
