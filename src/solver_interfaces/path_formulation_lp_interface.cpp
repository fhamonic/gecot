#include "solver_interfaces/path_formulation_lp_interface.hpp"

gecot::solve_result<Instance> PathFormulationLPInterface::solve(
    const Instance & instance, const gecot::budget_t & B) const {
    auto [solution, report, option_bounds] = relax(instance, B);
    return {std::move(solution), std::move(report)};
}

gecot::solvers::path_formulation_lp::relaxation<Instance>
PathFormulationLPInterface::relax(const Instance & instance,
                                  const gecot::budget_t & B) const {
    const auto started = with_mip_start(solver, instance, B);
    std::vector<double> option_bounds;
    auto [solution, report] = with_solver([&]<typename LP, typename MILP>() {
        auto relaxation = started.template relax<LP>(instance, B);
        option_bounds = std::move(relaxation.option_bounds);
        return gecot::solve_result<Instance>{std::move(relaxation.solution),
                                             std::move(relaxation.report)};
    });
    return {std::move(solution), std::move(report), std::move(option_bounds)};
}
