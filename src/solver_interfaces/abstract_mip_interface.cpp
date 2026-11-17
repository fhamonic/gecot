#include "solver_interfaces/abstract_mip_interface.hpp"

#include <algorithm>
#include <format>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "gecot/helper.hpp"
#include "gecot/solvers/greedy.hpp"
#include "gecot/solvers/static.hpp"

void AbstractMipInterface::compute_mip_start(
    gecot::solvers::mip_solver_base & solver, const Instance & instance,
    const gecot::budget_t & B) const {
    // or with that of set_start: --mip-start excludes the algorithm
    if(mip_start_algorithm == "none" || solver.mip_start) return;
    spdlog::info("Computing the MIP start with {}", mip_start_algorithm);
    const spdlog::stopwatch sw;
    const double tol = solver.feasibility_tol;
    using namespace gecot::solvers;
    const auto start =
        mip_start_algorithm == "static"
            ? Static{.feasibility_tol = tol}.solve(instance, B)
            : Greedy{.feasibility_tol = tol}.solve<spdlog::level::trace>(
                  instance, B);
    solver.mip_start.emplace();
    for(auto && o : instance.options())
        if(start[o]) solver.mip_start->push_back(instance.option_name(o));
    if(spdlog::should_log(spdlog::level::info))
        spdlog::info(
            "MIP start: {}{}",
            gecot::gain_text(instance.eval_criterion(
                gecot::compute_solution_cases_pc_num(instance, start))),
            B.size() == 1
                ? std::format(", cost {:.10g} of {:.10g}",
                              gecot::compute_solution_cost(instance, start, 0),
                              B[0])
                : std::string());
    if(solver.time_limit) {
        solver.time_limit =
            std::max(0.0, *solver.time_limit - sw.elapsed().count());
        if(*solver.time_limit == 0)
            spdlog::warn(
                "No time is left for the algorithm after the MIP start");
    }
}
