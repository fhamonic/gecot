#ifndef GECOT_MIP_HELPER
#define GECOT_MIP_HELPER

#include <filesystem>
#include <memory>

#include <spdlog/spdlog.h>

#include "mippp/solvers/all.hpp"

#ifdef WIN32
#include <windows.h>
#endif
std::filesystem::path get_exec_path() {
#ifdef WIN32
    char buffer[MAX_PATH];
    GetModuleFileName(NULL, buffer, MAX_PATH);
    return std::filesystem::path(buffer);
#else
    return std::filesystem::canonical("/proc/self/exe");
#endif
}

namespace fhamonic {
namespace gecot {

struct mip_helper {
    enum solvers : int { grb = 0, scip = 1, cbc = 2 };
    static inline std::optional<solvers> prefered_solver;
    static std::unique_ptr<mippp::abstract_cli_solver_wrapper> build_solver(
        const auto & model) {
        std::unique_ptr<mippp::abstract_cli_solver_wrapper> solver;

        if(prefered_solver.has_value() &&
           prefered_solver.value() == solvers::grb &&
           !mippp::cli_grb_solver::is_available()) {
            spdlog::warn("Gurboi is prefered but not available at '{}'",
                         mippp::cli_grb_solver::exec_path.string());
        }
        if(prefered_solver.has_value() &&
           prefered_solver.value() == solvers::scip &&
           !mippp::cli_scip_solver::is_available()) {
            spdlog::warn("SCIP is prefered but not available at '{}'",
                         mippp::cli_scip_solver::exec_path.string());
        }
        if(prefered_solver.has_value() &&
           prefered_solver.value() == solvers::cbc &&
           !mippp::cli_cbc_solver::is_available()) {
            spdlog::warn("Cbc is prefered but not available at '{}'",
                         mippp::cli_cbc_solver::exec_path.string());
        }

        if(mippp::cli_grb_solver::is_available() &&
           (!prefered_solver.has_value() ||
            prefered_solver.value() == solvers::grb)) {
            solver = std::make_unique<mippp::cli_grb_solver>(model);

        } else if(prefered_solver.has_value() &&
                  prefered_solver.value() == solvers::scip) {
            solver = std::make_unique<mippp::cli_scip_solver>(model);
        } else {
            solver = std::make_unique<mippp::cli_cbc_solver>(model);
        }

        spdlog::info("Selected {} as MIP solver", solver->name());
        return solver;
    }
};

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_MIP_HELPER