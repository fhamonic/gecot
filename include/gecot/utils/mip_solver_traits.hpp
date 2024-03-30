#ifndef GECOT_MIP_SOLVER_TRAITS
#define GECOT_MIP_SOLVER_TRAITS

#include <filesystem>
#include <memory>

#include <spdlog/spdlog.h>

#include "mippp/solver_traits/all.hpp"

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

struct mip_solver_traits {
    enum opt_sense : int { min = -1, max = 1 };
    enum var_category : char { continuous = 0, integer = 1, binary = 2 };
    enum ret_code : int { success = 0, infeasible = 1, timeout = 2 };

    enum solvers : int { grb = 0, scip = 1, cbc = 2 };

    struct solver_wrapper : public mippp::abstract_solver_wrapper {
        std::unique_ptr<mippp::abstract_solver_wrapper> solver;

        [[nodiscard]] solver_wrapper(const auto & model) {
            using traits =
                typename std::decay_t<decltype(model)>::solver_traits;

            if(traits::prefered_solver.has_value() && traits::prefered_solver.value() == traits::solvers::grb && !mippp::cli_grb_traits::is_available()) {
                spdlog::warn("Gurboi is prefered but not available at '{}'", mippp::cli_grb_traits::exec_path.string());
            }
            if(traits::prefered_solver.has_value() && traits::prefered_solver.value() == traits::solvers::scip && !mippp::cli_scip_traits::is_available() ) {
                spdlog::warn("SCIP is prefered but not available at '{}'", mippp::cli_scip_traits::exec_path.string());
            }
            if(traits::prefered_solver.has_value() && traits::prefered_solver.value() == traits::solvers::cbc && !mippp::cli_cbc_traits::is_available()) {
                spdlog::warn("Cbc is prefered but not available at '{}'", mippp::cli_cbc_traits::exec_path.string());
            }

            if(mippp::cli_grb_traits::is_available() &&
               (!traits::prefered_solver.has_value() ||
                traits::prefered_solver.value() == traits::solvers::grb)) {
                solver =
                    std::make_unique<mippp::cli_grb_traits::solver_wrapper>(
                        model);
                spdlog::info("Selected Gurobi as MIP solver");

            } else if(traits::prefered_solver.has_value() &&
                       traits::prefered_solver.value() ==
                           traits::solvers::scip) {
                solver =
                    std::make_unique<mippp::cli_scip_traits::solver_wrapper>(
                        model);
                spdlog::info("Selected SCIP as MIP solver");
            } else {
                solver =
                    std::make_unique<mippp::cli_cbc_traits::solver_wrapper>(
                        model);
                spdlog::info("Selected Cbc as MIP solver");
            }
        }

        [[nodiscard]] solver_wrapper(const solver_wrapper &) = default;
        [[nodiscard]] solver_wrapper(solver_wrapper &&) = default;

        ~solver_wrapper() {}

        void set_loglevel(int loglevel) noexcept {
            solver->set_loglevel(loglevel);
        }
        void set_timeout(int timeout_s) noexcept {
            solver->set_timeout(timeout_s);
        }
        void set_mip_gap(double precision) noexcept {
            solver->set_mip_gap(precision);
        }

    public:
        int optimize() noexcept { return solver->optimize(); }

        [[nodiscard]] std::vector<double> get_solution() const noexcept {
            return solver->get_solution();
        }
        [[nodiscard]] double get_objective_value() const noexcept {
            return solver->get_objective_value();
        }
    };

    static inline std::optional<solvers> prefered_solver;
    static solver_wrapper build(const auto & model) {
        solver_wrapper solver(model);
        return solver;
    }
};

}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_MIP_SOLVER_TRAITS