#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/scip/all.hpp"
#include "mippp/solvers/xpress/all.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class AbstractMipInterface : public AbstractSolverInterface {
public:
    using AbstractSolverInterface::AbstractSolverInterface;

    static std::vector<std::string> parse_mip_start(
        const std::filesystem::path & solution_path) {
        if(!std::filesystem::exists(solution_path))
            throw std::invalid_argument("File '" + solution_path.string() +
                                        "' does not exists");
        if(std::filesystem::is_directory(solution_path))
            throw std::invalid_argument("'" + solution_path.string() +
                                        "' is a directory");

        std::ifstream instance_stream(solution_path);
        nlohmann::json solution_json;
        instance_stream >> solution_json;

        std::vector<std::string> taken_option_names;
        for(auto && [option_name, value] : solution_json["solution"].items()) {
            if(!value.template get<int>()) continue;
            taken_option_names.emplace_back(option_name);
        }
        return taken_option_names;
    }

protected:
    std::string solver_name;

    static inline const std::vector<std::string> all_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek",
        "highs",  "scip",  "cbc",  "glpk"};
    static inline const std::vector<std::string> lazy_constraint_solvers{
        "gurobi", "cplex", "copt"};
    // GLPK is not thread-safe, tree_rr prices in parallel
    static inline const std::vector<std::string> thread_safe_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek", "highs", "scip", "cbc"};

    void add_mip_options(
        gecot::solvers::mip_solver_base & solver, double default_tol = 1e-7,
        const std::vector<std::string> & solvers = all_solvers) {
        app.add_option("--solver", solver_name, "Solver used through mippp")
            ->check(CLI::IsMember(solvers))
            ->default_val("gurobi");
        add_feasibility_tol_option(solver.feasibility_tol, default_tol);
        app.add_option_function<std::filesystem::path>(
            "--mip-start",
            [&solver](const std::filesystem::path & solution_path) {
                solver.mip_start = parse_mip_start(solution_path);
            },
            "Solution file (JSON) to propose as a MIP start to the solver.");
        app.add_option("--time-limit", solver.time_limit,
                       "Time limit in seconds, after which the best solution "
                       "found is returned")
            ->check(CLI::NonNegativeNumber);
    }

    // f.template operator()<LP, MILP>() with the --solver mippp models
    template <typename F>
    gecot::solve_result<Instance> with_solver(F && f) const {
        using namespace mippp;
        if(solver_name == "gurobi") return call<gurobi_lp, gurobi_milp>(f);
        if(solver_name == "cplex") return call<cplex_lp, cplex_milp>(f);
        if(solver_name == "copt") return call<copt_lp, copt_milp>(f);
        if(solver_name == "xpress") return call<xpress_lp, xpress_milp>(f);
        if(solver_name == "mosek") return call<mosek_lp, mosek_milp>(f);
        if(solver_name == "highs") return call<highs_lp, highs_milp>(f);
        // SCIP embeds a SoPlex that clashes with a loaded libsoplex
        if(solver_name == "scip") return call<highs_lp, scip_milp>(f);
        if(solver_name == "cbc") return call<clp_lp, cbc_milp>(f);
        if(solver_name == "glpk") return call<glpk_lp, glpk_milp>(f);
        throw std::invalid_argument("unknown solver '" + solver_name + "'");
    }

    void add_preprocessing_options(gecot::solvers::mip_solver_base & solver) {
        app.add_option("--resolution", solver.probability_resolution,
                       "Sets the resolution of the probabilities used by the "
                       "preprocessing algorithm")
            ->default_val(1e-8);
        app.add_option("--nb-mus", solver.num_mus,
                       "Sets the number of lagrangian multipliers to test for "
                       "the constrained preprocessing algorithm (experimental)")
            ->default_val(0);
    }

private:
    template <typename LP, typename MILP, typename F>
    gecot::solve_result<Instance> call(F && f) const {
        if constexpr(requires { f.template operator()<LP, MILP>(); })
            return f.template operator()<LP, MILP>();
        else
            throw std::invalid_argument(name() + " cannot use solver '" +
                                        solver_name + "'");
    }
};
