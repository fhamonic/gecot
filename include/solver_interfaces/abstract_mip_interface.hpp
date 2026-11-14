#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
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

#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/solvers/greedy.hpp"
#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/solvers/static.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

class AbstractMipInterface : public AbstractSolverInterface {
public:
    using AbstractSolverInterface::AbstractSolverInterface;

    static std::vector<std::string> parse_mip_start(
        const std::filesystem::path & solution_path) {
        if(!std::filesystem::exists(solution_path))
            throw std::invalid_argument("File '" + solution_path.string() +
                                        "' does not exist");
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

    // the MIP start, unless --mip-start or --mip-start-algorithm gives one
    void set_start(std::vector<std::string> option_names) override {
        if(!mip_solver->mip_start && mip_start_algorithm == "none")
            mip_solver->mip_start = std::move(option_names);
    }

protected:
    std::string solver_name;
    std::string mip_start_algorithm;
    gecot::solvers::mip_solver_base * mip_solver = nullptr;

    static inline const std::vector<std::string> all_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek",
        "highs",  "scip",  "cbc",  "glpk"};
    static inline const std::vector<std::string> lazy_constraint_solvers{
        "gurobi", "cplex", "copt"};
    // GLPK is not thread-safe, tree_lp prices in parallel
    static inline const std::vector<std::string> thread_safe_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek", "highs", "scip", "cbc"};

    void add_mip_options(
        gecot::solvers::mip_solver_base & solver, double default_tol = 1e-7,
        const std::vector<std::string> & solvers = all_solvers) {
        mip_solver = &solver;
        app.add_option("--solver", solver_name, "Solver used through mippp")
            ->check(CLI::IsMember(solvers))
            ->default_val("gurobi");
        add_feasibility_tol_option(solver.feasibility_tol, default_tol);
        auto * mip_start_option =
            app.add_option_function<std::filesystem::path>(
                "--mip-start",
                [&solver](const std::filesystem::path & solution_path) {
                    solver.mip_start = parse_mip_start(solution_path);
                },
                "Solution file (JSON) to propose as a MIP start to the "
                "solver.");
        app.add_option("--mip-start-algorithm", mip_start_algorithm,
                       "Algorithm whose solution is proposed as a MIP start "
                       "to the solver, its time counted in --time-limit")
            ->check(CLI::IsMember({"none", "static", "greedy"}))
            ->default_val("none")
            ->excludes(mip_start_option);
        app.add_option("--time-limit", solver.time_limit,
                       "Time limit in seconds, after which the best solution "
                       "found is returned")
            ->check(CLI::NonNegativeNumber);
    }

    // solver with the solution of --mip-start-algorithm as its MIP start
    template <typename S>
    S with_mip_start(S solver, const Instance & instance,
                     const gecot::budget_t & B) const {
        if(mip_start_algorithm == "none") return solver;
        spdlog::info("Computing the MIP start with {}", mip_start_algorithm);
        const spdlog::stopwatch sw;
        const double tol = solver.feasibility_tol;
        using namespace gecot::solvers;
        const auto start = mip_start_algorithm == "static"
                               ? Static{tol}.solve(instance, B)
                               : Greedy{tol}.solve(instance, B);
        solver.mip_start.emplace();
        for(auto && o : instance.options())
            if(start[o]) solver.mip_start->push_back(instance.option_name(o));
        if(solver.time_limit)
            solver.time_limit =
                std::max(0.0, *solver.time_limit - sw.elapsed().count());
        return solver;
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

    void add_big_M_option(gecot::big_M_kind & big_Ms) {
        app.add_option_function<std::string>(
               "--big-m",
               [&big_Ms](const std::string & s) {
                   using enum gecot::big_M_kind;
                   big_Ms = s == "plain"              ? plain
                            : s == "knapsack_dantzig" ? knapsack_dantzig
                            : s == "tight"            ? tight
                                                      : knapsack_bnb;
               },
               "The big-Ms: plain, with every quality gain (for comparison); "
               "knapsack_bnb, with the gains of the best options within the "
               "budget, by branch and bound; knapsack_dantzig, with the "
               "linear relaxation of that knapsack, looser but faster; tight "
               "(experimental), knapsack_bnb lowered per arc over the sources "
               "that can use it")
            ->check(CLI::IsMember(
                {"plain", "knapsack_bnb", "knapsack_dantzig", "tight"}))
            ->default_val("knapsack_bnb");
    }

    void add_preprocessing_options(gecot::solvers::mip_solver_base & solver) {
        app.add_option("--resolution", solver.probability_resolution,
                       "Sets the resolution of the probabilities used by the "
                       "preprocessing algorithm")
            ->default_val(1e-8);
        app.add_option("--nb-mus", solver.num_mus,
                       "Sets the number of geometric steps between the "
                       "lagrangian multipliers of the constrained "
                       "preprocessing, which tests N + 1 positive ones after 0 "
                       "(0: unconstrained preprocessing; experimental: 20, "
                       "or 9 on large instances)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(0);
        app.add_option_function<std::string>(
               "--mus-sweep",
               [&solver](const std::string & s) {
                   solver.decades_sweep = s == "decades";
               },
               "Range of the positive multipliers, up to the largest useful "
               "one: decades spans at most the 3 decades below it and tests 0 "
               "again last, full starts from the inverse of the largest option "
               "cost per arc (needs --nb-mus > 0; experimental)")
            ->check(CLI::IsMember({"decades", "full"}))
            ->default_val("decades");
        app.add_option_function<std::string>(
               "--preprocessing",
               [&solver](const std::string & s) {
                   solver.contract_strong_arcs = s == "all" || s == "strong";
                   solver.remove_useless_arcs = s == "all" || s == "useless";
               },
               "For comparison, what the strong and useless arcs are used "
               "for: all; strong, only the contraction of the strong arcs; "
               "useless, only the removal of the useless arcs; none, where "
               "--resolution, --nb-mus and --mus-sweep have no effect")
            ->check(CLI::IsMember({"all", "strong", "useless", "none"}))
            ->default_val("all");
        app.add_flag("--all-targets", solver.all_targets,
                     "For comparison: the vertices that can never be targets "
                     "are preprocessed and modelled as the others");
        app.add_flag("--no-pruning", solver.no_pruning,
                     "For comparison: keeps, in the graph of a target, the "
                     "arcs that leave it and the vertices that no quality "
                     "reaches");
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
