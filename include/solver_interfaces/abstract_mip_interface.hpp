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

#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/solvers/benders_base.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

// the backends that the help of --no-lazy-constraints names: without these,
// one that no longer satisfied the concept would search without lazy
// constraints without a word
static_assert(gecot::solvers::lazy_constraint_model<mippp::gurobi_milp>);
static_assert(gecot::solvers::lazy_constraint_model<mippp::cplex_milp>);
static_assert(gecot::solvers::lazy_constraint_model<mippp::copt_milp>);
// column_solvers leaves SCIP out: its mippp model has no add_column, with
// which flow_benders and multiflow_benders build their masters
static_assert(!mippp::has_column_generation<mippp::scip_milp>);
// the same for the two backends that the help of --indicator-constraints names
static_assert(mippp::has_indicator_constraints<mippp::gurobi_milp>);
static_assert(mippp::has_indicator_constraints<mippp::cplex_milp>);

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

        // not solution_json["solution"]: on a file without one it inserts
        // a null, and the start is empty without a word
        if(!solution_json.is_object() || !solution_json.contains("solution") ||
           !solution_json.at("solution").is_object())
            throw std::invalid_argument("File '" + solution_path.string() +
                                        "' has no \"solution\" object");
        std::vector<std::string> taken_option_names;
        for(auto && [option_name, value] :
            solution_json.at("solution").items()) {
            if(!value.template get<int>()) continue;
            taken_option_names.emplace_back(option_name);
        }
        return taken_option_names;
    }

    // the MIP start, unless --mip-start gives one; with_mip_start then does
    // not compute that of --mip-start-algorithm
    bool set_start(std::vector<std::string> option_names) override {
        if(mip_solver->mip_start) return false;
        mip_solver->mip_start = std::move(option_names);
        return true;
    }

protected:
    std::string solver_name = "gurobi";
    std::string mip_start_algorithm = "none";
    gecot::solvers::mip_solver_base * mip_solver = nullptr;

    static inline const std::vector<std::string> all_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek",
        "highs",  "scip",  "cbc",  "glpk"};
    static inline const std::vector<std::string> column_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek", "highs", "cbc", "glpk"};
    // GLPK is not thread-safe, tree_lp prices in parallel
    static inline const std::vector<std::string> thread_safe_solvers{
        "gurobi", "cplex", "copt", "xpress", "mosek", "highs", "scip", "cbc"};

    void add_mip_options(
        gecot::solvers::mip_solver_base & solver,
        const std::vector<std::string> & solvers = all_solvers) {
        mip_solver = &solver;
        app.add_option("--solver", solver_name, "Solver used through mippp")
            ->check(CLI::IsMember(solvers))
            ->default_val(solver_name);
        add_feasibility_tol_option(solver.feasibility_tol);
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
            ->default_val(mip_start_algorithm)
            ->excludes(mip_start_option);
        app.add_option("--time-limit", solver.time_limit,
                       "Time limit in seconds, after which the best solution "
                       "found is returned")
            ->check(CLI::NonNegativeNumber);
        app.add_flag("--numeric-focus", solver.numeric_focus,
                     "Asks the solver for more care in its numerical "
                     "computations, at some cost in time: NumericFocus 1 of "
                     "gurobi, the numerical emphasis of cplex, "
                     "NUMERICALEMPHASIS 1 of xpress, and for the MIP models "
                     "of mosek its numerical emphasis level 1; copt, highs, "
                     "cbc and glpk have none, and the numerics emphasis of "
                     "scip is out of mippp's reach: they warn");
    }

    // add_mip_options, for a Benders decomposition
    void add_benders_options(
        gecot::solvers::benders_base & solver,
        const std::vector<std::string> & solvers = all_solvers) {
        add_mip_options(solver, solvers);
        app.add_flag_callback(
            "--no-lazy-constraints",
            [&solver] { solver.lazy_constraints = false; },
            "For comparison: solves the master again with the cuts of its "
            "solution until it receives none, as with the solvers that have "
            "no lazy constraints (all but gurobi, cplex and copt)");
    }

    // solver with the solution of --mip-start-algorithm as its MIP start
    template <typename S>
    S with_mip_start(S solver, const Instance & instance,
                     const gecot::budget_t & B) const {
        compute_mip_start(solver, instance, B);
        return solver;
    }
    // Gives solver the solution of --mip-start-algorithm as its MIP start,
    // and takes the time of the heuristic off its time limit. In a source
    // file of its own: the heuristics are compiled there once, not in the
    // unit of each algorithm.
    void compute_mip_start(gecot::solvers::mip_solver_base & solver,
                           const Instance & instance,
                           const gecot::budget_t & B) const;

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
        using enum gecot::big_M_kind;
        add_choice_option<gecot::big_M_kind>(
            app, "--big-m", big_Ms,
            {{"plain", plain},
             {"knapsack_bnb", knapsack_bnb},
             {"knapsack_dantzig", knapsack_dantzig},
             {"tight", tight},
             {"exact", exact}},
            [&big_Ms](const gecot::big_M_kind kind) { big_Ms = kind; },
            "The big-Ms: plain, with every quality gain (for comparison); "
            "knapsack_bnb, with the gains of the best options within the "
            "budget, by branch and bound; knapsack_dantzig, with the "
            "linear relaxation of that knapsack, looser but faster; tight "
            "(experimental), knapsack_bnb lowered per improvement and per "
            "target quality gain over the sources that can use them; exact "
            "(for comparison, slow), the largest flow that can reach each "
            "vertex within the budget, which knapsack_bnb bounds, by a MIP "
            "per vertex");
    }

    void add_preprocessing_options(
        gecot::solvers::flow_preprocessing & solver) {
        app.add_option("--resolution", solver.probability_resolution,
                       "Sets the resolution of the probabilities used by the "
                       "preprocessing algorithm")
            ->default_val(solver.probability_resolution);
        app.add_option("--nb-mus", solver.num_mus,
                       "Sets the number of geometric steps between the "
                       "lagrangian multipliers of the constrained "
                       "preprocessing, which tests N + 1 positive ones after 0 "
                       "(0: unconstrained preprocessing; experimental: 20, "
                       "or 9 on large instances)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(solver.num_mus);
        add_choice_option<bool>(
            app, "--mus-sweep", solver.decades_sweep,
            {{"decades", true}, {"full", false}},
            [&solver](const bool decades) { solver.decades_sweep = decades; },
            "Range of the positive multipliers, up to the largest useful "
            "one: decades spans at most the 3 decades below it and tests 0 "
            "again last, full starts from the inverse of the largest option "
            "cost per arc (needs --nb-mus > 0; experimental)");
        // contract_strong_arcs, remove_useless_arcs
        add_choice_option<std::pair<bool, bool>>(
            app, "--preprocessing",
            {solver.contract_strong_arcs, solver.remove_useless_arcs},
            {{"all", {true, true}},
             {"strong", {true, false}},
             {"useless", {false, true}},
             {"none", {false, false}}},
            [&solver](const std::pair<bool, bool> & uses) {
                solver.contract_strong_arcs = uses.first;
                solver.remove_useless_arcs = uses.second;
            },
            "For comparison, what the strong and useless arcs are used "
            "for: all; strong, only the contraction of the strong arcs; "
            "useless, only the removal of the useless arcs; none, where "
            "--resolution, --nb-mus and --mus-sweep have no effect");
        app.add_flag("--all-targets", solver.all_targets,
                     "For comparison: the vertices that can never be targets "
                     "are preprocessed and modelled as the others");
        app.add_flag_callback(
            "--no-pruning", [&solver] { solver.pruning = false; },
            "For comparison: keeps, in the graph of a target, the arcs that "
            "leave it and the vertices that no quality reaches");
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
