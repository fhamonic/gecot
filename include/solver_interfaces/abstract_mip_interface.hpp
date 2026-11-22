#pragma once

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
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
// and for the three that the help of --root-cut-purge names, and one that
// keeps its rows, with which the parity matrix runs the warning
static_assert(mippp::has_remove_constraint<mippp::gurobi_milp>);
static_assert(mippp::has_remove_constraint<mippp::cplex_milp>);
static_assert(mippp::has_remove_constraint<mippp::highs_milp>);
static_assert(!mippp::has_remove_constraint<mippp::cbc_milp>);

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

    // whether --root-cut-purge was given, for the warning of a solver that
    // cannot remove rows: the solve sets it on its copy of the solver
    bool root_cut_purge_given() const {
        return app.count("--root-cut-purge") > 0;
    }
    // The rounds of cuts on the relaxation of multiflow_benders, which
    // multiflow_rounding solves too
    void add_root_cut_options(auto & solver) {
        app.add_option("--root-cut-tolerance", solver.root_cut_tol,
                       "Relative tolerance of the cuts on the linear "
                       "relaxation before the search: a cut is added when the "
                       "relaxation's flow exceeds it by more than this share, "
                       "and the rounds end when none does. A larger value "
                       "ends them sooner, the relaxation then above its "
                       "value with all the cuts by about this share at most, "
                       "unless the criterion subtracts a constant")
            ->check(CLI::Range(gecot::default_feasibility_tol,
                               std::numeric_limits<double>::max(),
                               "AT LEAST 1e-07"))
            ->default_val(solver.root_cut_tol);
        app.add_option("--root-cut-gap", solver.root_cut_gap,
                       "Ends the cuts on the linear relaxation before the "
                       "search once the max gain of their table is within "
                       "this gap of its fractional gain, relative to the "
                       "score of the fractional gain (0: never, the rounds "
                       "then end by --root-cut-tolerance only)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(solver.root_cut_gap);
        app.add_option("--root-cut-purge", solver.root_cut_purge,
                       "Removes from the linear relaxation before the search "
                       "the row of each cut slack, by --root-cut-tolerance, "
                       "at this many of its optima in a row, with gurobi, "
                       "cplex and highs: rounds that converge end at the same "
                       "value within --root-cut-tolerance, usually with fewer "
                       "rows, rounds cut short can end higher, and "
                       "multiflow_benders searches with the rows left. A small "
                       "value adds removed cuts back (0: never)")
            ->default_val(solver.root_cut_purge);
        app.add_option("--in-out-ratio", solver.in_out_ratio,
                       "Step of the center of the cuts on the linear "
                       "relaxation before the search: the share of the way "
                       "to each new optimum of the relaxation that it moves, "
                       "from the first optimum. 1 separates each optimum "
                       "itself, without the center")
            ->check(CLI::Range(0.0, 1.0))
            ->default_val(solver.in_out_ratio);
        app.add_option("--in-out-separation", solver.in_out_separation,
                       "Point separated by the cuts on the linear relaxation "
                       "before the search: the share of the way from the "
                       "center to each new optimum of the relaxation, before "
                       "the center moves (default: --in-out-ratio, the new "
                       "center)")
            ->check(CLI::Range(0.0, 1.0));
        app.add_option("--in-out-lift", solver.in_out_lift,
                       "Lift of the cuts on the linear relaxation before the "
                       "search: each option of the point they separate is "
                       "raised to at least this value, until they add no cut; "
                       "the optimum itself is separated from then on (0: "
                       "never)")
            ->check(CLI::Range(0.0, 1.0))
            ->default_val(solver.in_out_lift);
    }
    void add_source_group_options(auto & solver) {
        app.add_option(
               "--source-group-size", solver.source_group_size,
               "Experimental: the n sources of each target in ceil(n / this) "
               "groups, each with a flow variable that receives their part of "
               "each cut, f_t then bounded by their sum: the same relaxation "
               "once converged, and a master that combines the parts of "
               "different cuts, with a variable per group (0: no group, the "
               "cuts on f_t; 1: a variable per source). A target left with "
               "one group keeps the cuts on f_t, and, but with "
               "--no-stored-arcs, a source with no improvable arc on its best "
               "paths is written exactly, in no group")
            ->default_val(solver.source_group_size);
        // dependent: the interfaces without multiflow do not see the enum
        using grouping = std::remove_cvref_t<decltype(solver.grouping)>;
        add_choice_option<grouping>(
            app, "--source-grouping", solver.grouping,
            {{"voronoi", grouping::voronoi},
             {"consecutive", grouping::consecutive},
             {"random", grouping::random}},
            [&solver](const grouping groups) { solver.grouping = groups; },
            "With --source-group-size, voronoi: the cells of a network "
            "Voronoi diagram of the target's graph around sources of flow far "
            "from each other; consecutive: groups of consecutive sources; "
            "random: groups of sources drawn at random, as a control");
        app.add_option("--source-grouping-seed", solver.grouping_seed,
                       "Seed of --source-grouping random")
            ->default_val(solver.grouping_seed);
    }
    void add_stored_arcs_option(auto & solver) {
        app.add_flag_callback(
            "--no-stored-arcs", [&solver] { solver.stored_arcs = false; },
            "Keeps no set A_s of arcs per source but rebuilds them at each "
            "separation: the same cuts, slower, in less memory when the "
            "sets add up to too much");
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
        // --nb-mus: its name in GECOT 1.0
        app.add_option("--mu-steps,--nb-mus", solver.num_mus,
                       "Sets the number of geometric steps between the "
                       "lagrangian multipliers of the constrained "
                       "preprocessing, which tests N + 1 positive ones after 0 "
                       "(0: unconstrained preprocessing; experimental: 20, "
                       "or 9 on large instances)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(solver.num_mus);
        add_choice_option<bool>(
            app, "--mu-sweep", solver.decades_sweep,
            {{"decades", true}, {"full", false}},
            [&solver](const bool decades) { solver.decades_sweep = decades; },
            "Range of the positive multipliers, up to the largest useful "
            "one: decades spans at most the 3 decades below it and tests 0 "
            "again last, full starts from the inverse of the largest option "
            "cost per arc (needs --mu-steps > 0; experimental)");
        app.add_flag_callback(
            "--no-strong-arcs",
            [&solver] { solver.contract_strong_arcs = false; },
            "For comparison: the strong arcs are not contracted");
        app.add_flag_callback(
            "--no-useless-arcs",
            [&solver] { solver.remove_useless_arcs = false; },
            "For comparison: the useless arcs are not removed; with "
            "--no-strong-arcs, no arc is labelled, and --resolution, "
            "--mu-steps and --mu-sweep have no effect");
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
