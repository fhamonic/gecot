#pragma once

#include <limits>

#include "gecot/solvers/multiflow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MultiflowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::multiflow_benders solver;

public:
    MultiflowBendersInterface() : AbstractMipInterface("multiflow_benders") {
        add_benders_options(solver, column_solvers);
        add_preprocessing_options(solver);
        add_big_M_option(solver.big_Ms);
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
        app.add_flag_callback(
            "--no-root-cuts", [this] { solver.root_cuts = false; },
            "For comparison: no cut on the linear relaxation before the "
            "search");
        app.add_flag_callback(
            "--no-in-out", [this] { solver.in_out = false; },
            "For comparison: the cuts of each optimum of the relaxation, "
            "without the center that moves halfway to it");
        app.add_flag_callback(
            "--no-stored-arcs", [this] { solver.stored_arcs = false; },
            "Keeps no set A_s of arcs per source but rebuilds them at each "
            "separation: the same cuts, slower, in less memory when the "
            "sets add up to too much");
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "Benders decomposition of the multicommodity flows, one per "
               "source and target, its cuts separated on the linear "
               "relaxation first";
    }
    std::optional<std::string> bound_description() const override {
        return "Dual bound of the multiflow Benders master; needs an "
               "incumbent.";
    }
};
