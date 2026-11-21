#pragma once

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
        add_root_cut_options(solver);
        app.add_flag_callback(
            "--no-root-cuts", [this] { solver.root_cuts = false; },
            "For comparison: no cut on the linear relaxation before the "
            "search");
        add_stored_arcs_option(solver);
        add_source_group_options(solver);
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
