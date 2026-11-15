#pragma once

#include "gecot/solvers/multiflow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MultiflowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::multiflow_benders solver;

public:
    MultiflowBendersInterface() : AbstractMipInterface("multiflow_benders") {
        add_mip_options(solver, lazy_constraint_solvers);
        add_preprocessing_options(solver);
        app.add_flag("--no-root-cuts", solver.no_root_cuts,
                     "For comparison: no cut on the linear relaxation before "
                     "the search");
        app.add_flag("--no-in-out", solver.no_in_out,
                     "For comparison: the cuts of each optimum of the "
                     "relaxation, without the center that moves halfway to "
                     "it");
        app.add_flag("--no-stored-arcs", solver.no_stored_arcs,
                     "Keeps no set A_s of arcs per source but rebuilds them at "
                     "each separation: the same cuts, slower, in less memory "
                     "when the sets add up to too much");
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
