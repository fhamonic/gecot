#pragma once

#include "gecot/solvers/multiflow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MultiflowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::multiflow_benders solver;

public:
    MultiflowBendersInterface() : AbstractMipInterface("multiflow_benders") {
        add_mip_options(solver, 1e-7, lazy_constraint_solvers);
        add_preprocessing_options(solver);
        app.add_flag("--no-root-cuts", solver.no_root_cuts,
                     "For comparison: no cut on the linear relaxation before "
                     "the search");
        app.add_flag("--no-in-out", solver.no_in_out,
                     "For comparison: the cuts of each optimum of the "
                     "relaxation, without the center that moves halfway to "
                     "it");
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "Benders decomposition of the multicommodity flows, one per "
               "source and target, its cuts separated on the linear "
               "relaxation first";
    }
};
