#pragma once

#include "gecot/solvers/tree_formulation_lp.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class TreeFormulationLPInterface : public AbstractMipInterface {
private:
    gecot::solvers::tree_formulation_lp solver;

public:
    TreeFormulationLPInterface() : AbstractMipInterface("tree_lp") {
        add_mip_options(solver, 1e-7, thread_safe_solvers);
        add_preprocessing_options(solver);
        app.add_option("--evict-age", solver.evict_age,
                       "Pricing rounds a column stays in the master before it "
                       "can go back to the pool, if unattractive (0: never)")
            ->default_val(0);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "tree formulation relaxation, one column per target and set of "
               "options";
    }
};
