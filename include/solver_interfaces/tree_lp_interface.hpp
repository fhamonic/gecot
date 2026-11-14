#pragma once

#include "gecot/solvers/tree_lp.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class TreeLpInterface : public AbstractMipInterface {
private:
    gecot::solvers::tree_lp solver;

public:
    TreeLpInterface() : AbstractMipInterface("tree_lp") {
        add_mip_options(solver, thread_safe_solvers);
        add_preprocessing_options(solver);
        app.add_option("--evict-age", solver.evict_age,
                       "Pricing rounds a column stays in the master before it "
                       "can go back to the pool, if unattractive (0: never)")
            ->default_val(solver.evict_age);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "tree formulation relaxation, one column per target and set of "
               "options";
    }
    // no bound_description(): its pricing can miss improving columns, so
    // the value of its master bounds nothing
};
