#pragma once

#include "gecot/solvers/path_formulation_lp.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class PathFormulationLPInterface : public AbstractMipInterface {
private:
    gecot::solvers::path_formulation_lp solver;

public:
    PathFormulationLPInterface() : AbstractMipInterface("path_lp") {
        add_mip_options(solver, 1e-7);
        app.add_option("--evict-age", solver.evict_age,
                       "Pricing rounds a column stays in the master before it "
                       "can go back to the pool, if unattractive (0: never)")
            ->default_val(0);
        app.add_option_function<std::string>(
               "--pricing",
               [this](const std::string & p) {
                   solver.search_pricing = p == "search";
               },
               "search: a search per pair and round, without the budget; "
               "enumerate: each pair's columns once, within the budget, for "
               "few options per path")
            ->check(CLI::IsMember({"search", "enumerate"}))
            ->default_val("search");
        app.add_option("--max-activations", solver.max_activations,
                       "Columns entering the master per round, the best ones "
                       "(0: every improving column)")
            ->default_val(0);
        app.add_option("--relative-gap", solver.relative_gap,
                       "Stops once the Lagrangian bound is within this gap of "
                       "the relaxation's value, relative to it (0: at "
                       "convergence only)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(0);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;
    // solve, with path_formulation_lp's bound per option
    gecot::solvers::path_formulation_lp::relaxation<Instance> relax(
        const Instance & instance, const gecot::budget_t & B) const;

    std::string description() const override {
        return "path formulation relaxation, one column per pair of vertices "
               "and set of options";
    }
};
