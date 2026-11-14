#pragma once

#include "gecot/solvers/path_lp.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class PathLpInterface : public AbstractMipInterface {
private:
    gecot::solvers::path_lp solver;

public:
    PathLpInterface() : AbstractMipInterface("path_lp") {
        add_mip_options(solver);
        app.add_option("--evict-age", solver.evict_age,
                       "Pricing rounds a column stays in the master before it "
                       "can go back to the pool, if unattractive (0: never)")
            ->default_val(solver.evict_age);
        add_choice_option<bool>(
            app, "--pricing", solver.search_pricing,
            {{"search", true}, {"enumerate", false}},
            [this](const bool search) { solver.search_pricing = search; },
            "search: a search per pair and round, without the budget; "
            "enumerate: each pair's columns once, within the budget, for "
            "few options per path");
        app.add_option("--max-activations", solver.max_activations,
                       "Columns entering the master per round, the best ones "
                       "(0: every improving column)")
            ->default_val(solver.max_activations);
        app.add_option("--relative-gap", solver.relative_gap,
                       "Stops once the Lagrangian bound is within this gap of "
                       "the relaxation's value, relative to it (0: at "
                       "convergence only)")
            ->check(CLI::NonNegativeNumber)
            ->default_val(solver.relative_gap);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;
    // solve, with path_lp's bound per option
    gecot::solvers::path_lp::relaxation<Instance> relax(
        const Instance & instance, const gecot::budget_t & B) const;

    std::string description() const override {
        return "path formulation relaxation, one column per pair of vertices "
               "and set of options";
    }
    std::optional<std::string> bound_description() const override {
        return "Lagrangian bound of the path formulation relaxation, the "
               "lowest over its complete pricing rounds.";
    }
};
