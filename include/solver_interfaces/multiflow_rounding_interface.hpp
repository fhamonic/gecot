#pragma once

#include "gecot/solvers/multiflow_rounding.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MultiflowRoundingInterface : public AbstractMipInterface {
private:
    gecot::solvers::multiflow_rounding solver;

public:
    MultiflowRoundingInterface() : AbstractMipInterface("multiflow_rounding") {
        add_mip_options(solver, column_solvers);
        add_preprocessing_options(solver);
        add_big_M_option(solver.big_Ms);
        add_root_cut_options(solver);
        add_stored_arcs_option(solver);
        add_source_group_options(solver);
        app.add_option("--roundings", solver.roundings,
                       "Scenarios drawn from the optimum of the relaxation, "
                       "the first by decreasing x (0: until --time-limit, "
                       "which is then required)")
            ->default_val(solver.roundings);
        app.add_option("--rounding-seed", solver.rounding_seed,
                       "Seed of the draws")
            ->default_val(solver.rounding_seed);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "relaxation of multiflow_benders with its cuts, then "
               "randomized rounding of its optimum";
    }
    std::optional<std::string> bound_description() const override {
        return "Value of the relaxation of multiflow_benders with the cuts of "
               "its rounds.";
    }
};
