#pragma once

#include "gecot/solvers/multicut_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MulticutBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::multicut_benders solver;

public:
    MulticutBendersInterface() : AbstractMipInterface("multicut_benders") {
        add_mip_options(solver, lazy_constraint_solvers);
        add_preprocessing_options(solver);
        app.add_option("--groups", solver.num_groups,
                       "Groups of targets per graph, each with its own cuts "
                       "(1: a single cut per graph; 0: a group per target)")
            ->default_val(solver.num_groups);
        using grouping = gecot::solvers::multicut_benders::grouping;
        add_choice_option<grouping>(
            app, "--grouping", solver.groups,
            {{"voronoi", grouping::voronoi},
             {"consecutive", grouping::consecutive},
             {"random", grouping::random}},
            [this](const grouping groups) { solver.groups = groups; },
            "voronoi: the cells of a network Voronoi diagram around "
            "targets of value far from each other; consecutive: groups of "
            "consecutive targets; random: groups of targets drawn at "
            "random, as a control");
        app.add_option("--grouping-seed", solver.random_seed,
                       "Seed of --grouping random")
            ->default_val(solver.random_seed);
        add_big_M_option(solver.big_Ms);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "Benders decomposition with a cut per group of targets, "
               "from a single cut per graph to a cut per target";
    }
    std::optional<std::string> bound_description() const override {
        return "Dual bound of the multicut Benders master; needs an incumbent.";
    }
};
