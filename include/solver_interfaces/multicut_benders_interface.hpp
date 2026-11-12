#pragma once

#include "gecot/solvers/multicut_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class MulticutBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::multicut_benders solver;

public:
    MulticutBendersInterface() : AbstractMipInterface("multicut_benders") {
        add_mip_options(solver, 1e-7, lazy_constraint_solvers);
        add_preprocessing_options(solver);
        app.add_option("--groups", solver.num_groups,
                       "Groups of targets per graph, each with its own cuts "
                       "(1: a single cut per graph; 0: a group per target)")
            ->default_val(2);
        app.add_option_function<std::string>(
               "--grouping",
               [this](const std::string & s) {
                   using grouping = gecot::solvers::multicut_benders::grouping;
                   solver.groups = s == "voronoi"       ? grouping::voronoi
                                   : s == "consecutive" ? grouping::consecutive
                                                        : grouping::random;
               },
               "voronoi: the cells of a network Voronoi diagram around "
               "targets of value far from each other; consecutive: groups of "
               "consecutive targets; random: groups of targets drawn at "
               "random, as a control")
            ->check(CLI::IsMember({"voronoi", "consecutive", "random"}))
            ->default_val("voronoi");
        app.add_option("--grouping-seed", solver.random_seed,
                       "Seed of --grouping random")
            ->default_val(0);
        app.add_option_function<std::string>(
               "--big-m",
               [this](const std::string & s) {
                   solver.tight_big_M = s == "tight";
               },
               "Experimental: the big-Ms of the knapsack of quality gains, or "
               "per arc over the sources that can use it")
            ->check(CLI::IsMember({"knapsack", "tight"}))
            ->default_val("knapsack");
    }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override;

    std::string description() const override {
        return "Benders decomposition with a cut per group of targets, "
               "from a single cut per graph to a cut per target";
    }
};
