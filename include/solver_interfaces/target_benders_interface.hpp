#pragma once

#include "gecot/solvers/target_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class TargetBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::target_benders solver;

public:
    TargetBendersInterface() : AbstractMipInterface("target_benders") {
        add_mip_options(solver, 1e-7, lazy_constraint_solvers);
        add_preprocessing_options(solver);
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
        return "Benders decomposition across each vertex";
    }
};
