#pragma once

#include "gecot/solvers/unprep_target_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class UnprepTargetBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::unprep_target_benders solver;

public:
    UnprepTargetBendersInterface()
        : AbstractMipInterface("unprep_target_benders") {
        add_mip_options(solver);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "Benders decomposition across each vertex without "
               "preprocessing (poorer big-M bounds).";
    }
};
