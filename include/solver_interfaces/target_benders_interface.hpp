#pragma once

#include "gecot/solvers/target_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class TargetBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::target_benders solver;

public:
    TargetBendersInterface() : AbstractMipInterface("target_benders") {
        add_mip_options(solver);
    }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "Benders decomposition across each vertex";
    }
};
