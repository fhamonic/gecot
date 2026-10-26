#pragma once

#include "gecot/solvers/naive_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class NaiveBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::naive_benders solver;

public:
    NaiveBendersInterface() : AbstractMipInterface("naive_benders") {
        add_mip_options(solver);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "Benders decomposition across each landscape graph";
    }
};
