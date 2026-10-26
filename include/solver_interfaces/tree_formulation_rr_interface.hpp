#pragma once

#include "gecot/solvers/tree_formulation_rr.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class TreeFormulationRRInterface : public AbstractMipInterface {
private:
    gecot::solvers::tree_formulation_rr solver;

public:
    TreeFormulationRRInterface() : AbstractMipInterface("tree_rr") {
        add_mip_options(solver);
        add_preprocessing_options(solver);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "tree formulation relaxation with randomized rounding";
    }
};
