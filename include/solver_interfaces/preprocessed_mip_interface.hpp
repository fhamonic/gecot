#pragma once

#include "gecot/solvers/preprocessed_mip.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class PreprocessedmipInterface : public AbstractMipInterface {
private:
    gecot::solvers::preprocessed_mip solver;

public:
    PreprocessedmipInterface() : AbstractMipInterface("prep_mip") {
        add_mip_options(solver);
        add_preprocessing_options(solver);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "mip formulation with preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
};
