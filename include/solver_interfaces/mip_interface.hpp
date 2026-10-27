#pragma once

#include "gecot/solvers/mip.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class mipInterface : public AbstractMipInterface {
private:
    gecot::solvers::mip solver;

public:
    mipInterface() : AbstractMipInterface("mip") { add_mip_options(solver); }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override;

    std::string description() const override {
        return "mip formulation without preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
};
