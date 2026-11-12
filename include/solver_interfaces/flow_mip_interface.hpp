#pragma once

#include "gecot/solvers/flow_mip.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class FlowMipInterface : public AbstractMipInterface {
private:
    gecot::solvers::flow_mip solver;

public:
    FlowMipInterface() : AbstractMipInterface("flow_mip") {
        add_mip_options(solver);
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
        app.add_flag("--lp-relaxation", solver.relax_integrality,
                     "Experimental: solve the continuous relaxation");
    }

    gecot::solve_result<Instance> solve(const Instance & instance,
                                        const double B) const override;

    std::string description() const override {
        return "MIP with a generalized flow per target, preprocessed, from "
               "'Optimizing the ecological connectivity of landscapes', "
               "F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
};
