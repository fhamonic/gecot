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
        add_big_M_option(solver.big_Ms);
        app.add_flag("--lp-relaxation", solver.relax_integrality,
                     "Experimental: solve the continuous relaxation");
        app.add_flag("--indicator-constraints", solver.indicator_constraints,
                     "Experimental: an improved copy and a target quality "
                     "gain are closed by an indicator constraint on their "
                     "option, in place of a big-M constraint: --big-m is then "
                     "not read (gurobi and cplex)");
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "MIP with a generalized flow per target, preprocessed, from "
               "'Optimizing the ecological connectivity of landscapes', "
               "F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
    std::optional<std::string> bound_description() const override {
        return "Dual bound of the flow MIP, exact up to --resolution; needs an "
               "incumbent.";
    }
};
