#pragma once

#include "gecot/solvers/flow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class FlowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::flow_benders solver;

public:
    FlowBendersInterface() : AbstractMipInterface("flow_benders") {
        add_benders_options(solver, column_solvers);
        add_preprocessing_options(solver);
        add_big_M_option(solver.big_Ms);
        app.add_flag("--indicator-constraints", solver.indicator_constraints,
                     "Experimental: a target quality gain is closed by an "
                     "indicator constraint on its option, in place of its "
                     "big-M constraint; the cuts keep their big-Ms (gurobi "
                     "and cplex)");
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "Benders decomposition across each generalized flow with non "
               "linearity of vertex improvements handled by the master "
               "problem";
    }
    std::optional<std::string> bound_description() const override {
        return "Dual bound of the flow Benders master; needs an incumbent.";
    }
};
