#pragma once

#include "gecot/solvers/flow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class FlowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::flow_benders solver;

public:
    FlowBendersInterface() : AbstractMipInterface("flow_benders") {
        add_mip_options(solver, 1e-7, lazy_constraint_solvers);
        add_preprocessing_options(solver);
        add_big_M_option(solver.big_Ms);
    }

    gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const override;

    std::string description() const override {
        return "Benders decomposition across each generalized flow with non "
               "linearity of vertex improvements handled by the master "
               "problem";
    }
};
