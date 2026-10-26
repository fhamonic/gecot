#pragma once

#include "gecot/solvers/flow_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

class FlowBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::flow_benders solver;

public:
    FlowBendersInterface() : AbstractMipInterface("flow_benders") {
        add_mip_options(solver);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const override {
        return solver.solve(instance, B);
    };

    std::string description() const override {
        return "Benders decomposition across each generalized flow with non "
               "linearity of vertex improvements handled by the master "
               "problem";
    }
};
