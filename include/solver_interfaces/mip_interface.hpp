#pragma once

#include <filesystem>
#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/mip.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

namespace fhamonic {

class mipInterface : public AbstractMipInterface {
private:
    gecot::solvers::mip solver;

public:
    mipInterface() : AbstractMipInterface(name()) {}

    void handle_options(const boost::program_options::variables_map & vm) {
        solver.feasibility_tol = feasibility_tolerance;
        solver.print_model = vm.count("print-model") > 0;
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "mip"; }
    std::string description() const {
        return "mip formulation without preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic
