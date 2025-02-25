#ifndef GECOT_MIP_INTERFACE_HPP
#define GECOT_MIP_INTERFACE_HPP

#include <filesystem>
#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/mip.hpp"
#include "gecot/utils/mip_helper.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

namespace fhamonic {

class MIPInterface : public AbstractMIPInterface {
private:
    gecot::solvers::MIP solver;

public:
    MIPInterface() : AbstractMIPInterface(name()) {}

    void handle_options(const boost::program_options::variables_map & vm) {
        solver.parallel = vm.count("parallel") > 0;
        solver.feasability_tol = vm.at("feasability-tolerance").as<double>();
        solver.print_model = vm.count("print-model") > 0;
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "mip"; }
    std::string description() const {
        return "MIP formulation without preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic

#endif  // GECOT_MIP_INTERFACE_HPP
