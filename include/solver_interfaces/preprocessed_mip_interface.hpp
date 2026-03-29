#pragma once

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/preprocessed_mip.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

namespace fhamonic {

class PreprocessedmipInterface : public AbstractMipInterface {
private:
    gecot::solvers::preprocessed_mip solver;

public:
    PreprocessedmipInterface() : AbstractMipInterface(name()) {
        desc.add_options()(
            "resolution",
            po::value<double>(&solver.probability_resolution)
                ->default_value(1e-8),
            "Sets the resolution of the probabilities used by the "
            "preprocessing algorithm")(
            "nb-mus", po::value<int>(&solver.num_mus)->default_value(0),
            "Sets the number of lagragrian multipliers to test for the "
            "constrained preprocessing algorithm (experimental)");
    }

    void handle_options(const boost::program_options::variables_map & vm) {
        solver.feasibility_tol = vm["feasibility-tolerance"].as<double>();
        solver.print_model = vm.count("print-model") > 0;
        if(vm.count("mip-start"))
            solver.mip_start =
                parse_mip_start(vm["mip-start"].as<std::filesystem::path>());
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "prep_mip"; }
    std::string description() const {
        return "mip formulation with preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic
