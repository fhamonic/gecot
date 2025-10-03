#pragma once

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/naive_benders.hpp"

#include "solver_interfaces/abstract_mip_interface.hpp"

namespace fhamonic {

class NaiveBendersInterface : public AbstractMipInterface {
private:
    gecot::solvers::naive_benders solver;

public:
    NaiveBendersInterface() : AbstractMipInterface(name()) {
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
        solver.feasibility_tol = feasibility_tolerance;
        solver.print_model = vm.count("print-model") > 0;
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "naive_benders"; }
    std::string description() const {
        return "Benders decomposition accross each landscape graph";
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic
