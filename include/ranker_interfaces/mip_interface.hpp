#ifndef GECOT_MIP_INTERFACE_HPP
#define GECOT_MIP_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/mip.hpp"

#include "instance.hpp"
#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class MIPInterface : public AbstractSolver {
private:
    gecot::solvers::MIP solver;
    boost::program_options::options_description desc;

public:
    MIPInterface() : desc(name() + " options") {
        desc.add_options()("verbose,v", "Log the algorithm steps")(
            "parallel,p", "Use multithreaded version");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

        solver.verbose = vm.count("verbose") > 0;
        solver.parallel = vm.count("parallel") > 0;
    }

    typename Instance::Solution solve(const Instance & instance,
                                      const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "mip"; }
    std::string description() const {
        return "Mixed Integer Program from 'Optimizing the ecological "
               "connectivity of landscapes with generalized flow models and "
               "preprocessing', François Hamonic, Cécile Albert, Basile "
               "Couëtoux, Yann Vaxès";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "mip"; }
};

}  // namespace fhamonic

#endif  // GECOT_MIP_INTERFACE_HPP