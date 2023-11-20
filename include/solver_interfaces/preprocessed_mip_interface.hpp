#ifndef GECOT_PREPROCESSED_MIP_INTERFACE_HPP
#define GECOT_PREPROCESSED_MIP_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/preprocessed_mip.hpp"

#include "instance.hpp"
#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class PreprocessedMIPInterface : public AbstractSolver {
private:
    gecot::solvers::preprocessed_MIP solver;
    boost::program_options::options_description desc;

public:
    PreprocessedMIPInterface() : desc(name() + " options") {
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

    gecot::instance_solution_t<Instance> solve(
        const Instance & instance, const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "prep_mip"; }
    std::string description() const {
        return "MIP formulation with preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', François Hamonic, "
               "Cécile Albert, Basile Couëtoux, Yann Vaxès";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic

#endif  // GECOT_PREPROCESSED_MIP_INTERFACE_HPP