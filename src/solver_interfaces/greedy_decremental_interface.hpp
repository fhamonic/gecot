#ifndef LSCP_GREEDY_DECREMENTAL_INTERFACE_HPP
#define LSCP_GREEDY_DECREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "landscape_opt/solvers/greedy_decremental.hpp"

#include "instance.hpp"
#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class GreedyDecrementalInterface : public AbstractSolver {
private:
    landscape_opt::solvers::GreedyDecremental solver;
    boost::program_options::options_description desc;

public:
    GreedyDecrementalInterface() : desc(name() + " options") {
        desc.add_options()("verbose,v", "Log the algorithm steps")(
            "parallel,p", "Use multithreaded version")(
            "only-dec",
            "Do not perform the final incremental steps that ensure that the "
            "entire budget is used");
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
        solver.only_dec = vm.count("only-dec") > 0;
    }

    typename Instance::Solution solve(const Instance & instance,
                                      const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "greedy_decremental"; }
    std::string description() const {
        return "From the improved landscape, iteratively remove the option with "
               "the worst gain/cost ratio.";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "greedy_decremental"; }
};

}  // namespace fhamonic

#endif  // LSCP_GREEDY_DECREMENTAL_INTERFACE_HPP