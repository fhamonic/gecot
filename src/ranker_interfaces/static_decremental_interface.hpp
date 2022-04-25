#ifndef LSCP_STATIC_DECREMENTAL_INTERFACE_HPP
#define LSCP_STATIC_DECREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

<<<<<<< HEAD
#include "landscape_opt/solvers/static_decremental.hpp"

#include "instance.hpp"
#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class StaticDecrementalInterface : public AbstractSolver {
private:
    landscape_opt::solvers::StaticDecremental solver;
=======
#include "landscape_opt/rankers/static_decremental.hpp"

#include "instance.hpp"
#include "ranker_interfaces/abstract_ranker.hpp"

namespace fhamonic {

class StaticDecrementalInterface : public AbstractRanker {
private:
    landscape_opt::rankers::StaticDecremental ranker;
>>>>>>> a05a4a85bcd5a46a71a30f4b5faa4258309b6a2e
    boost::program_options::options_description desc;

public:
    StaticDecrementalInterface() : desc(name() + " options") {
        desc.add_options()("verbose,v", "Log the algorithm steps")(
<<<<<<< HEAD
            "parallel,p", "Use multithreaded version")(
            "only-dec",
            "Do not perform the final incremental steps that ensure that the "
            "entire budget is used");
=======
            "parallel,p", "Use multithreaded version");
>>>>>>> a05a4a85bcd5a46a71a30f4b5faa4258309b6a2e
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

<<<<<<< HEAD
        solver.verbose = vm.count("verbose") > 0;
        solver.parallel = vm.count("parallel") > 0;
        solver.only_dec = vm.count("only-dec") > 0;
    }

    typename Instance::Solution solve(const Instance & instance,
                                      const double B) const {
        return solver.solve(instance, B);
=======
        ranker.verbose = vm.count("verbose") > 0;
        ranker.parallel = vm.count("parallel") > 0;
    }

    typename Instance::OptionPotentialMap rank_options(
        const Instance & instance) const {
        return ranker.rank_options(instance);
>>>>>>> a05a4a85bcd5a46a71a30f4b5faa4258309b6a2e
    };

    std::string name() const { return "static_decremental"; }
    std::string description() const {
        return "From the improved landscape, remove the options with the worst "
               "gain/cost ratio.";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "static_decremental"; }
};

}  // namespace fhamonic

#endif  // LSCP_STATIC_DECREMENTAL_INTERFACE_HPP