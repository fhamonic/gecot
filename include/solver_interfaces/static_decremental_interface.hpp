#ifndef GECOT_STATIC_DECREMENTAL_INTERFACE_HPP
#define GECOT_STATIC_DECREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/static_decremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

namespace fhamonic {

class StaticDecrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::StaticDecremental solver;
    boost::program_options::options_description desc;

public:
    StaticDecrementalInterface() : desc(name() + " options") {
        desc.add_options()(
            "feasability-tolerance,t", po::value<double>(&solver.feasability_tol)
                ->default_value(1e-7), "Tolearnce for rounding errors")(
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

        solver.only_dec = vm.count("only-dec") > 0;
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "static_decr"; }
    std::string description() const {
        return "From the improved landscape, remove the options with the worst "
               "gain/cost ratio.";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic

#endif  // GECOT_STATIC_DECREMENTAL_INTERFACE_HPP