#ifndef GECOT_STATIC_INCREMENTAL_INTERFACE_HPP
#define GECOT_STATIC_INCREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/concepts/instance.hpp"
#include "gecot/solvers/static_incremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

namespace fhamonic {

class StaticIncrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::StaticIncremental solver;
    boost::program_options::options_description desc;

public:
    StaticIncrementalInterface() : desc(name() + " options") {
        desc.add_options()(
            "feasibility-tolerance,t", po::value<double>(&solver.feasibility_tol)
                ->default_value(1e-7), "Tolearnce for rounding errors");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "static_incr"; }
    std::string description() const {
        return "From the base landscape, add the options with the best "
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

#endif  // GECOT_STATIC_INCREMENTAL_INTERFACE_HPP