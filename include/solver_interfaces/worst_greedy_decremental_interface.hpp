#pragma once

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/worst_greedy_decremental.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

namespace fhamonic {

class WorstGreedyDecrementalInterface : public AbstractSolverInterface {
private:
    gecot::solvers::WorstGreedyDecremental solver;
    boost::program_options::options_description desc;

public:
    WorstGreedyDecrementalInterface() : desc(name() + " options") {
        desc.add_options()(
            "feasibility-tolerance,t",
            po::value<double>(&solver.feasibility_tol)->default_value(1e-7),
            "Tolearnce for rounding errors");
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

    std::string name() const { return "worst_greedy_decr"; }
    std::string description() const {
        return "From the improved landscape, iteratively remove the option "
               "with the best gain/cost ratio (to minimize connectivity).";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic
