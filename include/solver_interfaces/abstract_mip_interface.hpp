#pragma once

#include <sstream>

#include <boost/program_options.hpp>

#include "solver_interfaces/abstract_solver_interface.hpp"

namespace fhamonic {

class AbstractMipInterface : public AbstractSolverInterface {
protected:
    boost::program_options::options_description desc;
    double feasibility_tolerance;

public:
    AbstractMipInterface(const std::string & name) : desc(name + " options") {
        desc.add_options()(
            "feasibility-tolerance,t",
            po::value<double>(&feasibility_tolerance)->default_value(1e-7),
            "Tolearnce for rounding errors")("print-model,m",
                                             "Print the mip model");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

        handle_options(vm);
    }

    virtual void handle_options(
        const boost::program_options::variables_map & vm) = 0;

    virtual gecot::instance_solution_t<Instance> solve(
        const Instance & instance, const double B) const = 0;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    virtual std::string string() const = 0;
};

}  // namespace fhamonic
