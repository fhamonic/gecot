#ifndef GECOT_ABSTRACT_MIP_INTERFACE_HPP
#define GECOT_ABSTRACT_MIP_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/utils/mip_helper.hpp"

#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class AbstractMIPInterface : public AbstractSolver {
protected:
    boost::program_options::options_description desc;
    std::filesystem::path cbc_default_path;

public:
    AbstractMIPInterface(const std::string & name)
        : desc(name + " options")
        , cbc_default_path(
#ifdef WIN32
              get_exec_path().parent_path() /
#endif
              "cbc") {
        desc.add_options()("parallel,p", "Use multithreaded version")(
            "print-model,m", "Print the MIP model")(
            "use-cbc",
            "Prioritizes cbc for solving MIPs\n(default if gurobi_cl and scip "
            "are not found)")("use-grb",
                              "Prioritizes cbc for soplving MIPs\n(default "
                              "if gurobi_cl is found)")(
            "use-scip",
            "Prioritizes scip for soplving MIPs\n(warning: scip is for "
            "academic "
            "use only)")(
            "cbc-path",
            po::value<std::filesystem::path>(&mippp::cli_cbc_solver::exec_path),
            (std::string("Sets path to cbc executable\n(default: '") +
             cbc_default_path.string() + "')")
                .c_str())(
            "grb-path",
            po::value<std::filesystem::path>(&mippp::cli_grb_solver::exec_path),
            "Sets path to gurobi_cl executable")(
            "scip-path",
            po::value<std::filesystem::path>(
                &mippp::cli_scip_solver::exec_path),
            "Sets path to scip executable");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

        if(vm.count("use-cbc") + vm.count("use-gurobi") + vm.count("use-scip") >
           1) {
            throw std::logic_error(
                "Chose only one solver between 'use-cbc', 'use-gurobi' and "
                "'use-scip'");
        }
        if(!vm.count("cbc-path"))
            mippp::cli_cbc_solver::exec_path = cbc_default_path;
        if(vm.count("use-cbc"))
            gecot::mip_helper::prefered_solver =
                gecot::mip_helper::solvers::cbc;
        if(vm.count("use-gurobi"))
            gecot::mip_helper::prefered_solver =
                gecot::mip_helper::solvers::grb;
        if(vm.count("use-scip"))
            gecot::mip_helper::prefered_solver =
                gecot::mip_helper::solvers::scip;

        handle_options(vm);
    }

    virtual void handle_options(const boost::program_options::variables_map & vm) = 0;

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

#endif  // GECOT_ABSTRACT_MIP_INTERFACE_HPP
