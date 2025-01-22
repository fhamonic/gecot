#ifndef GECOT_MIP_INTERFACE_HPP
#define GECOT_MIP_INTERFACE_HPP

#include <filesystem>
#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/solvers/mip.hpp"
#include "gecot/utils/mip_helper.hpp"

#include "solver_interfaces/abstract_solver.hpp"

namespace fhamonic {

class MIPInterface : public AbstractSolver {
private:
    gecot::solvers::MIP solver;
    boost::program_options::options_description desc;
    std::filesystem::path cbc_default_path;

public:
    MIPInterface()
        : desc(name() + " options")
        ,  cbc_default_path(
#ifdef WIN32
get_exec_path().parent_path() /
#endif
"cbc") {
        desc.add_options()("parallel,p", "Use multithreaded version")(
            "print-model,m", "Print the MIP model")(
            "use-cbc",
            "Prioritizes cbc for soplving MIPs\n(default if gurobi_cl and scip "
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

        solver.parallel = vm.count("parallel") > 0;
        solver.print_model = vm.count("print-model") > 0;
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
    }

    gecot::instance_solution_t<Instance> solve(const Instance & instance,
                                               const double B) const {
        return solver.solve(instance, B);
    };

    std::string name() const { return "mip"; }
    std::string description() const {
        return "MIP formulation without preprocessing, from 'Optimizing the "
               "ecological connectivity of landscapes', F.\u00A0Hamonic, "
               "C.\u00A0H.\u00A0Albert, B.\u00A0Couëtoux, Y.\u00A0Vaxès";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return name(); }
};

}  // namespace fhamonic

#endif  // GECOT_MIP_INTERFACE_HPP
