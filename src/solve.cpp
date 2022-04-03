#include <filesystem>
#include <iostream>

#include <boost/range/algorithm.hpp>
namespace br = boost::range;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
namespace logging = boost::log;

#include "landscape_opt/concepts/instance.hpp"
#include "landscape_opt/concepts/landscape.hpp"

#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/indices/parallel_eca.hpp"

#include "landscape_opt/solvers/static_decremental.hpp"
#include "landscape_opt/solvers/static_incremental.hpp"
#include "landscape_opt/solvers/random_solution.hpp"
// #include "landscape_opt/solvers/incremental_greedy.hpp"
// #include "landscape_opt/solvers/decremental_greedy.hpp"
// #include "landscape_opt/solvers/mip_xue.hpp"
// #include "landscape_opt/solvers/mip_eca.hpp"
// #include "landscape_opt/solvers/mip_eca_preprocessed.hpp"
// #include "landscape_opt/solvers/randomized_rounding.hpp"

#include "po_utils.hpp"
#include "solver_interfaces/abstract_solver.hpp"
using namespace fhamonic;


// #include "landscape_opt/parsers/parse_instance.hpp"

void init_logging() {
    logging::core::get()->set_filter(logging::trivial::severity >=
                                     logging::trivial::warning);
}

static bool process_command_line(int argc, const char * argv[],
                                 std::unique_ptr<AbstractSolver> &
  solver, std::filesystem::path & instances_description_json_file,
                                 double & budget, bool & output_in_file,
                                 std::filesystem::path & output_csv_file) {
    // std::vector<std::shared_ptr<concepts::Solver>> solvers{
    //     std::make_unique<solvers::Bogo>(),
    //     std::make_unique<solvers::Naive_ECA_Inc>(),
    //     std::make_unique<solvers::Naive_ECA_Dec>(),
    //     std::make_unique<solvers::Glutton_ECA_Inc>(),
    //     std::make_unique<solvers::Glutton_ECA_Dec>(),
    //     std::make_unique<solvers::PL_ECA_2>(),
    //     std::make_unique<solvers::PL_ECA_3>(),
    //     std::make_unique<solvers::Randomized_Rounding_ECA>()};

    std::string solver_name;
    std::vector<std::string> solver_params;

    try {
        po::options_description desc("Allowed options");
        desc.add_options()("help,h", "Display this help message")(
            "algorithm,a", po::value<std::string>(&solver_name)->required(),
            "Algorithm to use")(
            "params,p",
            po::value<std::vector<std::string>>(&solver_params)->multitoken(),
            "Parameters of the chosen algorithm")(
            "list-params",
            "Display the parameters of the chosen algorithm")(
            "instance,i",
            po::value<std::filesystem::path>(&instances_description_json_file)
                ->required(),
            "Instance description file")(
            "budget,B", po::value<double>(&budget)->required(),
            "Budget value")(
            "output,o", po::value<std::filesystem::path>(&output_csv_file),
            "CSV output path");

        po::positional_options_description p;
        p.add("algorithm", 1);
        p.add("instance", 2);
        p.add("budget", 3);
        p.add("params", -1);
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        if(vm.count("help")) {
            std::cout << desc << "\n";
            // std::cout << "Available solvers:" << std::endl;
            // for(auto & s : solvers) std::cout << "\t" << s->name() << std::endl;
            return false;
        }
        // po::notify(vm);
        // auto it = br::find_if(solvers, [&solver_name](auto & s) {
        //     return s->name() == solver_name;
        // });
        // if(it == solvers.end())
        // solver = make_solver(solver_name);
        // if(vm.count("list-params")) {
        //     std::cout << "Available options for " << solver_name << ":"
        //               << std::endl;
        //     for(auto & param : solver->getParams())
        //         std::cout << "\t" << param.first << std::endl;
        //     return false;
        // }
        // for(auto param : solver_params) {
        //     auto [param_name, param_value] =
        //         po_utils::split_equality_str(param);
        //     bool param_exists =
        //         solver->setParam(param_name, param_value.data());
        //     if(!param_exists)
        //         throw std::invalid_argument(
        //             "'" + param + "' is not a valid parameter for " +
        //             solver_name + ", see --list-params.");
        //     if(param_value.empty())
        //         throw std::invalid_argument("Invalid value for parameter '" +
        //                                     param_name + "' of " + solver_name);
        //     solver->setParam(param_name, param_value.data());
        // }
        // output_in_file = vm.count("output");
    } catch(std::exception & e) {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    return true;
}

int main(int argc, const char * argv[]) {
    (void)argc;
    (void)argv;

    // std::shared_ptr<concepts::Solver> solver;
    // std::filesystem::path instances_description_json;
    // double budget;
    // bool output_in_file;
    // std::filesystem::path output_csv;

    bool valid_command =
        process_command_line(argc, argv, solver, instances_description_json,
                             budget, output_in_file, output_csv);
    if(!valid_command) return EXIT_FAILURE;
    init_logging();

    // Instance2 instance = parse_instance2(instances_description_json);

    // Helper::assert_well_formed(instance.landscape, instance.plan);

    // Solution solution = solver.solve(instance.landscape, instance.plan,
    // instance.budget);

    // Helper::printSolution(instance.landscape, instance.plan, name, solver,
    // instance.budget, solution);

    return EXIT_SUCCESS;
}
