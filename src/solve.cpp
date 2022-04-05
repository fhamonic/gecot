#include <filesystem>
#include <iostream>
#include <optional>

#include <boost/range/algorithm.hpp>
namespace br = boost::range;

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace postyle = boost::program_options::command_line_style;

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
namespace logging = boost::log;

// #include "landscape_opt/concepts/instance.hpp"
// #include "landscape_opt/concepts/landscape.hpp"

// #include "landscape_opt/indices/eca.hpp"
// #include "landscape_opt/indices/parallel_eca.hpp"

#include "landscape_opt/solvers/random_solution.hpp"
#include "landscape_opt/solvers/static_decremental.hpp"
#include "landscape_opt/solvers/static_incremental.hpp"
// #include "landscape_opt/solvers/incremental_greedy.hpp"
// #include "landscape_opt/solvers/decremental_greedy.hpp"
// #include "landscape_opt/solvers/mip_xue.hpp"
// #include "landscape_opt/solvers/mip_eca.hpp"
// #include "landscape_opt/solvers/mip_eca_preprocessed.hpp"
// #include "landscape_opt/solvers/randomized_rounding.hpp"

#include "po_utils.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

#include "solver_interfaces/abstract_solver.hpp"
#include "solver_interfaces/static_decremental_interface.hpp"
using namespace fhamonic;

// #include "landscape_opt/parsers/parse_instance.hpp"

void init_logging() {
    logging::core::get()->set_filter(logging::trivial::severity >=
                                     logging::trivial::warning);
}

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolver> & solver,
    std::filesystem::path & instances_description_json_file, double & budget,
    bool & output_in_file, std::filesystem::path & output_csv_file) {
    std::vector<std::shared_ptr<AbstractSolver>> solver_interfaces{
        std::make_unique<StaticDecrementalInterface>()};

    auto print_soft_name = []() { std::cout << "LSCP 0.1\n\n"; };
    auto print_usage = []() {
        std::cout << R"(Usage:
  lcsp_solve --help
  lcsp_solve --list-algorithms
  lcsp_solve <algorithm> --list-params
  lcsp_solve <algorithm> <instance> <budget> [<params> ...]

)";
    };

    auto print_available_algorithms = [&solver_interfaces]() {
        std::cout << "Available algorithms:\n";
        for(auto & s : solver_interfaces)
            std::cout << "  " << s->name() << '\n';
        std::cout << std::endl;
        return false;
    };

    std::string solver_name;

    try {
        po::options_description desc("Allowed options");
        desc.add_options()("help,h", "Display this help message")(
            "list-algorithms", "List the available algorithms")(
            "list-params", "List the parameters of the chosen algorithm")(
            "algorithm,a", po::value<std::string>(&solver_name)->required(),
            "Algorithm to use")(
            "instance,i",
            po::value<std::filesystem::path>(&instances_description_json_file)
                ->required(),
            "Instance json file")(
            "budget,B", po::value<double>(&budget)->required(), "Budget value")(
            "output,o", po::value<std::filesystem::path>(&output_csv_file),
            "Solution output file");

        po::positional_options_description p;
        p.add("algorithm", 1);
        p.add("instance", 1);
        p.add("budget", 1);
        po::variables_map vm;
        po::parsed_options parsed = po::basic_command_line_parser(args)
                                        .options(desc)
                                        .positional(p)
                                        .allow_unregistered()
                                        .run();
        po::store(parsed, vm);

        if(vm.count("help")) {
            print_soft_name();
            print_usage();

            std::cout << desc << std::endl;
            return false;
        }

        if(vm.count("list-algorithms")) {
            print_soft_name();
            print_available_algorithms();
            return false;
        }

        if(vm.count("algorithm")) {
            solver_name = vm["algorithm"].as<std::string>();
            bool found = false;
            for(auto & s : solver_interfaces) {
                if(s->name() == solver_name) {
                    solver = std::move(s);
                    found = true;
                    break;
                }
            }
            if(!found) {
                std::cout << "Error: the argument ('" << solver_name
                          << "') for option '--algorithm' is invalid\n\n";
                print_available_algorithms();
                return false;
            }
        }

        if(vm.count("list-params")) {
            if(!vm.count("algorithm")) {
                throw std::logic_error(std::string(
                    "Option '--list-params' requires option '--algorithm'."));
            }
            print_soft_name();
            std::cout << solver->description();
            std::cout << std::endl;
            return false;
        }

        po::notify(vm);
        std::vector<std::string> opts =
            po::collect_unrecognized(parsed.options, po::exclude_positional);

        solver->parse(opts);
        output_in_file = vm.count("output");
    } catch(std::exception & e) {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    return true;
}

int main(int argc, const char * argv[]) {
    (void)argc;
    (void)argv;

    std::shared_ptr<AbstractSolver> solver;
    std::filesystem::path instances_description_json;
    double budget;
    bool output_in_file;
    std::filesystem::path output_csv;

    std::vector<std::string> args(argv + 1, argv + argc);
    bool valid_command =
        process_command_line(args, solver, instances_description_json, budget,
                             output_in_file, output_csv);
    if(!valid_command) return EXIT_FAILURE;
    init_logging();

    Instance instance = parse_instance(instances_description_json);

    Instance::Solution solution = solver->solve(instance, budget);

    // Helper::printSolution(instance.landscape, instance.plan, name, solver,
    // instance.budget, solution);

    return EXIT_SUCCESS;
}
