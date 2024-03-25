#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
namespace logging = boost::log;

// #include "gecot/solvers/mip_xue.hpp"
// #include "gecot/solvers/randomized_rounding.hpp"

#include "gecot/helper.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

#include "solver_interfaces/abstract_solver.hpp"
#include "solver_interfaces/greedy_decremental_interface.hpp"
#include "solver_interfaces/greedy_incremental_interface.hpp"
#include "solver_interfaces/mip_interface.hpp"
#include "solver_interfaces/preprocessed_mip_interface.hpp"
#include "solver_interfaces/static_decremental_interface.hpp"
#include "solver_interfaces/static_incremental_interface.hpp"

using namespace fhamonic;

void init_logging() {
    logging::core::get()->set_filter(logging::trivial::severity >=
                                     logging::trivial::warning);
}

void print_paragraph(std::ostream & os, std::size_t offset,
                     std::size_t column_width, const std::string & str) {
    std::size_t line_start = 0;
    while(str.size() - line_start > column_width) {
        std::size_t n = str.rfind(' ', line_start + column_width);
        if(n <= line_start) {
            os << str.substr(line_start, column_width - 1) << '-' << '\n'
               << std::string(offset, ' ');
            line_start += column_width - 1;
        } else {
            os << str.substr(line_start, n - line_start) << '\n'
               << std::string(offset, ' ');
            line_start = n + 1;
        }
    }
    os << str.substr(line_start) << '\n';
}

#include "mippp/solver_traits/all.hpp" 


static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolver> & solver,
    std::filesystem::path & instances_description_json_file, double & budget,
    std::optional<std::filesystem::path> & opt_output_csv_file) {
    std::vector<std::shared_ptr<AbstractSolver>> solver_interfaces{
        std::make_unique<StaticIncrementalInterface>(),
        std::make_unique<StaticDecrementalInterface>(),
        std::make_unique<GreedyIncrementalInterface>(),
        std::make_unique<GreedyDecrementalInterface>(),
        std::make_unique<MIPInterface>(),
        std::make_unique<PreprocessedMIPInterface>()};

    auto print_soft_name = []() {
        std::cout << "GECOT — Graph-based Ecological Connectivity "
                     "Optimization Tool\nVersion: "
                  << PROJECT_VERSION << " (built on " << __DATE__ << ")\n\n";
    };
    auto print_usage = []() {
        std::cout << R"(Usage:
  gecot_solve --help
  gecot_solve --list-algorithms
  gecot_solve <algorithm> --list-params
  gecot_solve <algorithm> <instance> <budget> [<params> ...]

)";
    };

    auto print_available_algorithms = [&solver_interfaces]() {
        std::cout << "Available algorithms:\n";
        const std::size_t algorithm_name_max_length = std::ranges::max(
            std::ranges::views::transform(solver_interfaces, [&](auto && s) {
                return s->name().size();
            }));
        const std::size_t offset = algorithm_name_max_length + 8;

        for(auto & s : solver_interfaces) {
            std::cout << "  " << s->name()
                      << std::string(offset - s->name().size() - 2, ' ');
            print_paragraph(std::cout, offset, 80 - offset, s->description());
        }
        std::cout << std::endl;
        return false;
    };

    // path istead of string to silence a warning...
    std::filesystem::path output_csv_file;

    try {
        po::options_description desc("Allowed options");
        desc.add_options()("help,h", "Display this help message")(
            "list-algorithms,A", "List the available algorithms")(
            "list-params,P", "List the parameters of the chosen algorithm")(
            "algorithm,a", po::value<std::string>()->required(),
            "Algorithm to use")(
            "instance,i",
            po::value<std::filesystem::path>(&instances_description_json_file)
                ->required(),
            "Instance JSON file")(
            "budget,B", po::value<double>(&budget)->required(), "Budget value")(
            "output-csv,o", po::value<std::filesystem::path>(&output_csv_file),
            "Output solution in CSV file");

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

        if(vm.count("help") || args.empty()) {
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
            auto solver_name = vm["algorithm"].as<std::string>();
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
            std::cout << solver->options_description();
            std::cout << std::endl;
            return false;
        }

        po::notify(vm);
        if(vm.count("output-csv")) {
            opt_output_csv_file.emplace(std::move(output_csv_file));
        }
        std::vector<std::string> opts =
            po::collect_unrecognized(parsed.options, po::exclude_positional);
        solver->parse(opts);
    } catch(std::exception & e) {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    return true;
}

int main(int argc, const char * argv[]) {
#ifdef WIN32
    std::system("chcp 65001 > NUL");
#endif
    std::cout.precision(20);
    
    std::shared_ptr<AbstractSolver> solver;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_csv_file;

    std::vector<std::string> args(argv + 1, argv + argc);
    bool valid_command =
        process_command_line(args, solver, instances_description_json, budget,
                             opt_output_csv_file);
    if(!valid_command) return EXIT_FAILURE;
    init_logging();

    Instance raw_instance = parse_instance(instances_description_json);
    Instance instance = trivial_reformulate_instance(raw_instance, budget);
    // Instance & instance = raw_instance; //30.52972718

    chronometer chrono;
    auto solution = solver->solve(instance, budget);
    const auto time_ms = chrono.time_ms();

    const double solution_score =
        gecot::compute_solution_score(instance, solution);
    std::cout << "Score: " << solution_score << std::endl;
    std::cout << "Cost: "
                << fhamonic::gecot::compute_solution_cost(instance, solution)
                << std::endl;
    std::cout << "Solution:" << std::endl;
    const std::size_t option_name_max_length =
        std::ranges::max(std::ranges::views::transform(
            raw_instance.options(),
            [&](auto && o) { return raw_instance.option_name(o).size(); }));
    for(auto && option : raw_instance.options()) {
        auto option_name = raw_instance.option_name(option);
        bool value = instance.contains_option(option_name)
                            ? solution[instance.option_from_name(option_name)]
                            : false;
        std::cout << "  " << option_name
                    << std::string(
                            option_name_max_length + 1 -
                                raw_instance.option_name(option).size(),
                            ' ')
                    << value << '\n';
    }
    std::cout << std::endl << "in " << time_ms << " ms" << std::endl;
    
    if(opt_output_csv_file.has_value()){
        std::ofstream output_file(opt_output_csv_file.value());
        if(output_file.is_open()) {
            output_file << "option_id,value\n";
            for(auto && option : raw_instance.options()) {
                auto option_name = raw_instance.option_name(option);
                bool value =
                    instance.contains_option(option_name)
                        ? solution[instance.option_from_name(option_name)]
                        : false;
                output_file << option_name << ',' << value << '\n';
            }
        } else
            std::cerr << "ERR: '" << opt_output_csv_file.value() << "' not opened" << std::endl;
    }

    return EXIT_SUCCESS;
}
