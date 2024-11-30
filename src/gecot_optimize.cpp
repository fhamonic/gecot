#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <fmt/core.h>
#include <fmt/os.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "gecot/helper.hpp"

#include "io_helper.hpp"
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

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolver> & solver,
    std::filesystem::path & instances_description_json_file, double & budget,
    std::optional<std::filesystem::path> & opt_output_json_file,
    std::optional<std::filesystem::path> & opt_output_csv_file) {
    std::vector<std::shared_ptr<AbstractSolver>> solver_interfaces{
        std::make_unique<StaticIncrementalInterface>(),
        std::make_unique<StaticDecrementalInterface>(),
        std::make_unique<GreedyIncrementalInterface>(),
        std::make_unique<GreedyDecrementalInterface>(),
        std::make_unique<MIPInterface>(),
        std::make_unique<PreprocessedMIPInterface>()};

    auto print_soft_name = []() {
        fmt::print(
            "GECOT — Graph-based Ecological Connectivity "
            "Optimization Tool\nVersion: {} (built on {})\n\n",
            PROJECT_VERSION, __DATE__);
    };
    auto print_usage = []() {
        fmt::print(R"(Usage:
  gecot_optimize --help
  gecot_optimize --list-algorithms
  gecot_optimize <algorithm> --list-params
  gecot_optimize <algorithm> <instance> <budget> [<params> ...]

)");
    };

    auto print_available_algorithms = [&solver_interfaces]() {
        fmt::println("Available algorithms:");
        const std::size_t algorithm_name_max_length = std::ranges::max(
            std::ranges::views::transform(solver_interfaces, [&](auto && s) {
                return s->name().size();
            }));
        const std::size_t offset = algorithm_name_max_length + 8;

        for(auto & s : solver_interfaces) {
            fmt::print("  {:<{}}", s->name(), offset - 2);
            print_paragraph(offset, 80 - offset, s->description());
        }
        fmt::println("");
        return false;
    };

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
        "output-json,o", po::value<std::filesystem::path>(),
        "Output solution data in JSON file")(
        "output-csv", po::value<std::filesystem::path>(),
        "Output solution value in CSV file")("verbose,v",
                                             "Enable all logging informations")(
        "quiet,q", "Silence all logging except errors");

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
            print_available_algorithms();
            throw std::logic_error(std::string("Error: the argument ('") +
                                   solver_name +
                                   "') for option '--algorithm' is invalid");
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
    if(vm.count("output-json")) {
        opt_output_json_file.emplace(
            vm["output-json"].as<std::filesystem::path>());
    }
    if(vm.count("output-csv")) {
        opt_output_csv_file.emplace(
            vm["output-csv"].as<std::filesystem::path>());
    }
    if(vm.count("quiet")) {
        spdlog::set_level(spdlog::level::err);
    } else if(vm.count("verbose")) {
        spdlog::set_level(spdlog::level::trace);
    } else {
        spdlog::set_level(spdlog::level::info);
    }
    std::vector<std::string> opts =
        po::collect_unrecognized(parsed.options, po::exclude_positional);
    solver->parse(opts);

    return true;
}

int main(int argc, const char * argv[]) {
#ifdef WIN32
    std::system("chcp 65001 > NUL");
#endif
    std::shared_ptr<AbstractSolver> solver;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<std::filesystem::path> opt_output_csv_file;

    spdlog::set_pattern("[%^%l%$] %v");
    std::string program_state = "Parsing arguments";

    try {
        std::vector<std::string> args(argv + 1, argv + argc);
        if(!process_command_line(args, solver, instances_description_json,
                                 budget, opt_output_json_file,
                                 opt_output_csv_file))
            return EXIT_SUCCESS;
        program_state = "Parsing instance";
        Instance raw_instance = parse_instance(instances_description_json);
        spdlog::info(
            "Parsed instance: {}",
            std::filesystem::absolute(instances_description_json).string());

        print_instance_size<spdlog::level::info>(raw_instance);
        double options_total_cost = 0;
        for(auto && o : raw_instance.options())
            options_total_cost += raw_instance.option_cost(o);
        spdlog::info(
            "Budget represents {:.2f}% of the total options cost (={:g})",
            budget / options_total_cost * 100, options_total_cost);

        Instance instance = trivial_reformulate_instance(raw_instance, budget);
        print_instance_size<spdlog::level::trace>(instance,
                                                  "Simplified instance");
        if(instance.num_options() == 0) {
            spdlog::warn("Budget doesn't allow purchasing any options");
        }

        spdlog::info("Calling algorithm '{}'", solver->name());
        program_state = "Running algorithm '" + solver->name() + '\'';
        spdlog::stopwatch sw;
        auto solution = solver->solve(instance, budget);
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();
        spdlog::info("Algorithm '{}' ran in {} ms", solver->name(),
                     computation_time_ms);

        program_state = "Printing solution";
        const double initial_score = gecot::compute_base_score(instance);
        const double solution_score =
            gecot::compute_solution_score(instance, solution);
        const double solution_cost =
            gecot::compute_solution_cost(instance, solution);

        if(!opt_output_json_file.has_value() &&
           !opt_output_csv_file.has_value()) {
            fmt::print(
                "Initial Score:  {}\nBudget: {}\nSolution score: {}\nSolution "
                "cost: {}\nSolution:\n",
                initial_score, budget, solution_score, solution_cost);
            const std::size_t option_name_max_length =
                std::ranges::max(std::ranges::views::transform(
                    raw_instance.options(), [&](auto && o) {
                        return raw_instance.option_name(o).size();
                    }));
            for(auto && option : raw_instance.options()) {
                const auto & option_name = raw_instance.option_name(option);
                int value = static_cast<int>(
                    instance.contains_option(option_name)
                        ? solution[instance.option_from_name(option_name)]
                        : false);
                fmt::println("    {:<{}} {}", option_name,
                             option_name_max_length, value);
            }
            fmt::println("Computation time: {} ms", computation_time_ms);
        }

        if(opt_output_json_file.has_value()) {
            auto out =
                fmt::output_file(opt_output_json_file.value().string().c_str());
            out.print(
                "{{\n    \"instance_path\":  \"{}\",\n    \"algorithm\":  "
                "\"{}\",\n    \"computation_time_ms\":  {},\n    "
                "\"initial_score\":  {},\n    \"budget\":         {},\n    "
                "\"solution_score\": {},\n    \"solution_cost\":  {},\n    "
                "\"solution\": {{",
                std::filesystem::absolute(instances_description_json).string(),
                solver->name(), computation_time_ms, initial_score, budget,
                solution_score, solution_cost);
            const std::size_t option_name_max_length =
                std::ranges::max(std::ranges::views::transform(
                    raw_instance.options(),
                    [&](auto && o) {
                        return raw_instance.option_name(o).size();
                    })) +
                3;
            bool first_line = true;
            for(auto && option : raw_instance.options()) {
                const auto & option_name = raw_instance.option_name(option);
                int value =
                    instance.contains_option(option_name)
                        ? solution[instance.option_from_name(option_name)]
                        : false;
                out.print("{}\n        {:<{}} {}", first_line ? "" : ",",
                          std::string("\"") + option_name + "\":",
                          option_name_max_length, value);
                first_line = false;
            }
            out.print("\n    }}\n}}");
        }

        if(opt_output_csv_file.has_value()) {
            auto out =
                fmt::output_file(opt_output_csv_file.value().string().c_str());
            out.print("option_id,value\n");
            for(auto && option : raw_instance.options()) {
                const auto & option_name = raw_instance.option_name(option);
                int value = static_cast<int>(
                    instance.contains_option(option_name)
                        ? solution[instance.option_from_name(option_name)]
                        : false);
                out.print("\"{}\",{}\n", option_name, value);
            }
            spdlog::info("Solution written to '{}'",
                         std::filesystem::absolute(opt_output_csv_file.value())
                             .string());
        }
    } catch(const std::exception & e) {
        spdlog::error("{}: {}", program_state, e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
