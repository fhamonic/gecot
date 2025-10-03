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

#include <tbb/global_control.h>

#include "gecot/helper.hpp"

#include "instance.hpp"
#include "io_helper.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"
#include "solver_interfaces/flow_benders_interface.hpp"
#include "solver_interfaces/greedy_decremental_interface.hpp"
#include "solver_interfaces/greedy_incremental_interface.hpp"
#include "solver_interfaces/mip_interface.hpp"
#include "solver_interfaces/naive_benders_interface.hpp"
#include "solver_interfaces/preprocessed_mip_interface.hpp"
#include "solver_interfaces/static_decremental_interface.hpp"
#include "solver_interfaces/static_incremental_interface.hpp"
#include "solver_interfaces/target_benders_interface.hpp"

using namespace fhamonic;

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolverInterface> & solver,
    std::filesystem::path & instances_description_json_file, double & budget,
    std::optional<std::filesystem::path> & opt_output_json_file,
    std::optional<std::filesystem::path> & opt_output_csv_file,
    std::optional<tbb::global_control> & opt_tbb_global_control) {
    std::vector<std::shared_ptr<AbstractSolverInterface>> solver_interfaces{
        std::make_unique<StaticIncrementalInterface>(),
        std::make_unique<StaticDecrementalInterface>(),
        std::make_unique<GreedyIncrementalInterface>(),
        std::make_unique<GreedyDecrementalInterface>(),
        std::make_unique<mipInterface>(),
        std::make_unique<PreprocessedmipInterface>(),
        std::make_unique<NaiveBendersInterface>(),
        std::make_unique<TargetBendersInterface>(),
        std::make_unique<FlowBendersInterface>()};

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
  gecot_optimize <algorithm> <instance> <budget> [<parameters> ...]

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

    po::options_description desc("Allowed parameters");
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
        "Output solution value in CSV file")(
        "verbose,v", "Enable logging of algorithms trace")(
        "quiet,q", "Silence all logging except errors")(
        "max-threads", po::value<std::size_t>(),
        "Limits the number of threads used");

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
    if(vm.count("max-threads")) {
        opt_tbb_global_control.emplace(
            tbb::global_control::max_allowed_parallelism,
            vm.at("max-threads").as<std::size_t>());
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
    std::shared_ptr<AbstractSolverInterface> solver;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<std::filesystem::path> opt_output_csv_file;
    std::optional<tbb::global_control> opt_tbb_global_control;

    spdlog::set_pattern("[%^%l%$] %v");
    std::string program_state = "Parsing arguments";

    // try {
    std::vector<std::string> args(argv + 1, argv + argc);
    if(!process_command_line(args, solver, instances_description_json, budget,
                             opt_output_json_file, opt_output_csv_file,
                             opt_tbb_global_control))
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
    spdlog::info("Budget represents {:.2f}% of the total options cost (={:g})",
                 budget / options_total_cost * 100, options_total_cost);

    Instance instance = trivial_reformulate_instance(raw_instance, budget);
    print_instance_size<spdlog::level::trace>(instance, "Simplified instance");
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
    const auto initial_cases_pc_num =
        gecot::compute_base_cases_pc_num(instance);
    const double initial_score = instance.eval_criterion(initial_cases_pc_num);
    const auto solution_cases_pc_num =
        gecot::compute_solution_cases_pc_num(instance, solution);
    const double solution_score =
        instance.eval_criterion(solution_cases_pc_num);
    const double solution_cost =
        gecot::compute_solution_cost(instance, solution);

    if(!opt_output_json_file.has_value() && !opt_output_csv_file.has_value()) {
        fmt::print(
            "Initial Score:  {}\nBudget: {}\nSolution score: {}\nSolution "
            "cost: {}\nPC_num:\n",
            initial_score, budget, solution_score, solution_cost);
        for(const auto & instance_case : instance.cases()) {
            fmt::print("\t{}: {}\n", instance_case.name(),
                       solution_cases_pc_num[instance_case.id()]);
        }
        fmt::print("Solution:\n");
        const std::size_t option_name_max_length =
            std::ranges::max(std::ranges::views::transform(
                raw_instance.options(),
                [&](auto && o) { return raw_instance.option_name(o).size(); }));
        for(auto && option : raw_instance.options()) {
            const auto & option_name = raw_instance.option_name(option);
            int value = static_cast<int>(
                instance.contains_option(option_name)
                    ? solution[instance.option_from_name(option_name)]
                    : false);
            fmt::println("    {:<{}} {}", option_name, option_name_max_length,
                         value);
        }
        fmt::println("Computation time: {} ms", computation_time_ms);
    }

    if(opt_output_json_file.has_value()) {
        auto out =
            fmt::output_file(opt_output_json_file.value().string().c_str());
        auto print_property = [&](const std::string & property_name,
                                  auto && pairs_range) {
            out.print("\n    \"{}\": {{", property_name);
            const std::size_t key_max_length =
                std::ranges::max(std::ranges::views::transform(
                    std::views::keys(pairs_range),
                    [&](auto && s) { return s.size(); })) +
                3;
            bool first_line = true;
            for(const auto & [key, value] : pairs_range) {
                out.print("{}\n        {:<{}} {}", first_line ? "" : ",",
                          std::string("\"") + key + "\":", key_max_length,
                          value);
                first_line = false;
            }
            out.print("\n    }}");
        };
        out.print(
            "{{\n    \"instance_path\":  \"{}\",\n    \"algorithm\":  "
            "\"{}\",\n    \"computation_time_ms\":  {},\n    "
            "\"initial_score\":  {},",
            std::filesystem::absolute(instances_description_json)
                .generic_string(),
            solver->name(), computation_time_ms, initial_score);
        print_property("initial_pc_num",
                       std::views::transform(
                           instance.cases(),
                           [&initial_cases_pc_num](const auto & instance_case) {
                               return std::make_pair(
                                   instance_case.name(),
                                   initial_cases_pc_num[instance_case.id()]);
                           }));
        out.print(
            ",\n    \"budget\":         {},\n    "
            "\"solution_score\": {},",
            budget, solution_score);
        print_property("solution_pc_num",
                       std::views::transform(
                           instance.cases(), [&solution_cases_pc_num](
                                                 const auto & instance_case) {
                               return std::make_pair(
                                   instance_case.name(),
                                   solution_cases_pc_num[instance_case.id()]);
                           }));
        out.print(",\n    \"solution_cost\":  {},", solution_cost);
        print_property(
            "solution",
            std::views::transform(
                raw_instance.options(),
                [&raw_instance, &instance, &solution](const auto & option) {
                    const auto & option_name = raw_instance.option_name(option);
                    return std::make_pair(
                        option_name,
                        static_cast<int>(
                            solution[instance.option_from_name(option_name)]));
                }));
        out.print("\n}}");

        spdlog::info(
            "Solution printed to '{}'",
            std::filesystem::absolute(opt_output_json_file.value()).string());
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
        spdlog::info(
            "Solution written to '{}'",
            std::filesystem::absolute(opt_output_csv_file.value()).string());

        spdlog::info(
            "Solution printed to '{}'",
            std::filesystem::absolute(opt_output_csv_file.value()).string());
    }
    // } catch(const std::exception & e) {
    //     spdlog::error("{}: {}", program_state, e.what());
    //     return EXIT_FAILURE;
    // }

    return EXIT_SUCCESS;
}
