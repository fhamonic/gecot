#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>

#include <CLI/CLI.hpp>

#include <fmt/core.h>
#include <fmt/os.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/global_control.h>

#include "gecot/helper.hpp"

#include "commands/commands.hpp"
#include "commands/common.hpp"

#include "instance.hpp"
#include "io_helper.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolverInterface> & solver,
    std::filesystem::path & instances_description_json_file, double & budget,
    std::optional<std::filesystem::path> & opt_output_json_file,
    std::optional<std::filesystem::path> & opt_output_csv_file,
    std::optional<tbb::global_control> & opt_tbb_global_control) {
    auto solver_interfaces = make_solver_interfaces();

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

    bool help = false, list_algorithms = false, list_params = false,
         verbose = false, quiet = false;
    std::string solver_name;
    std::optional<std::size_t> opt_max_threads;
    CLI::App app("", "gecot optimize");
    app.usage(R"(Usage:
  gecot optimize --help
  gecot optimize --list-algorithms
  gecot optimize <algorithm> --list-params
  gecot optimize <algorithm> <instance> <budget> [<parameters> ...])");
    app.set_help_flag();
    app.add_flag("-h,--help", help, "Display this help message");
    app.add_flag("-A,--list-algorithms", list_algorithms,
                 "List the available algorithms");
    app.add_flag("-P,--list-params", list_params,
                 "List the parameters of the chosen algorithm");
    // a name without dashes also makes the option positional
    auto * algorithm_opt = app.add_option("algorithm,-a,--algorithm",
                                          solver_name, "Algorithm to use");
    auto * instance_opt =
        app.add_option("instance,-i,--instance",
                       instances_description_json_file, "Instance JSON file");
    auto * budget_opt =
        app.add_option("budget,-B,--budget", budget, "Budget value");
    app.add_option("-o,--output-json", opt_output_json_file,
                   "Output solution data in JSON file");
    app.add_option("--output-csv", opt_output_csv_file,
                   "Output solution value in CSV file");
    app.add_flag("-v,--verbose", verbose, "Enable logging of algorithms trace");
    app.add_flag("-q,--quiet", quiet, "Silence all logging except errors");
    app.add_option("--max-threads", opt_max_threads,
                   "Limits the number of threads used")
        ->check(CLI::Range(std::size_t{1},
                           std::numeric_limits<std::size_t>::max(),
                           "POSITIVE"));
    // unknown options and surplus values are left for solver->parse
    app.allow_extras();
    // else on Windows CLI11 takes a '/abs/path' instance for an option
    app.allow_windows_style_options(false);
    // CLI::App::parse consumes the args from the back
    app.parse(std::vector<std::string>(args.rbegin(), args.rend()));

    if(help || args.empty()) {
        print_soft_name();
        std::cout << app.help() << std::endl;
        return false;
    }

    if(list_algorithms) {
        print_soft_name();
        print_available_algorithms();
        return false;
    }

    if(algorithm_opt->count()) {
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

    if(list_params) {
        if(!algorithm_opt->count()) {
            throw std::logic_error(std::string(
                "Option '--list-params' requires option '--algorithm'."));
        }
        print_soft_name();
        std::cout << solver->options_description();
        std::cout << std::endl;
        return false;
    }

    for(auto * opt : {algorithm_opt, instance_opt, budget_opt})
        if(!opt->count())
            throw std::logic_error("the option '" + opt->get_name() +
                                   "' is required but missing");
    if(!std::isfinite(budget) || budget < 0)
        throw std::logic_error("the budget must be a non-negative number");
    apply_logging(verbose, quiet);
    if(opt_max_threads.has_value()) {
        opt_tbb_global_control.emplace(
            tbb::global_control::max_allowed_parallelism,
            opt_max_threads.value());
    }

    // remaining() is in command line order: the solver would read '-t 1e-5'
    // as '1e-5 -t'
    solver->parse(app.remaining_for_passthrough());

    return true;
}

int optimize_command(const std::vector<std::string> & args) {
    std::shared_ptr<AbstractSolverInterface> solver;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<std::filesystem::path> opt_output_csv_file;
    std::optional<tbb::global_control> opt_tbb_global_control;

    std::string program_state = "Parsing arguments";

    try {
        if(!process_command_line(args, solver, instances_description_json,
                                 budget, opt_output_json_file,
                                 opt_output_csv_file, opt_tbb_global_control))
            return EXIT_SUCCESS;
        program_state = "Checking output files";
        check_output_directories({opt_output_json_file, opt_output_csv_file});
        program_state = "Parsing instance";
        const prepared_instance prepared =
            prepare_instance(instances_description_json, budget, "algorithm");
        const Instance & raw_instance = prepared.raw_instance;
        const Instance & instance = prepared.instance;
        // the algorithm's scores, bounds and logs are in this unit
        const double score_scale = prepared.score_scale;
        const auto & initial_cases_pc_num = prepared.initial_cases_pc_num;
        const double initial_score = prepared.initial_score;

        spdlog::info("Calling algorithm '{}'", solver->name());
        program_state = "Running algorithm '" + solver->name() + '\'';
        spdlog::stopwatch sw;
        auto [solution, report] =
            instance.num_options() == 0
                ? gecot::solve_result<Instance>{instance.create_option_map(
                                                    false),
                                                {"no option left"}}
                : solver->solve(instance, budget);
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();
        spdlog::info("Algorithm '{}' ran in {} ms", solver->name(),
                     computation_time_ms);

        program_state = "Printing solution";
        auto solution_cases_pc_num =
            gecot::compute_solution_cases_pc_num(instance, solution);
        for(const auto & instance_case : instance.cases())
            solution_cases_pc_num[instance_case.id()] *= score_scale;
        const double solution_score =
            raw_instance.eval_criterion(solution_cases_pc_num);
        const double solution_cost =
            gecot::compute_solution_cost(instance, solution);
        for(const auto & [key, value] : report.score_stats)
            report.stats.emplace_back(key, value * score_scale);
        if(report.best_bound) {
            const double bound = report.best_bound.value() * score_scale;
            report.stats.emplace_back("best_bound", bound);
            if(solution_score > 0)
                report.stats.emplace_back(
                    "gap", (bound - solution_score) / solution_score);
        }
        // free options were bought by trivial_reformulate_instance
        auto option_value = [&](const gecot::option_t & option) {
            const auto & option_name = raw_instance.option_name(option);
            return static_cast<int>(
                raw_instance.option_cost(option) == 0 ||
                (instance.contains_option(option_name) &&
                 solution[instance.option_from_name(option_name)]));
        };

        if(!opt_output_json_file.has_value() &&
           !opt_output_csv_file.has_value()) {
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
                raw_instance.options().empty()
                    ? 0
                    : std::ranges::max(std::ranges::views::transform(
                          raw_instance.options(), [&](auto && o) {
                              return raw_instance.option_name(o).size();
                          }));
            for(auto && option : raw_instance.options()) {
                fmt::println("    {:<{}} {}", raw_instance.option_name(option),
                             option_name_max_length, option_value(option));
            }
            fmt::println("Computation time: {} ms", computation_time_ms);
        }

        if(opt_output_json_file.has_value()) {
            auto out = fmt::output_file(
                std::filesystem::absolute(opt_output_json_file.value())
                    .string()
                    .c_str());
            out.print(
                "{{\n    \"instance_path\":  \"{}\",\n    \"algorithm\":  "
                "\"{}\",\n    \"computation_time_ms\":  {},\n    "
                "\"initial_score\":  {},",
                std::filesystem::absolute(instances_description_json)
                    .generic_string(),
                solver->name(), computation_time_ms, initial_score);
            print_json_property(
                out, "initial_pc_num",
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
            print_json_property(
                out, "solution_pc_num",
                std::views::transform(
                    instance.cases(),
                    [&solution_cases_pc_num](const auto & instance_case) {
                        return std::make_pair(
                            instance_case.name(),
                            solution_cases_pc_num[instance_case.id()]);
                    }));
            out.print(",\n    \"solution_cost\":  {},", solution_cost);
            print_json_property(
                out, "solution",
                std::views::transform(
                    raw_instance.options(),
                    [&raw_instance, &option_value](const auto & option) {
                        return std::make_pair(raw_instance.option_name(option),
                                              option_value(option));
                    }));
            out.print(
                ",\n    \"algorithm_report\": {{\n        \"status\": \"{}\"",
                report.status);
            for(const auto & [key, value] : report.stats)
                out.print(",\n        \"{}\": {}", key, value);
            out.print("\n    }}\n}}");

            spdlog::info("Solution printed to '{}'",
                         std::filesystem::absolute(opt_output_json_file.value())
                             .string());
        }

        if(opt_output_csv_file.has_value()) {
            auto out =
                fmt::output_file(opt_output_csv_file.value().string().c_str());
            out.print("option_id,value\n");
            for(auto && option : raw_instance.options()) {
                out.print("\"{}\",{}\n", raw_instance.option_name(option),
                          option_value(option));
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
