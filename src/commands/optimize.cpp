#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <vector>

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
    std::optional<tbb::global_control> & opt_tbb_global_control,
    std::shared_ptr<const PathFormulationLPInterface> & path_reformulation) {
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
         quiet = false;
    int verbosity = 0;
    std::string solver_name, reformulation, reformulation_params;
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
    app.add_option("--reformulation", reformulation,
                   "Instance given to the algorithm, trivial: without the "
                   "options of cost 0, bought, or over the budget; path_lp: "
                   "also without the options that the path relaxation proves "
                   "to be in no better solution than its own")
        ->check(CLI::IsMember({"trivial", "path_lp"}))
        ->default_val("trivial");
    app.add_option("--reformulation-params", reformulation_params,
                   "Parameters of the path_lp of --reformulation path_lp, as "
                   "\"time-limit=60 pricing=enumerate\"; its MIP start is "
                   "greedy_incr's solution by default");
    app.add_flag("-v,--verbose", verbosity,
                 "Log the algorithms' details, -vv everything with the "
                 "solver's own log on stdout");
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
            throw std::logic_error("unknown algorithm '" + solver_name + "'");
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
    apply_logging(verbosity, quiet);
    if(opt_max_threads.has_value()) {
        opt_tbb_global_control.emplace(
            tbb::global_control::max_allowed_parallelism,
            opt_max_threads.value());
    }

    // remaining() is in command line order: the solver would read '-t 1e-5'
    // as '1e-5 -t'
    solver->parse(app.remaining_for_passthrough());
    if(reformulation == "path_lp")
        path_reformulation = make_path_reformulation(reformulation_params);
    else if(!reformulation_params.empty())
        throw std::logic_error(
            "'--reformulation-params' requires '--reformulation path_lp'");

    return true;
}

int optimize_command(const std::vector<std::string> & args) {
    std::shared_ptr<AbstractSolverInterface> solver;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<std::filesystem::path> opt_output_csv_file;
    std::optional<tbb::global_control> opt_tbb_global_control;
    std::shared_ptr<const PathFormulationLPInterface> path_reformulation;

    std::string program_state = "Parsing arguments";

    try {
        if(!process_command_line(args, solver, instances_description_json,
                                 budget, opt_output_json_file,
                                 opt_output_csv_file, opt_tbb_global_control,
                                 path_reformulation))
            return EXIT_SUCCESS;
        program_state = "Checking output files";
        check_output_directories({opt_output_json_file, opt_output_csv_file});
        spdlog::info("GECOT {}, algorithm {}", PROJECT_VERSION, solver->name());
        log_parameters(solver->parameters());
        program_state = "Parsing instance";
        const prepared_instance prepared =
            prepare_instance(instances_description_json, budget, "algorithm");
        const Instance & raw_instance = prepared.raw_instance;
        const Instance & instance = prepared.instance;
        // the algorithm's scores, bounds and logs are in this unit
        const double score_scale = prepared.score_scale;
        const auto & initial_cases_pc_num = prepared.initial_cases_pc_num;
        const double initial_score = prepared.initial_score;

        program_state = "Running algorithm '" + solver->name() + '\'';
        spdlog::stopwatch sw;
        auto [solution, report] = [&] {
            const progress_watch watch(prepared);
            return instance.num_options() == 0
                       ? gecot::solve_result<Instance>{instance
                                                           .create_option_map(
                                                               false),
                                                       {"no option left"}}
                   : path_reformulation
                       ? solve_path_reformulated(*path_reformulation, *solver,
                                                 instance, budget)
                       : solver->solve(instance, budget);
        }();
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();

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
        std::optional<double> best_bound;
        if(report.best_bound) {
            const double bound = report.best_bound.value() * score_scale;
            report.stats.emplace_back("best_bound", bound);
            if(solution_score > 0)
                report.stats.emplace_back(
                    "gap", (bound - solution_score) / solution_score);
            // at the optimum, a few ulps below the recomputed score
            best_bound = std::max(bound, solution_score);
        }
        if(solution_cost > budget + 1e-6 * std::max(1.0, budget))
            spdlog::warn("The solution costs {:.10g}, over the budget {:.10g}",
                         solution_cost, budget);
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
            fmt::println("Status          {}", report.status);
            fmt::println("Initial score   {}", format_score(initial_score));
            fmt::println("Solution score  {} ({})",
                         format_score(solution_score),
                         format_gain(solution_score, initial_score));
            if(best_bound)
                fmt::println(
                    "Upper bound     {}{}", format_score(best_bound.value()),
                    guaranteed_gain_share(solution_score, best_bound.value(),
                                          prepared.base_score)
                        .transform([](double share) {
                            return fmt::format(
                                " (at least {:.1f} % of the best possible "
                                "gain)",
                                share);
                        })
                        .value_or(""));
            fmt::println("Cost            {:.10g} of {:.10g}", solution_cost,
                         budget);
            fmt::println(
                "Time            {}",
                format_duration(static_cast<double>(computation_time_ms) /
                                1000.0));
            if(instance.cases().size() > 1) {
                fmt::println("Per species");
                const std::size_t name_max_length = std::ranges::max(
                    std::views::transform(instance.cases(), [](auto && c) {
                        return c.name().size();
                    }));
                for(const auto & instance_case : instance.cases())
                    fmt::println(
                        "    {:<{}} {} ({})", instance_case.name() + ':',
                        name_max_length + 1,
                        format_score(solution_cases_pc_num[instance_case.id()]),
                        format_gain(solution_cases_pc_num[instance_case.id()],
                                    initial_cases_pc_num[instance_case.id()]));
            }
            const auto bought = std::ranges::to<std::vector>(std::views::filter(
                raw_instance.options(),
                [&](auto && option) { return option_value(option); }));
            fmt::println("Options bought  {} of {}{}", bought.size(),
                         raw_instance.num_options(), bought.empty() ? "" : ":");
            for(auto && option : bought)
                fmt::println("    {}", raw_instance.option_name(option));
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

            spdlog::info("Solution written to {}",
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
            spdlog::info("Solution written to {}",
                         std::filesystem::absolute(opt_output_csv_file.value())
                             .string());
        }
    } catch(const std::exception & e) {
        spdlog::error("{}: {}", program_state, e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
