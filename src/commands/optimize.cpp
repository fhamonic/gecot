#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <print>
#include <ranges>
#include <vector>

#include <CLI/CLI.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/global_control.h>

#include "gecot/helper.hpp"

#include "commands/commands.hpp"
#include "commands/common.hpp"

#include "instance.hpp"
#include "io_helper.hpp"
#include "parse_instance.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractSolverInterface> & solver,
    std::filesystem::path & instances_description_json_file,
    std::string & budget,
    std::optional<std::filesystem::path> & opt_output_json_file,
    std::optional<std::filesystem::path> & opt_output_csv_file,
    std::optional<tbb::global_control> & opt_tbb_global_control,
    std::shared_ptr<const PathFormulationLPInterface> & path_reformulation,
    bool & no_merging,
    std::vector<std::pair<std::string, std::string>> & command_parameters) {
    auto solver_interfaces = make_solver_interfaces();

    auto print_available_algorithms = [&solver_interfaces]() {
        std::println("Available algorithms:");
        const std::size_t algorithm_name_max_length = std::ranges::max(
            std::ranges::views::transform(solver_interfaces, [&](auto && s) {
                return s->name().size();
            }));
        const std::size_t offset = algorithm_name_max_length + 8;

        for(auto & s : solver_interfaces) {
            std::print("  {:<{}}", s->name(), offset - 2);
            print_paragraph(offset, 80 - offset, s->description());
        }
        std::println("");
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
        app.add_option("budget,-B,--budget", budget,
                       "Budget value, or name=value,... for options of "
                       "several costs")
            ->type_name("BUDGET");
    app.add_option("-o,--output-json", opt_output_json_file,
                   "Output solution data in JSON file");
    app.add_option("--output-csv", opt_output_csv_file,
                   "Output solution value in CSV file");
    app.add_option("--reformulation", reformulation,
                   "Instance given to the algorithm, trivial: without the "
                   "options over the budget, and those of cost 0, bought "
                   "unless a conflict set makes them a choice; path_lp: also "
                   "without the options that the path relaxation proves to be "
                   "in no better solution than its own")
        ->check(CLI::IsMember({"trivial", "path_lp"}))
        ->default_val("trivial");
    app.add_option("--reformulation-params", reformulation_params,
                   "Parameters of the path_lp of --reformulation path_lp, as "
                   "\"time-limit=60 pricing=enumerate\"; its MIP start is "
                   "greedy's solution by default");
    app.add_flag("--no-merging", no_merging,
                 "For comparison: keeps apart the vertices that arcs of "
                 "probability 1 link, merged in the instance given to the "
                 "algorithm");
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
    // the names are checked with the instance
    if(!budget.contains('=') && !parse_budget_value(budget))
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
    // the options of the command that change the instance of the algorithm
    for(const char * name :
        {"--reformulation", "--reformulation-params", "--no-merging"})
        command_parameters.emplace_back(
            name + 2, option_json_value(*app.get_option(name)));

    return true;
}

int optimize_command(const std::vector<std::string> & args) {
    std::shared_ptr<AbstractSolverInterface> solver;
    std::filesystem::path instances_description_json;
    std::string budget_text;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<std::filesystem::path> opt_output_csv_file;
    std::optional<tbb::global_control> opt_tbb_global_control;
    std::shared_ptr<const PathFormulationLPInterface> path_reformulation;
    bool no_merging = false;
    std::vector<std::pair<std::string, std::string>> command_parameters;

    std::string program_state = "Parsing arguments";

    try {
        if(!process_command_line(args, solver, instances_description_json,
                                 budget_text, opt_output_json_file,
                                 opt_output_csv_file, opt_tbb_global_control,
                                 path_reformulation, no_merging,
                                 command_parameters))
            return EXIT_SUCCESS;
        program_state = "Checking output files";
        check_output_directories({opt_output_json_file, opt_output_csv_file});
        spdlog::info("GECOT {}, algorithm {}", PROJECT_VERSION, solver->name());
        log_parameters(solver->parameters());
        program_state = "Parsing instance";
        const prepared_instance prepared = prepare_instance(
            instances_description_json, budget_text, "algorithm", !no_merging);
        const Instance & raw_instance = prepared.raw_instance;
        const Instance & instance = prepared.instance;
        const gecot::budget_t & budget = prepared.budget;
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
                                                 instance, budget, !no_merging)
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
        std::vector<double> solution_cost;
        for(std::size_t r = 0; r < budget.size(); ++r)
            solution_cost.push_back(
                gecot::compute_solution_cost(instance, solution, r));
        for(const auto & [key, value] : report.score_stats)
            report.stats.emplace_back(key, value * score_scale);
        auto option_value = [&](const gecot::option_t & option) {
            const auto & option_name = raw_instance.option_name(option);
            return static_cast<int>(
                prepared.static_options[option] ||
                (instance.contains_option(option_name) &&
                 solution[instance.option_from_name(option_name)]));
        };
        bool feasible = true;
        for(std::size_t r = 0; r < budget.size(); ++r) {
            if(solution_cost[r] <= budget[r] + 1e-6 * std::max(1.0, budget[r]))
                continue;
            feasible = false;
            if(budget.size() == 1)
                spdlog::warn(
                    "The solution costs {:.10g}, over the budget {:.10g}",
                    solution_cost[r], budget[r]);
            else
                spdlog::warn(
                    "The solution costs {:.10g} in {}, over the budget {:.10g}",
                    solution_cost[r], raw_instance.cost_dimension_name(r),
                    budget[r]);
        }
        // on the raw sets, which also checks the reformulation
        for(std::size_t l = 0; l < raw_instance.conflict_sets().size(); ++l) {
            const auto bought = std::ranges::count_if(
                raw_instance.conflict_sets()[l], option_value);
            if(bought > 1) {
                feasible = false;
                spdlog::warn(
                    "The solution buys {} options of the conflict set '{}'",
                    bought, raw_instance.conflict_set_name(l));
            }
        }
        std::optional<double> best_bound;
        if(report.best_bound) {
            const double bound = report.best_bound.value() * score_scale;
            report.stats.emplace_back("best_bound", bound);
            if(solution_score > 0)
                report.stats.emplace_back(
                    "gap", (bound - solution_score) / solution_score);
            // at the optimum, a few ulps below the recomputed score; an
            // infeasible selection is no proof of its own score
            best_bound = feasible ? std::max(bound, solution_score) : bound;
        }

        if(!opt_output_json_file.has_value() &&
           !opt_output_csv_file.has_value()) {
            std::println("Status          {}", report.status);
            std::println("Initial score   {}", format_score(initial_score));
            std::println("Solution score  {} ({})",
                         format_score(solution_score),
                         format_gain(solution_score, initial_score));
            if(best_bound)
                std::println(
                    "Upper bound     {}{}", format_score(best_bound.value()),
                    (feasible ? guaranteed_gain_share(solution_score,
                                                      best_bound.value(),
                                                      prepared.base_score)
                              : std::nullopt)
                        .transform([](double share) {
                            return std::format(
                                " (at least {:.1f} % of the best possible "
                                "gain)",
                                share);
                        })
                        .value_or(""));
            for(std::size_t r = 0; r < budget.size(); ++r)
                if(budget.size() == 1)
                    std::println("Cost            {:.10g} of {:.10g}",
                                 solution_cost[r], budget[r]);
                else
                    std::println("Cost {:<10} {:.10g} of {:.10g}",
                                 raw_instance.cost_dimension_name(r),
                                 solution_cost[r], budget[r]);
            std::println(
                "Time            {}",
                format_duration(static_cast<double>(computation_time_ms) /
                                1000.0));
            if(instance.cases().size() > 1) {
                std::println("Per species");
                const std::size_t name_max_length = std::ranges::max(
                    std::views::transform(instance.cases(), [](auto && c) {
                        return c.name().size();
                    }));
                for(const auto & instance_case : instance.cases())
                    std::println(
                        "    {:<{}} {} ({})", instance_case.name() + ':',
                        name_max_length + 1,
                        format_score(solution_cases_pc_num[instance_case.id()]),
                        format_gain(solution_cases_pc_num[instance_case.id()],
                                    initial_cases_pc_num[instance_case.id()]));
            }
            const auto bought = std::ranges::to<std::vector>(std::views::filter(
                raw_instance.options(),
                [&](auto && option) { return option_value(option); }));
            std::println("Options bought  {} of {}{}", bought.size(),
                         raw_instance.num_options(), bought.empty() ? "" : ":");
            for(auto && option : bought)
                std::println("    {}", raw_instance.option_name(option));
        }

        if(opt_output_json_file.has_value()) {
            const auto file = open_output_file(
                std::filesystem::absolute(opt_output_json_file.value()));
            std::FILE * stream = file.get();
            std::print(
                stream,
                "{{\n    \"instance_path\":  \"{}\",\n    \"algorithm\":  "
                "\"{}\",",
                std::filesystem::absolute(instances_description_json)
                    .generic_string(),
                solver->name());
            print_json_property(stream, "algorithm_parameters",
                                solver->json_parameters());
            std::print(stream, ",");
            print_json_property(stream, "command_parameters",
                                command_parameters);
            std::print(stream,
                       ",\n    \"computation_time_ms\":  {},\n    "
                       "\"initial_score\":  {},",
                       computation_time_ms, initial_score);
            print_json_property(
                stream, "initial_pc_num",
                std::views::transform(
                    instance.cases(),
                    [&initial_cases_pc_num](const auto & instance_case) {
                        return std::make_pair(
                            instance_case.name(),
                            initial_cases_pc_num[instance_case.id()]);
                    }));
            std::print(stream, ",");
            print_json_per_dimension(stream, "budget",
                                     "\"budget\":         ", raw_instance,
                                     budget.values);
            std::print(stream, ",\n    \"solution_score\": {},",
                       solution_score);
            print_json_property(
                stream, "solution_pc_num",
                std::views::transform(
                    instance.cases(),
                    [&solution_cases_pc_num](const auto & instance_case) {
                        return std::make_pair(
                            instance_case.name(),
                            solution_cases_pc_num[instance_case.id()]);
                    }));
            std::print(stream, ",");
            print_json_per_dimension(stream, "solution_cost",
                                     "\"solution_cost\":  ", raw_instance,
                                     solution_cost);
            std::print(stream, ",\n    \"feasible\":       {},", feasible);
            print_json_property(
                stream, "solution",
                std::views::transform(
                    raw_instance.options(),
                    [&raw_instance, &option_value](const auto & option) {
                        return std::make_pair(raw_instance.option_name(option),
                                              option_value(option));
                    }));
            std::print(
                stream,
                ",\n    \"algorithm_report\": {{\n        \"status\": \"{}\"",
                report.status);
            for(const auto & [key, value] : report.stats)
                std::print(stream, ",\n        \"{}\": {}", key, value);
            std::print(stream, "\n    }}\n}}");

            spdlog::info("Solution written to {}",
                         std::filesystem::absolute(opt_output_json_file.value())
                             .string());
        }

        if(opt_output_csv_file.has_value()) {
            const auto file = open_output_file(opt_output_csv_file.value());
            std::FILE * stream = file.get();
            std::print(stream, "option_id,value\n");
            for(auto && option : raw_instance.options()) {
                std::print(stream, "\"{}\",{}\n",
                           raw_instance.option_name(option),
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
