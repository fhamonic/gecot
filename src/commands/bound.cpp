#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>

#include <CLI/CLI.hpp>

#include <fmt/core.h>
#include <fmt/os.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/global_control.h>

#include "gecot/bounds/all_options.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/helper.hpp"

#include "commands/commands.hpp"
#include "commands/common.hpp"

#include "instance.hpp"
#include "io_helper.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractBoundInterface> & method,
    std::filesystem::path & instances_description_json_file, double & budget,
    std::optional<std::filesystem::path> & opt_output_json_file,
    std::optional<tbb::global_control> & opt_tbb_global_control) {
    auto methods = make_bound_interfaces();

    auto print_available_methods = [&methods]() {
        fmt::println("Available methods:");
        const std::size_t method_name_max_length =
            std::ranges::max(std::ranges::views::transform(
                methods, [&](auto && m) { return m->name().size(); }));
        const std::size_t offset = method_name_max_length + 8;

        for(auto & m : methods) {
            fmt::print("  {:<{}}", m->name(), offset - 2);
            print_paragraph(offset, 80 - offset, m->description());
        }
        fmt::println("");
    };

    bool help = false, list_methods = false, list_params = false,
         verbose = false, quiet = false;
    std::string method_name;
    std::optional<std::size_t> opt_max_threads;
    CLI::App app("", "gecot bound");
    app.usage(R"(Usage:
  gecot bound --help
  gecot bound --list-methods
  gecot bound <method> --list-params
  gecot bound <method> <instance> <budget> [<parameters> ...])");
    app.set_help_flag();
    app.add_flag("-h,--help", help, "Display this help message");
    app.add_flag("-M,--list-methods", list_methods,
                 "List the available bound methods");
    app.add_flag("-P,--list-params", list_params,
                 "List the parameters of the chosen method");
    // a name without dashes also makes the option positional
    auto * method_opt =
        app.add_option("method,--method", method_name, "Bound method to use");
    auto * instance_opt =
        app.add_option("instance,-i,--instance",
                       instances_description_json_file, "Instance JSON file");
    auto * budget_opt =
        app.add_option("budget,-B,--budget", budget, "Budget value");
    app.add_option("-o,--output-json", opt_output_json_file,
                   "Output bound data in JSON file");
    app.add_flag("-v,--verbose", verbose, "Enable logging of methods trace");
    app.add_flag("-q,--quiet", quiet, "Silence all logging except errors");
    app.add_option("--max-threads", opt_max_threads,
                   "Limits the number of threads used")
        ->check(CLI::Range(std::size_t{1},
                           std::numeric_limits<std::size_t>::max(),
                           "POSITIVE"));
    // unknown options and surplus values are left for method->parse
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

    if(list_methods) {
        print_soft_name();
        print_available_methods();
        return false;
    }

    if(method_opt->count()) {
        const auto it = std::ranges::find(methods, method_name,
                                          [](auto && m) { return m->name(); });
        if(it == methods.end()) {
            if(std::ranges::contains(make_solver_interfaces(), method_name,
                                     [](auto && s) { return s->name(); }))
                throw std::logic_error("'" + method_name +
                                       "' proves no bound, see 'gecot bound "
                                       "--list-methods'");
            print_available_methods();
            throw std::logic_error(std::string("Error: the argument ('") +
                                   method_name +
                                   "') for option '--method' is invalid");
        }
        method = *it;
    }

    if(list_params) {
        if(!method_opt->count()) {
            throw std::logic_error(std::string(
                "Option '--list-params' requires option '--method'."));
        }
        print_soft_name();
        std::cout << method->options_description();
        std::cout << std::endl;
        return false;
    }

    for(auto * opt : {method_opt, instance_opt, budget_opt})
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

    // remaining() is in command line order: the method would read '-t 1e-5'
    // as '1e-5 -t'
    method->parse(app.remaining_for_passthrough());

    return true;
}

int bound_command(const std::vector<std::string> & args) {
    std::shared_ptr<AbstractBoundInterface> method;
    std::filesystem::path instances_description_json;
    double budget;
    std::optional<std::filesystem::path> opt_output_json_file;
    std::optional<tbb::global_control> opt_tbb_global_control;

    std::string program_state = "Parsing arguments";

    try {
        if(!process_command_line(args, method, instances_description_json,
                                 budget, opt_output_json_file,
                                 opt_tbb_global_control))
            return EXIT_SUCCESS;
        program_state = "Checking output files";
        check_output_directories({opt_output_json_file});
        program_state = "Parsing instance";
        const prepared_instance prepared =
            prepare_instance(instances_description_json, budget, "method");
        const Instance & instance = prepared.instance;
        // the method's scores, bounds and logs are in this unit
        const double score_scale = prepared.score_scale;
        // the solutions are those of 'gecot optimize', the options of cost 0
        // bought: buying no other is one
        const double base_score =
            instance.eval_criterion(gecot::compute_base_cases_pc_num(instance));

        spdlog::info("Calling method '{}'", method->name());
        program_state = "Running method '" + method->name() + '\'';
        spdlog::stopwatch sw;
        gecot::bound_report report =
            instance.num_options() == 0
                ? gecot::bound_report{base_score, "no option left"}
                : method->compute(instance, budget, {score_scale});
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();
        spdlog::info("Method '{}' ran in {} ms", method->name(),
                     computation_time_ms);

        program_state = "Checking the bound of '" + method->name() + '\'';
        const double bound = report.bound * score_scale;
        if(bound < base_score * score_scale -
                       1e-9 * std::max(1.0, std::abs(base_score * score_scale)))
            throw std::runtime_error(fmt::format(
                "the bound {} is below the score {} of buying only the options "
                "of cost 0",
                bound, base_score * score_scale));
        program_state = "Printing bound";
        std::optional<double> all_options_bound;
        if(gecot::is_non_decreasing(instance.criterion()))
            all_options_bound =
                gecot::all_options_bound(instance).bound * score_scale;
        for(const auto & [key, value] : report.score_stats)
            report.stats.emplace_back(key, value * score_scale);

        if(!opt_output_json_file.has_value()) {
            fmt::print("Initial Score:  {}\nBudget: {}\nUpper bound: {}\n",
                       prepared.initial_score, budget, bound);
            if(all_options_bound)
                fmt::print("All options bound: {}\n",
                           all_options_bound.value());
            fmt::println("Computation time: {} ms", computation_time_ms);
        }

        if(opt_output_json_file.has_value()) {
            auto out = fmt::output_file(
                std::filesystem::absolute(opt_output_json_file.value())
                    .string()
                    .c_str());
            out.print(
                "{{\n    \"instance_path\":  \"{}\",\n    \"method\":  "
                "\"{}\",\n    \"computation_time_ms\":  {},\n    "
                "\"initial_score\":  {},",
                std::filesystem::absolute(instances_description_json)
                    .generic_string(),
                method->name(), computation_time_ms, prepared.initial_score);
            print_json_property(
                out, "initial_pc_num",
                std::views::transform(
                    instance.cases(), [&prepared](const auto & instance_case) {
                        return std::make_pair(
                            instance_case.name(),
                            prepared.initial_cases_pc_num[instance_case.id()]);
                    }));
            out.print(
                ",\n    \"budget\":         {},\n    \"bound\":          {},",
                budget, bound);
            if(all_options_bound)
                out.print("\n    \"all_options_bound\": {},",
                          all_options_bound.value());
            out.print("\n    \"method_report\": {{\n        \"status\": \"{}\"",
                      report.status);
            for(const auto & [key, value] : report.stats)
                out.print(",\n        \"{}\": {}", key, value);
            out.print("\n    }}\n}}");

            spdlog::info("Bound printed to '{}'",
                         std::filesystem::absolute(opt_output_json_file.value())
                             .string());
        }
    } catch(const std::exception & e) {
        spdlog::error("{}: {}", program_state, e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
