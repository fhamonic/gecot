#ifndef COMMANDS_RUN_COMMAND_HPP
#define COMMANDS_RUN_COMMAND_HPP

#include <algorithm>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <print>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>

#include <tbb/global_control.h>

#include "commands/common.hpp"

#include "io_helper.hpp"
#include "parse_instance.hpp"

// The words by which the command lines of 'gecot optimize' and 'gecot bound'
// differ.
struct run_command_words {
    std::string command;    // optimize
    std::string noun;       // algorithm
    std::string nouns;      // algorithms
    std::string list_flag;  // -A,--list-algorithms
    std::string list_help;
    std::string name_option;  // algorithm,-a,--algorithm
    std::string name_help;
    std::string output_help;  // of -o
};

// What both commands read on their command line.
struct run_arguments {
    std::filesystem::path instance_file;
    std::string budget;  // as given: parsed with the instance
    std::optional<std::filesystem::path> output_json_file;
    std::optional<tbb::global_control> thread_limit;
};

// Parses the arguments of a command that runs one of interfaces on an
// instance and a budget, and gives the one chosen its own arguments. Returns
// it, or null if the arguments only asked for a help or a list, printed here.
// add_options(app) declares the options of the command itself: the help
// lists them after -o. unknown(name) is called for a name that no interface
// has, before the names are listed and the error thrown: it can throw
// another one.
template <typename Interface>
std::shared_ptr<Interface> parse_run_arguments(
    CLI::App & app, const std::vector<std::string> & args,
    const run_command_words & words,
    std::vector<std::shared_ptr<Interface>> interfaces, run_arguments & run,
    auto && add_options, auto && unknown) {
    auto print_available = [&] {
        std::println("Available {}:", words.nouns);
        const std::size_t name_max_length =
            std::ranges::max(std::views::transform(
                interfaces, [](auto && i) { return i->name().size(); }));
        const std::size_t offset = name_max_length + 8;
        for(auto & i : interfaces) {
            std::print("  {:<{}}", i->name(), offset - 2);
            print_paragraph(offset, 80 - offset, i->description());
        }
        std::println("");
    };

    bool help = false, list = false, list_params = false, quiet = false;
    int verbosity = 0;
    std::string name;
    std::optional<std::size_t> max_threads;
    app.usage(std::format(
        "Usage:\n  gecot {0} --help\n  gecot {0} --list-{2}\n  gecot {0} "
        "<{1}> --list-params\n  gecot {0} <{1}> <instance> <budget> "
        "[<parameters> ...]",
        words.command, words.noun, words.nouns));
    app.set_help_flag();
    app.add_flag("-h,--help", help, "Display this help message");
    app.add_flag(words.list_flag, list, words.list_help);
    app.add_flag("-P,--list-params", list_params,
                 "List the parameters of the chosen " + words.noun);
    // a name without dashes also makes the option positional
    auto * name_opt = app.add_option(words.name_option, name, words.name_help);
    auto * instance_opt = app.add_option(
        "instance,-i,--instance", run.instance_file, "Instance JSON file");
    auto * budget_opt =
        app.add_option("budget,-B,--budget", run.budget,
                       "Budget value, or name=value,... for options of "
                       "several costs")
            ->type_name("BUDGET");
    app.add_option("-o,--output-json", run.output_json_file, words.output_help);
    add_options(app);
    app.add_flag("-v,--verbose", verbosity,
                 "Log the " + words.nouns +
                     "' details, -vv everything with the solver's own log "
                     "on stdout");
    app.add_flag("-q,--quiet", quiet, "Silence all logging except errors");
    app.add_option("--max-threads", max_threads,
                   "Limits the number of threads of GECOT's own computations, "
                   "not those of the solver")
        ->check(CLI::Range(std::size_t{1},
                           std::numeric_limits<std::size_t>::max(),
                           "POSITIVE"));
    // unknown options and surplus values are left for the interface
    app.allow_extras();
    // else on Windows CLI11 takes a '/abs/path' instance for an option
    app.allow_windows_style_options(false);
    // CLI::App::parse consumes the args from the back
    app.parse(std::vector<std::string>(args.rbegin(), args.rend()));

    if(help || args.empty()) {
        print_soft_name();
        std::cout << app.help() << std::endl;
        return nullptr;
    }
    if(list) {
        print_soft_name();
        print_available();
        return nullptr;
    }
    std::shared_ptr<Interface> chosen;
    if(name_opt->count()) {
        const auto it = std::ranges::find(interfaces, name,
                                          [](auto && i) { return i->name(); });
        if(it == interfaces.end()) {
            unknown(name);
            print_available();
            throw std::logic_error("unknown " + words.noun + " '" + name + "'");
        }
        chosen = *it;
    }
    if(list_params) {
        if(!name_opt->count())
            throw std::logic_error(
                "Option '--list-params' requires option '--" + words.noun +
                "'.");
        print_soft_name();
        std::cout << chosen->options_description();
        std::cout << std::endl;
        return nullptr;
    }

    for(auto * opt : {name_opt, instance_opt, budget_opt})
        if(!opt->count())
            throw std::logic_error("the option '" + opt->get_name() +
                                   "' is required but missing");
    // the names are checked with the instance
    if(!run.budget.contains('=') && !parse_budget_value(run.budget))
        throw std::logic_error("the budget must be a non-negative number");
    apply_logging(verbosity, quiet);
    if(max_threads.has_value())
        run.thread_limit.emplace(tbb::global_control::max_allowed_parallelism,
                                 max_threads.value());
    // remaining() is in command line order: the interface would read
    // '-t 1e-5' as '1e-5 -t'
    chosen->parse(app.remaining_for_passthrough());
    return chosen;
}

#endif  // COMMANDS_RUN_COMMAND_HPP
