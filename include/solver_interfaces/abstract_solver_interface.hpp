#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "gecot/solvers/solve_result.hpp"

#include "instance.hpp"
#include "io_helper.hpp"

// the value of option, its default if not given, "none" without one
inline std::string option_value(const CLI::Option & option) {
    std::string value;
    if(option.get_expected_max() == 0)  // a flag
        value = option.count() && option.as<bool>() ? "true" : "false";
    else if(option.count())
        value = option.results() | std::views::join_with(' ') |
                std::ranges::to<std::string>();
    else
        value = option.get_default_str();
    return value.empty() ? "none" : value;
}

// the long name and the option_value of each option of app
inline std::vector<std::pair<std::string, std::string>> option_values(
    const CLI::App & app) {
    std::vector<std::pair<std::string, std::string>> values;
    for(const CLI::Option * option : app.get_options())
        if(!option->get_lnames().empty())
            values.emplace_back(option->get_lnames().front(),
                                option_value(*option));
    return values;
}

// The value of option, its default if not given, as a JSON value: a boolean
// for a flag, a number for a numeric option, null without a value, else a
// string. The type is the option's, not the text's: '--preprocessing none'
// is a string and a MIP start file named 7 too.
inline std::string option_json_value(const CLI::Option & option) {
    const std::string type = option.get_type_name();
    if(option.get_expected_max() == 0)  // a flag
        return option.count() && option.as<bool>() ? "true" : "false";
    if(!option.count() && option.get_default_str().empty()) return "null";
    // JSON has no infinity: '-t inf' stays a text
    if(type.starts_with("FLOAT") && std::isfinite(option.as<double>()))
        return std::format("{}", option.as<double>());
    if(type.starts_with("UINT"))
        return std::format("{}", option.as<std::uint64_t>());
    if(type.starts_with("INT"))
        return std::format("{}", option.as<std::int64_t>());
    return json_string(option.as<std::string>());
}

// the long name and the option_json_value of each option of app
inline std::vector<std::pair<std::string, std::string>> option_json_values(
    const CLI::App & app) {
    std::vector<std::pair<std::string, std::string>> values;
    for(const CLI::Option * option : app.get_options())
        if(!option->get_lnames().empty())
            values.emplace_back(option->get_lnames().front(),
                                option_json_value(*option));
    return values;
}

// An option that takes one of the names of choices. The names are its
// validator, in their order, and its default text is the name of current,
// the value that the struct holds: the default is written once, there. set
// receives the value of the name given. Throws if current has no name.
template <typename V, typename Set>
CLI::Option * add_choice_option(CLI::App & app, const std::string & name,
                                const V & current,
                                std::vector<std::pair<std::string, V>> choices,
                                Set set, const std::string & help) {
    const auto current_choice =
        std::ranges::find(choices, current, &std::pair<std::string, V>::second);
    if(current_choice == choices.end())
        throw std::logic_error(name + ": its default has no name");
    const std::string default_name = current_choice->first;
    const auto names = std::ranges::to<std::vector>(std::views::keys(choices));
    return app
        .add_option_function<std::string>(
            name,
            [choices = std::move(choices),
             set = std::move(set)](const std::string & given) {
                set(std::ranges::find(choices, given,
                                      &std::pair<std::string, V>::first)
                        ->second);
            },
            help)
        ->check(CLI::IsMember(names))
        ->default_val(default_name);
}

class AbstractSolverInterface {
protected:
    CLI::App app;

    void add_feasibility_tol_option(double & feasibility_tol) {
        app.add_option("-t,--feasibility-tolerance", feasibility_tol,
                       "Tolerance for rounding errors")
            ->default_val(feasibility_tol);
    }

public:
    explicit AbstractSolverInterface(const std::string & name) : app("", name) {
        app.set_help_flag();
    }
    virtual ~AbstractSolverInterface() = default;

    // args in reverse order, as CLI::App::parse consumes them
    void parse(std::vector<std::string> args) { app.parse(args); }
    std::string name() const { return app.get_name(); }
    std::string options_description() const { return app.help(); }
    std::vector<std::pair<std::string, std::string>> parameters() const {
        return option_values(app);
    }
    std::vector<std::pair<std::string, std::string>> json_parameters() const {
        return option_json_values(app);
    }

    // the options of a solution that the algorithm may start from; false if
    // it does not
    virtual bool set_start(std::vector<std::string>) { return false; }
    virtual gecot::solve_result<Instance> solve(
        const Instance & instance, const gecot::budget_t & B) const = 0;
    virtual std::string description() const = 0;
    // what its run proves as an upper bound, for 'gecot bound'; nothing if
    // it proves none
    virtual std::optional<std::string> bound_description() const {
        return std::nullopt;
    }
};
