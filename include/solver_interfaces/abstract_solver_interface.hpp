#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "gecot/solvers/solve_result.hpp"

#include "instance.hpp"

// the long name and the value of each option of app, its default if not
// given, "none" without one
inline std::vector<std::pair<std::string, std::string>> option_values(
    const CLI::App & app) {
    std::vector<std::pair<std::string, std::string>> values;
    for(const CLI::Option * option : app.get_options()) {
        if(option->get_lnames().empty()) continue;
        std::string value;
        if(option->get_expected_max() == 0)  // a flag
            value = option->count() && option->as<bool>() ? "true" : "false";
        else if(option->count())
            value = option->results() | std::views::join_with(' ') |
                    std::ranges::to<std::string>();
        else
            value = option->get_default_str();
        values.emplace_back(option->get_lnames().front(),
                            value.empty() ? "none" : value);
    }
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
    // a path can hold bytes that are not UTF-8, which dump() throws on
    return nlohmann::json(option.as<std::string>())
        .dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
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

class AbstractSolverInterface {
protected:
    CLI::App app;

    void add_feasibility_tol_option(double & feasibility_tol,
                                    double default_tol = 1e-7) {
        app.add_option("-t,--feasibility-tolerance", feasibility_tol,
                       "Tolerance for rounding errors")
            ->default_val(default_tol);
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

    // the options of a solution that the algorithm may start from
    virtual void set_start(std::vector<std::string>) {}
    virtual gecot::solve_result<Instance> solve(const Instance & instance,
                                                const double B) const = 0;
    virtual std::string description() const = 0;
};
