#pragma once

#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <CLI/CLI.hpp>

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
            value = option->count() ? "true" : "false";
        else if(option->count())
            value = fmt::format("{}", fmt::join(option->results(), " "));
        else
            value = option->get_default_str();
        values.emplace_back(option->get_lnames().front(),
                            value.empty() ? "none" : value);
    }
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

    virtual gecot::solve_result<Instance> solve(const Instance & instance,
                                                const double B) const = 0;
    virtual std::string description() const = 0;
};
