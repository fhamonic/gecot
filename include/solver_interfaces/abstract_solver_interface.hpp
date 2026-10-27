#pragma once

#include <string>
#include <vector>

#include <CLI/CLI.hpp>

#include "gecot/solvers/solve_result.hpp"

#include "instance.hpp"

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

    virtual gecot::solve_result<Instance> solve(const Instance & instance,
                                                const double B) const = 0;
    virtual std::string description() const = 0;
};
