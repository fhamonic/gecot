#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>

#include "gecot/bounds/bound_report.hpp"

#include "instance.hpp"

// what compute needs to read its parameters in the instance's unit
struct bound_context {
    double score_scale;  // a raw score is score_scale times the instance's
};

class AbstractBoundInterface {
protected:
    CLI::App app;

    // a raw score: an infinite one would make the steps NaN
    void add_target_option(std::optional<double> & target,
                           const std::string & description) {
        app.add_option("--target", target, description)
            ->check(CLI::Validator(
                [](std::string & s) -> std::string {
                    try {
                        std::size_t end = 0;
                        if(std::isfinite(std::stod(s, &end)) && end == s.size())
                            return {};
                    } catch(const std::exception &) {
                    }
                    return "'" + s + "' is not a finite number";
                },
                "FINITE"));
    }

public:
    explicit AbstractBoundInterface(const std::string & name) : app("", name) {
        app.set_help_flag();
    }
    virtual ~AbstractBoundInterface() = default;

    // args in reverse order, as CLI::App::parse consumes them
    virtual void parse(std::vector<std::string> args) { app.parse(args); }
    std::string name() const { return app.get_name(); }
    virtual std::string options_description() const { return app.help(); }

    // the bound and score_stats in the unit of instance
    virtual gecot::bound_report compute(
        const Instance & instance, const double B,
        const bound_context & context) const = 0;
    virtual std::string description() const = 0;
};
