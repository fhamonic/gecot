#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/core.h>

#include "gecot/helper.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"
#include "solver_interfaces/abstract_solver_interface.hpp"

// the bound an optimization algorithm proves, with its parameters
class AlgorithmBoundInterface : public AbstractBoundInterface {
private:
    std::shared_ptr<AbstractSolverInterface> solver;
    std::string _description;

public:
    AlgorithmBoundInterface(std::shared_ptr<AbstractSolverInterface> t_solver,
                            std::string description)
        : AbstractBoundInterface(t_solver->name())
        , solver(std::move(t_solver))
        , _description(std::move(description)) {}

    void parse(std::vector<std::string> args) override {
        solver->parse(std::move(args));
    }
    std::string options_description() const override {
        return solver->options_description();
    }

    gecot::bound_report compute(const Instance & instance, const double B,
                                const bound_context &) const override {
        auto [solution, report] = solver->solve(instance, B);
        if(!report.best_bound)
            throw std::runtime_error(fmt::format(
                "'{}' proved no bound (status: {})", name(), report.status));
        const double solution_score = instance.eval_criterion(
            gecot::compute_solution_cases_pc_num(instance, solution));
        // the solver's bound and the recomputed score differ in the last
        // bits at the optimum
        if(report.best_bound.value() <
           solution_score - 1e-9 * std::max(1.0, std::abs(solution_score)))
            throw std::runtime_error(fmt::format(
                "'{}' bounds the score by {}, below its solution's {}", name(),
                report.best_bound.value(), solution_score));
        gecot::bound_report bound_report{
            std::max(report.best_bound.value(), solution_score),
            std::move(report.status), std::move(report.stats),
            std::move(report.score_stats)};
        bound_report.score_stats.emplace_back("solution_score", solution_score);
        return bound_report;
    }

    std::string description() const override { return _description; }
};
