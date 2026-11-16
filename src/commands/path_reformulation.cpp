#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <sstream>
#include <utility>

#include <spdlog/spdlog.h>

#include "gecot/helper.hpp"
#include "gecot/utils/progress.hpp"

#include "commands/common.hpp"

#include "solver_interfaces/path_lp_interface.hpp"
#include "trivial_reformulate.hpp"

namespace {
// the options are ruled out against the MIP start
struct PathReformulationInterface : PathLpInterface {
    PathReformulationInterface() {
        app.get_option("--mip-start-algorithm")->default_val("greedy");
    }
    // With a start file, its parameters read mip-start-algorithm none: greedy
    // does not run, as with_mip_start keeps a start already set.
    void parse(std::vector<std::string> args) {
        PathLpInterface::parse(std::move(args));
        if(app.get_option("--mip-start")->count())
            app.get_option("--mip-start-algorithm")->default_val("none");
    }
};
}  // namespace

std::shared_ptr<const PathLpInterface> make_path_reformulation(
    const std::string & parameters) {
    auto path_lp = std::make_shared<PathReformulationInterface>();
    std::vector<std::string> args;
    std::istringstream words(parameters);
    for(std::string word; words >> word;) args.push_back("--" + word);
    std::ranges::reverse(args);
    path_lp->parse(std::move(args));
    return path_lp;
}

gecot::solve_result<Instance> solve_path_reformulated(
    const PathLpInterface & path_lp, AbstractSolverInterface & solver,
    const Instance & instance, const gecot::budget_t & budget,
    const bool merge) {
    auto score = [&](const auto & X) {
        return instance.eval_criterion(
            gecot::compute_solution_cases_pc_num(instance, X));
    };
    const auto parameters = path_lp.parameters();
    const std::string & time_limit =
        std::ranges::find(parameters, std::string("time-limit"),
                          &std::pair<std::string, std::string>::first)
            ->second;
    spdlog::info("Reformulating with path_lp, {}",
                 time_limit == "none" ? "without time limit"
                                      : "time limit " + time_limit + " s");
    log_parameters(parameters);
    auto [solution, report, option_bounds] = path_lp.relax(instance, budget);
    const double solution_score = score(solution);
    // an option is kept if a solution that buys it can beat this one by more
    // than 1e-9 of its score, ten times gecot::mip_gap
    const double cutoff = solution_score + 1e-9 * std::abs(solution_score);
    auto keep = [&](const gecot::option_t o) {
        return option_bounds[o] > cutoff;
    };
    const auto num_kept = std::ranges::count_if(instance.options(), keep);
    spdlog::info("The path relaxation finds {}, leaves {} of {} options",
                 gecot::gain_text(solution_score), num_kept,
                 instance.num_options());
    for(auto & [name, value] : report.stats) name = "reformulation_" + name;
    for(auto & [name, value] : report.score_stats)
        name = "reformulation_" + name;
    report.stats.emplace_back("remaining_options", num_kept);
    const double path_bound =
        report.best_bound.value_or(std::numeric_limits<double>::infinity());
    if(num_kept == 0 || path_bound <= cutoff) {
        spdlog::info(
            "{}: '{}' is not run",
            num_kept == 0 ? "No option left" : "Its solution is optimal",
            solver.name());
        report.status = "optimal";
        report.best_bound = std::min(path_bound, cutoff);
        return {std::move(solution), std::move(report)};
    }
    if(gecot::stop_requested) {
        spdlog::info("Interrupted: '{}' is not run", solver.name());
        return {std::move(solution), std::move(report)};
    }

    const Instance restricted_instance =
        restrict_instance(instance, keep, merge);
    std::vector<std::string> start;
    for(auto && o : instance.options())
        if(solution[o]) start.push_back(instance.option_name(o));
    spdlog::info("Running algorithm '{}' on the options left", solver.name());
    if(solver.set_start(std::move(start)))
        spdlog::info("MIP start: the path relaxation's solution");
    const auto [restricted_solution, solver_report] =
        solver.solve(restricted_instance, budget);
    auto solver_solution = instance.create_option_map(false);
    for(auto && o : restricted_instance.options())
        if(restricted_solution[o])
            solver_solution[instance.option_from_name(
                restricted_instance.option_name(o))] = true;
    if(score(solver_solution) > solution_score)
        solution = std::move(solver_solution);

    // the optimum is the better of the path relaxation's solution and of
    // the optimum on the options left
    report.status = solver_report.status;
    if(const double bound = std::min(
           path_bound,
           std::max(cutoff, solver_report.best_bound.value_or(path_bound)));
       std::isfinite(bound))
        report.best_bound = bound;
    else
        report.best_bound.reset();
    report.stats.append_range(solver_report.stats);
    report.score_stats.append_range(solver_report.score_stats);
    return {std::move(solution), std::move(report)};
}
