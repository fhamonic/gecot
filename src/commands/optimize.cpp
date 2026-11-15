#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <optional>
#include <print>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include <CLI/CLI.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "gecot/helper.hpp"

#include "commands/commands.hpp"
#include "commands/common.hpp"
#include "commands/run_command.hpp"

#include "instance.hpp"

#include "solver_interfaces/abstract_solver_interface.hpp"

namespace {

// what the command line of 'gecot optimize' holds
struct optimize_arguments {
    run_arguments run;
    std::optional<std::filesystem::path> output_csv_file;
    std::shared_ptr<const PathLpInterface> path_reformulation;
    bool no_merging = false;
    // the options of the command that change the instance of the algorithm,
    // with their JSON values
    std::vector<std::pair<std::string, std::string>> command_parameters;
};

// the algorithm chosen, with its arguments parsed, or null if the command
// line only asked for a help or a list
std::shared_ptr<AbstractSolverInterface> parse_arguments(
    const std::vector<std::string> & args, optimize_arguments & arguments) {
    std::string reformulation = "trivial", reformulation_params;
    CLI::App app("", "gecot optimize");
    auto solver = parse_run_arguments(
        app, args,
        {.command = "optimize",
         .noun = "algorithm",
         .nouns = "algorithms",
         .list_flag = "-A,--list-algorithms",
         .list_help = "List the available algorithms",
         .name_option = "algorithm,-a,--algorithm",
         .name_help = "Algorithm to use",
         .output_help = "Output solution data in JSON file"},
        make_solver_interfaces(), arguments.run,
        [&](CLI::App & cli) {
            cli.add_option("--output-csv", arguments.output_csv_file,
                           "Output solution value in CSV file");
            cli.add_option(
                   "--reformulation", reformulation,
                   "Instance given to the algorithm, trivial: without the "
                   "options over the budget, and those of cost 0, bought "
                   "unless a conflict set makes them a choice; path_lp: also "
                   "without the options that the path relaxation proves to be "
                   "in no better solution than its own")
                ->check(CLI::IsMember({"trivial", "path_lp"}))
                ->default_val(reformulation);
            cli.add_option(
                "--reformulation-params", reformulation_params,
                "Parameters of the path_lp of --reformulation path_lp, as "
                "\"time-limit=60 pricing=enumerate\"; its MIP start is "
                "greedy's solution by default");
            cli.add_flag(
                "--no-merging", arguments.no_merging,
                "For comparison: keeps apart the vertices that arcs of "
                "probability 1 link, merged in the instance given to the "
                "algorithm");
        },
        [](const std::string &) {});
    if(!solver) return nullptr;

    if(reformulation == "path_lp")
        arguments.path_reformulation =
            make_path_reformulation(reformulation_params);
    else if(!reformulation_params.empty())
        throw std::logic_error(
            "'--reformulation-params' requires '--reformulation path_lp'");
    for(const char * name :
        {"--reformulation", "--reformulation-params", "--no-merging"})
        arguments.command_parameters.emplace_back(
            name + 2, option_json_value(*app.get_option(name)));
    return solver;
}

// What 'gecot optimize' reports of a solution, in the scores of the raw
// instance.
struct optimize_outcome {
    std::string status;
    // per option of the raw instance: 1 if it is bought, statically or by
    // the algorithm
    std::vector<int> option_values;
    double score;
    // per case of the raw instance
    decltype(prepared_instance::initial_cases_pc_num) cases_pc_num;
    std::vector<double> cost;  // per cost dimension
    bool feasible;
    // the bound to show: the algorithm's, not under the score of a feasible
    // solution
    std::optional<double> best_bound;
    // in %: the share of the best possible gain that a feasible solution is
    // thereby proved to reach
    std::optional<double> guaranteed_share;
    // of the algorithm's report, then its bound as it gave it and the gap
    std::vector<std::pair<std::string, double>> stats;
    long long computation_time_ms;
};

// The verdict "feasible" of a result: every cost is within its budget, up to
// budget_margin relative to the budget, or to 1 under a smaller one, and at
// most one option of each conflict set of the raw instance is bought, which
// also checks the reformulation. Each breach is warned of. The margin does
// not follow -t: after a run with a larger -t, a selection that the algorithm
// accepted is warned of here.
constexpr double budget_margin = 1e-6;
bool check_feasibility(const prepared_instance & prepared,
                       const std::vector<double> & cost,
                       const std::vector<int> & option_values) {
    const Instance & raw_instance = prepared.raw_instance;
    const gecot::budget_t & budget = prepared.budget;
    bool feasible = true;
    for(std::size_t r = 0; r < budget.size(); ++r) {
        if(cost[r] <= budget[r] + budget_margin * std::max(1.0, budget[r]))
            continue;
        feasible = false;
        if(budget.size() == 1)
            spdlog::warn("The solution costs {:.10g}, over the budget {:.10g}",
                         cost[r], budget[r]);
        else
            spdlog::warn(
                "The solution costs {:.10g} in {}, over the budget {:.10g}",
                cost[r], raw_instance.cost_dimension_name(r), budget[r]);
    }
    for(std::size_t l = 0; l < raw_instance.conflict_sets().size(); ++l) {
        const auto bought = std::ranges::count_if(
            raw_instance.conflict_sets()[l],
            [&](const gecot::option_t i) { return option_values[i]; });
        if(bought > 1) {
            feasible = false;
            spdlog::warn(
                "The solution buys {} options of the conflict set '{}'", bought,
                raw_instance.conflict_set_name(l));
        }
    }
    return feasible;
}

optimize_outcome evaluate(const prepared_instance & prepared,
                          gecot::solve_result<Instance> result,
                          const long long computation_time_ms) {
    const Instance & raw_instance = prepared.raw_instance;
    const Instance & instance = prepared.instance;
    // the algorithm's scores and bounds are in this unit
    const double score_scale = prepared.score_scale;
    auto & [solution, report] = result;

    std::vector<int> option_values;
    for(const gecot::option_t i : raw_instance.options()) {
        const auto & option_name = raw_instance.option_name(i);
        option_values.push_back(
            prepared.static_options[i] ||
            (instance.contains_option(option_name) &&
             solution[instance.option_from_name(option_name)]));
    }
    // Per case of the raw instance. A case that the criterion reads takes
    // its score from the instance of the algorithm, by its name: it has
    // another id there as soon as a case before it is left out. The options
    // bought can improve the cases left out too: these are scored on the raw
    // instance.
    auto cases_pc_num = raw_instance.create_case_map<double>();
    if(instance.cases().size() < raw_instance.cases().size())
        cases_pc_num = gecot::compute_solution_cases_pc_num(
            raw_instance, melon::maps::function([&](const gecot::option_t i) {
                return option_values[i] != 0;
            }));
    const auto read_cases_pc_num =
        gecot::compute_solution_cases_pc_num(instance, solution);
    for(const auto & instance_case : instance.cases())
        cases_pc_num[raw_instance.case_id_from_name(instance_case.name())] =
            read_cases_pc_num[instance_case.id()] * score_scale;
    const double score = raw_instance.eval_criterion(cases_pc_num);
    check_finite_score(score, "the score of the solution");
    std::vector<double> cost;
    for(std::size_t r = 0; r < prepared.budget.size(); ++r)
        cost.push_back(gecot::compute_solution_cost(instance, solution, r));
    for(const auto & [key, value] : report.score_stats)
        report.stats.emplace_back(key, value * score_scale);
    const bool feasible = check_feasibility(prepared, cost, option_values);
    std::optional<double> best_bound;
    if(report.best_bound) {
        const double bound = report.best_bound.value() * score_scale;
        report.stats.emplace_back("best_bound", bound);
        if(score > 0) report.stats.emplace_back("gap", (bound - score) / score);
        // Shown as the score, a bound further under it would read as a
        // proof of optimality. An infeasible selection is no proof of its
        // own score.
        if(!feasible)
            best_bound = bound;
        else if(bound >= score - bound_margin * std::max(1.0, std::abs(score)))
            best_bound = std::max(bound, score);
        else
            spdlog::warn(
                "The bound {} of the algorithm is below the score {} of its "
                "solution: no bound is shown, nor any share guaranteed",
                bound, score);
    }
    return {.status = std::move(report.status),
            .option_values = std::move(option_values),
            .score = score,
            .cases_pc_num = std::move(cases_pc_num),
            .cost = std::move(cost),
            .feasible = feasible,
            .best_bound = best_bound,
            .guaranteed_share =
                feasible && best_bound
                    ? guaranteed_gain_share(score, best_bound.value(),
                                            prepared.base_score)
                    : std::nullopt,
            .stats = std::move(report.stats),
            .computation_time_ms = computation_time_ms};
}

void print_summary(const prepared_instance & prepared,
                   const optimize_outcome & outcome) {
    const Instance & raw_instance = prepared.raw_instance;
    const gecot::budget_t & budget = prepared.budget;
    std::println("Status          {}", outcome.status);
    std::println("Initial score   {}", format_score(prepared.initial_score));
    std::println("Solution score  {} ({})", format_score(outcome.score),
                 format_gain(outcome.score, prepared.initial_score));
    if(outcome.best_bound)
        std::println(
            "Upper bound     {}{}", format_score(outcome.best_bound.value()),
            outcome.guaranteed_share
                .transform([](double share) {
                    return std::format(
                        " (at least {:.1f} % of the best possible gain)",
                        share);
                })
                .value_or(""));
    for(std::size_t r = 0; r < budget.size(); ++r)
        if(budget.size() == 1)
            std::println("Cost            {:.10g} of {:.10g}", outcome.cost[r],
                         budget[r]);
        else
            std::println("Cost {:<10} {:.10g} of {:.10g}",
                         raw_instance.cost_dimension_name(r), outcome.cost[r],
                         budget[r]);
    std::println(
        "Time            {}",
        format_duration(static_cast<double>(outcome.computation_time_ms) /
                        1000.0));
    if(raw_instance.cases().size() > 1) {
        std::println("Per species");
        const std::size_t name_max_length = std::ranges::max(
            std::views::transform(raw_instance.cases(),
                                  [](auto && c) { return c.name().size(); }));
        for(const auto & instance_case : raw_instance.cases())
            std::println(
                "    {:<{}} {} ({})", instance_case.name() + ':',
                name_max_length + 1,
                format_score(outcome.cases_pc_num[instance_case.id()]),
                format_gain(outcome.cases_pc_num[instance_case.id()],
                            prepared.initial_cases_pc_num[instance_case.id()]));
    }
    const auto bought = std::ranges::to<std::vector>(std::views::filter(
        raw_instance.options(),
        [&](const gecot::option_t i) { return outcome.option_values[i]; }));
    std::println("Options bought  {} of {}{}", bought.size(),
                 raw_instance.num_options(), bought.empty() ? "" : ":");
    for(auto && i : bought) std::println("    {}", raw_instance.option_name(i));
}

void write_json(std::FILE * stream, const optimize_arguments & arguments,
                const AbstractSolverInterface & solver,
                const prepared_instance & prepared,
                const optimize_outcome & outcome) {
    const Instance & raw_instance = prepared.raw_instance;
    print_json_head(stream, arguments.run.instance_file, "algorithm",
                    solver.name(), solver.json_parameters());
    std::print(stream, ",");
    print_json_property(stream, "command_parameters",
                        arguments.command_parameters);
    print_json_instance(stream, prepared, outcome.computation_time_ms);
    std::print(stream, ",\n    \"solution_score\": {},", outcome.score);
    print_json_property(
        stream, "solution_pc_num",
        std::views::transform(
            raw_instance.cases(), [&outcome](const auto & instance_case) {
                return std::make_pair(instance_case.name(),
                                      outcome.cases_pc_num[instance_case.id()]);
            }));
    std::print(stream, ",");
    print_json_per_dimension(stream, "solution_cost",
                             "\"solution_cost\":  ", raw_instance,
                             outcome.cost);
    std::print(stream, ",\n    \"feasible\":       {},", outcome.feasible);
    if(outcome.guaranteed_share)
        std::print(stream, "\n    \"guaranteed_share\": {},",
                   outcome.guaranteed_share.value());
    print_json_property(stream, "solution",
                        std::views::transform(
                            raw_instance.options(),
                            [&raw_instance, &outcome](const gecot::option_t i) {
                                return std::make_pair(
                                    raw_instance.option_name(i),
                                    outcome.option_values[i]);
                            }));
    std::print(stream,
               ",\n    \"algorithm_report\": {{\n        \"status\": {}",
               json_string(outcome.status));
    // JSON has no infinity nor NaN
    for(const auto & [key, value] : outcome.stats)
        if(std::isfinite(value))
            std::print(stream, ",\n        \"{}\": {}", key, value);
    std::print(stream, "\n    }}\n}}");
}

void write_csv(std::FILE * stream, const prepared_instance & prepared,
               const optimize_outcome & outcome) {
    std::print(stream, "option_id,value\n");
    for(const gecot::option_t i : prepared.raw_instance.options())
        std::print(stream, "{},{}\n",
                   csv_field(prepared.raw_instance.option_name(i)),
                   outcome.option_values[i]);
}

}  // namespace

int optimize_command(const std::vector<std::string> & args) {
    std::string program_state = "Parsing arguments";
    try {
        optimize_arguments arguments;
        const auto solver = parse_arguments(args, arguments);
        if(!solver) return EXIT_SUCCESS;
        const auto & output_json_file = arguments.run.output_json_file;
        const auto & output_csv_file = arguments.output_csv_file;
        program_state = "Checking output files";
        check_output_directories({output_json_file, output_csv_file});
        spdlog::info("GECOT {}, algorithm {}", PROJECT_VERSION, solver->name());
        log_parameters(solver->parameters());
        program_state = "Parsing instance";
        const prepared_instance prepared =
            prepare_instance(arguments.run.instance_file, arguments.run.budget,
                             "algorithm", !arguments.no_merging);
        const Instance & instance = prepared.instance;

        program_state = "Running algorithm '" + solver->name() + '\'';
        spdlog::stopwatch sw;
        auto result = [&] {
            const progress_watch watch(prepared);
            return instance.num_options() == 0
                       ? gecot::solve_result<Instance>{instance
                                                           .create_option_map(
                                                               false),
                                                       {"no option left"}}
                   : arguments.path_reformulation
                       ? solve_path_reformulated(
                             *arguments.path_reformulation, *solver, instance,
                             prepared.budget, !arguments.no_merging)
                       : solver->solve(instance, prepared.budget);
        }();
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();

        program_state = "Printing solution";
        const optimize_outcome outcome =
            evaluate(prepared, std::move(result), computation_time_ms);
        if(!output_json_file && !output_csv_file)
            print_summary(prepared, outcome);
        if(output_json_file) {
            const auto file = open_output_file(
                std::filesystem::absolute(output_json_file.value()));
            write_json(file.get(), arguments, *solver, prepared, outcome);
            spdlog::info(
                "Solution written to {}",
                std::filesystem::absolute(output_json_file.value()).string());
        }
        if(output_csv_file) {
            const auto file = open_output_file(output_csv_file.value());
            write_csv(file.get(), prepared, outcome);
            spdlog::info(
                "Solution written to {}",
                std::filesystem::absolute(output_csv_file.value()).string());
        }
    } catch(const std::exception & e) {
        spdlog::error("{}: {}", program_state, e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
