#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <optional>
#include <print>
#include <string>
#include <utility>
#include <vector>

#include <CLI/CLI.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "gecot/bounds/all_options.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/helper.hpp"

#include "commands/commands.hpp"
#include "commands/common.hpp"
#include "commands/run_command.hpp"

#include "instance.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"

namespace {

// the method chosen, with its arguments parsed, or null if the command line
// only asked for a help or a list
std::shared_ptr<AbstractBoundInterface> parse_arguments(
    const std::vector<std::string> & args, run_arguments & run) {
    CLI::App app("", "gecot bound");
    return parse_run_arguments(
        app, args,
        {.command = "bound",
         .noun = "method",
         .nouns = "methods",
         .list_flag = "-M,--list-methods",
         .list_help = "List the available bound methods",
         .name_option = "method,--method",
         .name_help = "Bound method to use",
         .output_help = "Output bound data in JSON file"},
        make_bound_interfaces(), run, [](CLI::App &) {},
        [](const std::string & name) {
            if(std::ranges::contains(make_solver_interfaces(), name,
                                     [](auto && s) { return s->name(); }))
                throw std::logic_error(
                    "'" + name +
                    "' proves no bound, see 'gecot bound --list-methods'");
        });
}

// What 'gecot bound' reports, in the scores of the raw instance.
struct bound_outcome {
    std::string status;
    double bound;
    // for a criterion that all_options accepts
    std::optional<double> all_options_bound;
    std::vector<std::pair<std::string, double>> stats;
    long long computation_time_ms;
};

// throws if the bound is below the score of buying only the options bought
// statically, which is a solution
void check_bound(const prepared_instance & prepared,
                 const gecot::bound_report & report) {
    const double score_scale = prepared.score_scale;
    // the solutions are those of 'gecot optimize', the options bought
    // statically bought: buying no other is one
    const double base_score = prepared.base_score / score_scale;
    const double bound = report.bound * score_scale;
    if(bound < base_score * score_scale -
                   1e-9 * std::max(1.0, std::abs(base_score * score_scale)))
        throw std::runtime_error(std::format(
            "the bound {} is below the score {} of buying only the options "
            "bought statically",
            bound, base_score * score_scale));
}

bound_outcome evaluate(const prepared_instance & prepared,
                       gecot::bound_report report,
                       const long long computation_time_ms) {
    const double score_scale = prepared.score_scale;
    const double bound = report.bound * score_scale;
    check_finite_score(bound, "the bound");
    std::optional<double> all_options_bound;
    if(gecot::has_non_decreasing_criterion(prepared.instance)) {
        all_options_bound =
            gecot::all_options_bound(prepared.instance).bound * score_scale;
        check_finite_score(all_options_bound.value(), "the all options bound");
    }
    for(const auto & [key, value] : report.score_stats)
        report.stats.emplace_back(key, value * score_scale);
    return {.status = std::move(report.status),
            .bound = bound,
            .all_options_bound = all_options_bound,
            .stats = std::move(report.stats),
            .computation_time_ms = computation_time_ms};
}

void print_summary(const prepared_instance & prepared,
                   const bound_outcome & outcome) {
    const gecot::budget_t & budget = prepared.budget;
    std::println("Status             {}", outcome.status);
    std::println("Initial score      {}", format_score(prepared.initial_score));
    std::println("Upper bound        {} (at most {})",
                 format_score(outcome.bound),
                 format_gain(outcome.bound, prepared.initial_score));
    if(outcome.all_options_bound)
        std::println("All options bound  {}",
                     format_score(outcome.all_options_bound.value()));
    for(std::size_t r = 0; r < budget.size(); ++r)
        if(budget.size() == 1)
            std::println("Budget             {:.10g}", budget[r]);
        else
            std::println("Budget {:<11} {:.10g}",
                         prepared.instance.cost_dimension_name(r), budget[r]);
    std::println(
        "Time               {}",
        format_duration(static_cast<double>(outcome.computation_time_ms) /
                        1000.0));
}

void write_json(std::FILE * stream, const run_arguments & run,
                const AbstractBoundInterface & method,
                const prepared_instance & prepared,
                const bound_outcome & outcome) {
    print_json_head(stream, run.instance_file, "method", method.name(),
                    method.json_parameters());
    print_json_instance(stream, prepared, outcome.computation_time_ms);
    std::print(stream, ",\n    \"bound\":          {},", outcome.bound);
    if(outcome.all_options_bound)
        std::print(stream, "\n    \"all_options_bound\": {},",
                   outcome.all_options_bound.value());
    std::print(stream, "\n    \"method_report\": {{\n        \"status\": {}",
               json_string(outcome.status));
    // JSON has no infinity nor NaN
    for(const auto & [key, value] : outcome.stats)
        if(std::isfinite(value))
            std::print(stream, ",\n        \"{}\": {}", key, value);
    std::print(stream, "\n    }}\n}}");
}

}  // namespace

int bound_command(const std::vector<std::string> & args) {
    std::string program_state = "Parsing arguments";
    try {
        run_arguments run;
        const auto method = parse_arguments(args, run);
        if(!method) return EXIT_SUCCESS;
        program_state = "Checking output files";
        check_output_directories({run.output_json_file});
        spdlog::info("GECOT {}, bound method {}", PROJECT_VERSION,
                     method->name());
        log_parameters(method->parameters());
        program_state = "Parsing instance";
        const prepared_instance prepared =
            prepare_instance(run.instance_file, run.budget, "method");
        const Instance & instance = prepared.instance;
        // the method's scores, bounds and logs are in this unit
        const double score_scale = prepared.score_scale;

        program_state = "Running method '" + method->name() + '\'';
        spdlog::stopwatch sw;
        gecot::bound_report report = [&] {
            const progress_watch watch(prepared);
            return instance.num_options() == 0
                       ? gecot::bound_report{prepared.base_score / score_scale,
                                             "no option left"}
                       : method->compute(instance, prepared.budget,
                                         {score_scale});
        }();
        const auto computation_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed())
                .count();

        program_state = "Checking the bound of '" + method->name() + '\'';
        check_bound(prepared, report);
        program_state = "Printing bound";
        const bound_outcome outcome =
            evaluate(prepared, std::move(report), computation_time_ms);
        if(!run.output_json_file) print_summary(prepared, outcome);
        if(run.output_json_file) {
            const auto file = open_output_file(
                std::filesystem::absolute(run.output_json_file.value()));
            write_json(file.get(), run, *method, prepared, outcome);
            spdlog::info("Bound written to {}",
                         std::filesystem::absolute(run.output_json_file.value())
                             .string());
        }
    } catch(const std::exception & e) {
        spdlog::error("{}: {}", program_state, e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
