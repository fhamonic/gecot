#ifndef COMMANDS_COMMON_HPP
#define COMMANDS_COMMON_HPP

#include <cstdio>
#include <filesystem>
#include <format>
#include <initializer_list>
#include <memory>
#include <optional>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "melon/container/static_map.hpp"

#include "bound_interfaces/abstract_bound_interface.hpp"
#include "instance.hpp"
#include "io_helper.hpp"
#include "solver_interfaces/abstract_solver_interface.hpp"

void print_soft_name();
// on stderr, each line after the seconds since gecot started
void setup_logging();
// 0: the steps and the progress, 1: the algorithms' details, 2: everything
void apply_logging(int verbosity, bool quiet);

// at most 10 significant digits
std::string format_score(double score);
// two decimals, none from 1000 % on, in scientific notation below 0.01 %
std::string format_percent(double percent, bool sign);
// "+11.28 %" over initial_score, the score itself if initial_score <= 0
std::string format_gain(double score, double initial_score);
// throws if score, which what names, is not a finite number, which JSON has
// not: the qualities are checked when the instance is reformulated, so the
// weights or the products of the criterion are too large
void check_finite_score(double score, std::string_view what);
// How far under the score of a solution, relative to the score or to 1, a
// bound still counts as one: at the optimum, the bound of a solver is a few
// ulps under the recomputed score. Further under, its tolerances failed.
inline constexpr double bound_margin = 1e-6;
// in %, rounded down: the share of the best possible gain, bound -
// base_score, that score - base_score reaches; 100 if bound is within
// gecot::mip_gap above score; none if bound is not above base_score, or is
// under score by more than bound_margin
std::optional<double> guaranteed_gain_share(double score, double bound,
                                            double base_score);
std::string format_duration(double seconds);
// at debug level
void log_parameters(
    const std::vector<std::pair<std::string, std::string>> & parameters);
// throws if the directory of a given output file does not exist
void check_output_directories(
    std::initializer_list<std::optional<std::filesystem::path>> files);
// opened for writing, throws if it cannot be, closed with the pointer
std::unique_ptr<std::FILE, void (*)(std::FILE *)> open_output_file(
    const std::filesystem::path & file);

struct prepared_instance {
    Instance raw_instance;
    // trivial_reformulate_instance's: options bought statically applied,
    // options over the budget removed, qualities divided by
    // sqrt(case_score_scale)
    Instance instance;
    // a score of the criterion of the raw instance is score_scale times the
    // instance's; a score of a case, case_score_scale times
    double score_scale;
    double case_score_scale;
    // of raw_instance, options bought statically not applied
    melon::static_map<gecot::case_id_t, double> initial_cases_pc_num;
    double initial_score;
    double base_score;  // with the options bought statically, not divided
    gecot::budget_t budget;
    // the options of raw_instance bought statically
    gecot::instance_solution_t<Instance> static_options;
};
// logs the steps of 'gecot optimize', with "the <noun>'s scores"; the budget
// is parse_budget's text
prepared_instance prepare_instance(const std::filesystem::path & instance_file,
                                   const std::string & budget,
                                   std::string_view noun, bool merge = true);
// a number per cost dimension: on the line of the property when the options
// have one cost, as an object keyed by the dimensions otherwise
void print_json_per_dimension(std::FILE * stream, const std::string & property,
                              const std::string & line,
                              const Instance & instance,
                              const std::vector<double> & values);

// The format of the result files, raised when a field changes its meaning
// or goes: a field added leaves it.
inline constexpr int result_format_version = 1;

// The head of a result file: the versions, the instance, then the algorithm
// or the method, which kind names, with its parameters.
void print_json_head(
    std::FILE * stream, const std::filesystem::path & instance_file,
    const std::string & kind, const std::string & name,
    const std::vector<std::pair<std::string, std::string>> & parameters);
// what follows the parameters in a result file: the time, the initial and
// base scores and the budget
void print_json_instance(std::FILE * stream, const prepared_instance & prepared,
                         long long computation_time_ms);

// While it lives, the algorithm's progress is logged in raw scores, and Ctrl-C
// stops it: it returns its best result so far.
struct progress_watch {
    explicit progress_watch(const prepared_instance & prepared);
    ~progress_watch();
    progress_watch(const progress_watch &) = delete;
    progress_watch & operator=(const progress_watch &) = delete;

private:
    void (*_previous_handler)(int);
};

// 'gecot optimize --reformulation path_lp': path_lp, with the given
// "name=value" parameters and greedy's solution as its default MIP
// start, bounds for each option the score of the solutions that buy it. The
// options whose bound is no more than the score of path_lp's solution are in
// no better solution: solver runs on the instance without them, from
// path_lp's solution, and the result is the better of the two solutions.
class PathLpInterface;
std::shared_ptr<const PathLpInterface> make_path_reformulation(
    const std::string & parameters);
gecot::solve_result<Instance> solve_path_reformulated(
    const PathLpInterface & path_lp, AbstractSolverInterface & solver,
    const Instance & instance, const gecot::budget_t & budget,
    bool merge = true);

// the algorithms of 'gecot optimize', in the order -A lists them
std::vector<std::shared_ptr<AbstractSolverInterface>> make_solver_interfaces();
// the methods of 'gecot bound', in the order -M lists them
std::vector<std::shared_ptr<AbstractBoundInterface>> make_bound_interfaces();

// A name of the instance as a JSON string: its quotes, backslashes and
// control characters escaped, its other bytes kept. A name read from a CSV
// file in another encoding is not UTF-8: with the replacement character of
// json_string, two names could become one key, and none would match the
// instance when the result is given back as a MIP start.
inline std::string json_name(const std::string & name) {
    std::string quoted = "\"";
    for(const char c : name) {
        if(c == '"' || c == '\\')
            (quoted += '\\') += c;
        else if(static_cast<unsigned char>(c) < 0x20)
            quoted += std::format("\\u{:04x}", static_cast<unsigned char>(c));
        else
            quoted += c;
    }
    return quoted + '"';
}

// the keys are names, aligned on their escaped length
template <typename R>
void print_json_property(std::FILE * stream, const std::string & property_name,
                         R && pairs_range) {
    std::print(stream, "\n    \"{}\": {{", property_name);
    const std::size_t key_max_length =
        (std::ranges::empty(pairs_range)
             ? 2
             : std::ranges::max(std::ranges::views::transform(
                   std::views::keys(pairs_range),
                   [&](auto && s) { return json_name(s).size(); }))) +
        1;
    bool first_line = true;
    for(const auto & [key, value] : pairs_range) {
        std::print(stream, "{}\n        {:<{}} {}", first_line ? "" : ",",
                   json_name(key) + ":", key_max_length, value);
        first_line = false;
    }
    std::print(stream, "\n    }}");
}

#endif  // COMMANDS_COMMON_HPP
