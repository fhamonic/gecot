#ifndef COMMANDS_COMMON_HPP
#define COMMANDS_COMMON_HPP

#include <cstdio>
#include <filesystem>
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
#include "solver_interfaces/abstract_solver_interface.hpp"

void print_soft_name();
// on stderr, each line after the seconds since gecot started
void setup_logging();
// 0: the steps and the progress, 1: the algorithms' details, 2: everything
void apply_logging(int verbosity, bool quiet);

// at most 10 significant digits
std::string format_score(double score);
// two decimals, in scientific notation below 0.01 %
std::string format_percent(double percent, bool sign);
// "+11.28 %" over initial_score, the score itself if initial_score <= 0
std::string format_gain(double score, double initial_score);
// in %, rounded down: the share of the best possible gain, bound -
// base_score, that score - base_score reaches
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
    // options over the budget removed, qualities divided by sqrt(score_scale)
    Instance instance;
    double score_scale;
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

// While it lives, the algorithm's progress is logged in raw scores, and Ctrl-C
// stops it: it returns its best result so far.
struct progress_watch {
    explicit progress_watch(const prepared_instance & prepared);
    ~progress_watch();
    progress_watch(const progress_watch &) = delete;
    progress_watch & operator=(const progress_watch &) = delete;
};

// 'gecot optimize --reformulation path_lp': path_lp, with the given
// "name=value" parameters and greedy's solution as its default MIP
// start, bounds for each option the score of the solutions that buy it. The
// options whose bound is no more than the score of path_lp's solution are in
// no better solution: solver runs on the instance without them, from
// path_lp's solution, and the result is the better of the two solutions.
class PathFormulationLPInterface;
std::shared_ptr<const PathFormulationLPInterface> make_path_reformulation(
    const std::string & parameters);
gecot::solve_result<Instance> solve_path_reformulated(
    const PathFormulationLPInterface & path_lp,
    AbstractSolverInterface & solver, const Instance & instance,
    const gecot::budget_t & budget, bool merge = true);

// the algorithms of 'gecot optimize', in the order -A lists them
std::vector<std::shared_ptr<AbstractSolverInterface>> make_solver_interfaces();
// the methods of 'gecot bound', in the order -M lists them
std::vector<std::shared_ptr<AbstractBoundInterface>> make_bound_interfaces();

template <typename R>
void print_json_property(std::FILE * stream, const std::string & property_name,
                         R && pairs_range) {
    std::print(stream, "\n    \"{}\": {{", property_name);
    const std::size_t key_max_length =
        (std::ranges::empty(pairs_range)
             ? 0
             : std::ranges::max(std::ranges::views::transform(
                   std::views::keys(pairs_range),
                   [&](auto && s) { return s.size(); }))) +
        3;
    bool first_line = true;
    for(const auto & [key, value] : pairs_range) {
        std::print(stream, "{}\n        {:<{}} {}", first_line ? "" : ",",
                   std::string("\"") + key + "\":", key_max_length, value);
        first_line = false;
    }
    std::print(stream, "\n    }}");
}

#endif  // COMMANDS_COMMON_HPP
