#ifndef COMMANDS_COMMON_HPP
#define COMMANDS_COMMON_HPP

#include <filesystem>
#include <initializer_list>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/os.h>

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

struct prepared_instance {
    Instance raw_instance;
    // trivial_reformulate_instance's: free options applied, options over the
    // budget removed, qualities divided by sqrt(score_scale)
    Instance instance;
    double score_scale;
    // of raw_instance, free options not applied
    melon::static_map<gecot::case_id_t, double> initial_cases_pc_num;
    double initial_score;
    double base_score;  // with the options of cost 0, not divided
};
// logs the steps of 'gecot optimize', with "the <noun>'s scores"
prepared_instance prepare_instance(const std::filesystem::path & instance_file,
                                   double budget, std::string_view noun);

// While it lives, the algorithm's progress is logged in raw scores, and Ctrl-C
// stops it: it returns its best result so far.
struct progress_watch {
    explicit progress_watch(const prepared_instance & prepared);
    ~progress_watch();
    progress_watch(const progress_watch &) = delete;
    progress_watch & operator=(const progress_watch &) = delete;
};

// the 13 algorithms of 'gecot optimize', in the order -A lists them
std::vector<std::shared_ptr<AbstractSolverInterface>> make_solver_interfaces();
// the methods of 'gecot bound', in the order -M lists them
std::vector<std::shared_ptr<AbstractBoundInterface>> make_bound_interfaces();

template <typename R>
void print_json_property(fmt::ostream & out, const std::string & property_name,
                         R && pairs_range) {
    out.print("\n    \"{}\": {{", property_name);
    const std::size_t key_max_length =
        (std::ranges::empty(pairs_range)
             ? 0
             : std::ranges::max(std::ranges::views::transform(
                   std::views::keys(pairs_range),
                   [&](auto && s) { return s.size(); }))) +
        3;
    bool first_line = true;
    for(const auto & [key, value] : pairs_range) {
        out.print("{}\n        {:<{}} {}", first_line ? "" : ",",
                  std::string("\"") + key + "\":", key_max_length, value);
        first_line = false;
    }
    out.print("\n    }}");
}

#endif  // COMMANDS_COMMON_HPP
