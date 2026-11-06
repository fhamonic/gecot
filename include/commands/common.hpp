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
void apply_logging(bool verbose, bool quiet);
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
};
// logs the steps of 'gecot optimize', with "the <noun>'s scores"
prepared_instance prepare_instance(const std::filesystem::path & instance_file,
                                   double budget, std::string_view noun);

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
