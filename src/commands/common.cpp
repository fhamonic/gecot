#include "commands/common.hpp"

#include <stdexcept>

#include <fmt/core.h>

#include <spdlog/spdlog.h>

#include "gecot/helper.hpp"

#include "io_helper.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

void print_soft_name() {
    fmt::print(
        "GECOT — Graph-based Ecological Connectivity "
        "Optimization Tool\nVersion: {} (built on {})\n\n",
        PROJECT_VERSION, __DATE__);
}

void apply_logging(bool verbose, bool quiet) {
    if(quiet) {
        spdlog::set_level(spdlog::level::err);
    } else if(verbose) {
        spdlog::set_level(spdlog::level::trace);
    } else {
        spdlog::set_level(spdlog::level::info);
    }
}

void check_output_directories(
    std::initializer_list<std::optional<std::filesystem::path>> files) {
    for(auto && opt_file : files) {
        if(!opt_file.has_value()) continue;
        const auto parent =
            std::filesystem::absolute(opt_file.value()).parent_path();
        if(!std::filesystem::is_directory(parent))
            throw std::runtime_error("output directory '" + parent.string() +
                                     "' does not exist");
    }
}

prepared_instance prepare_instance(const std::filesystem::path & instance_file,
                                   double budget, std::string_view noun) {
    Instance raw_instance = parse_instance(instance_file);
    spdlog::info("Parsed instance: {}",
                 std::filesystem::absolute(instance_file).string());

    print_instance_size<spdlog::level::info>(raw_instance);
    double options_total_cost = 0;
    for(auto && o : raw_instance.options())
        options_total_cost += raw_instance.option_cost(o);
    if(options_total_cost > 0)
        spdlog::info(
            "Budget represents {:.2f}% of the total options cost (={:g})",
            budget / options_total_cost * 100, options_total_cost);

    auto [instance, quality_scale] =
        trivial_reformulate_instance(raw_instance, budget);
    print_instance_size<spdlog::level::trace>(instance, "Simplified instance");
    if(instance.num_options() == 0) {
        spdlog::warn("Budget doesn't allow purchasing any options");
    }
    // the algorithm's scores, bounds and logs are in this unit
    const double score_scale = quality_scale * quality_scale;
    auto initial_cases_pc_num = gecot::compute_base_cases_pc_num(raw_instance);
    const double initial_score =
        raw_instance.eval_criterion(initial_cases_pc_num);
    spdlog::info(
        "Qualities divided by {:g}: the {}'s scores are divided by {:g}, its "
        "initial score is {:g}",
        quality_scale, noun, score_scale, initial_score / score_scale);

    return {std::move(raw_instance), std::move(instance), score_scale,
            std::move(initial_cases_pc_num), initial_score};
}
