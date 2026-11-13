#include "commands/common.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <format>
#include <iterator>
#include <print>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "gecot/helper.hpp"
#include "gecot/utils/log_table.hpp"
#include "gecot/utils/progress.hpp"

#include "io_helper.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

namespace {

const auto start_time = std::chrono::steady_clock::now();

double elapsed_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start_time)
        .count();
}

// %*: the seconds since gecot started
class elapsed_flag : public spdlog::custom_flag_formatter {
public:
    void format(const spdlog::details::log_msg &, const std::tm &,
                spdlog::memory_buf_t & dest) override {
        std::format_to(std::back_inserter(dest), "{:6.1f}s", elapsed_seconds());
    }
    std::unique_ptr<custom_flag_formatter> clone() const override {
        return std::make_unique<elapsed_flag>();
    }
};

// %&: "warning: " or "error: ", nothing below
class level_flag : public spdlog::custom_flag_formatter {
public:
    void format(const spdlog::details::log_msg & msg, const std::tm &,
                spdlog::memory_buf_t & dest) override {
        if(msg.level < spdlog::level::warn) return;
        std::format_to(std::back_inserter(dest), "{}: ",
                       msg.level == spdlog::level::warn ? "warning" : "error");
    }
    std::unique_ptr<custom_flag_formatter> clone() const override {
        return std::make_unique<level_flag>();
    }
};

static_assert(std::atomic<bool>::is_always_lock_free,
              "stop_requested is set in a signal handler");
extern "C" void request_stop(int) {
    gecot::stop_requested = true;
    std::signal(SIGINT, SIG_DFL);  // a second Ctrl-C ends gecot
}

}  // namespace

void print_soft_name() {
    std::print(
        "GECOT — Graph-based Ecological Connectivity "
        "Optimization Tool\nVersion: {} (built on {})\n\n",
        PROJECT_VERSION, __DATE__);
}

void setup_logging() {
    auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
#ifndef _WIN32
    // hues apart from info's green, warning's yellow and error's red, where
    // spdlog's cyan debug is close to green; Windows keeps spdlog's
    sink->set_color(spdlog::level::trace, "\033[90m");  // gray
    sink->set_color(spdlog::level::debug, sink->magenta);
#endif
    auto formatter = std::make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<elapsed_flag>('*').add_flag<level_flag>('&');
    // the time, and "warning: " or "error: ", in the level's color. The
    // tables count gecot::log_table<>::margin characters for it: a longer
    // prefix pushes their lines past 80 characters
    formatter->set_pattern("%^[%*] %&%$%v");
    sink->set_formatter(std::move(formatter));
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("gecot", sink));
}

void apply_logging(int verbosity, bool quiet) {
    if(quiet) {
        spdlog::set_level(spdlog::level::err);
    } else if(verbosity >= 2) {
        spdlog::set_level(spdlog::level::trace);
    } else if(verbosity == 1) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }
}

std::string format_score(double score) {
    // without an exponent: scores reach 1e11
    return std::abs(score) < 1e10 ? std::format("{:.10g}", score)
                                  : std::format("{:.0f}", score);
}

std::string format_percent(double percent, bool sign) {
    // not -3.78e-12 % for a relaxation at the initial score
    if(std::abs(percent) < 1e-9) percent = 0;
    std::string s = percent != 0 && std::abs(percent) < 0.01
                        ? std::format("{:.2e}", percent)
                        : std::format("{:.2f}", percent);
    if(sign && percent >= 0) s.insert(0, "+");
    return s + " %";
}

std::string format_gain(double score, double initial_score) {
    if(initial_score <= 0) return format_score(score);
    return format_percent((score - initial_score) / initial_score * 100, true);
}

std::optional<double> guaranteed_gain_share(double score, double bound,
                                            double base_score) {
    if(bound <= base_score) return std::nullopt;
    // at most 100 %: the bound can be a few ulps below
    const double share =
        std::clamp((score - base_score) / (bound - base_score), 0.0, 1.0);
    return std::floor(share * 1000) / 10;
}

std::string format_duration(double seconds) {
    const auto s = static_cast<long>(seconds);
    if(seconds < 1) return std::format("{:.0f} ms", seconds * 1000);
    if(seconds < 60) return std::format("{:.1f} s", seconds);
    if(seconds < 3600) return std::format("{} min {:02} s", s / 60, s % 60);
    return std::format("{} h {:02} min", s / 3600, s % 3600 / 60);
}

void log_parameters(
    const std::vector<std::pair<std::string, std::string>> & parameters) {
    if(parameters.empty()) return;
    const std::size_t name_max_length = std::ranges::max(std::views::transform(
        parameters, [](auto && p) { return p.first.size(); }));
    spdlog::debug("Parameters:");
    for(const auto & [name, value] : parameters)
        spdlog::debug("    {:<{}} {}", name, name_max_length, value);
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

std::unique_ptr<std::FILE, void (*)(std::FILE *)> open_output_file(
    const std::filesystem::path & file) {
    std::FILE * stream = std::fopen(file.string().c_str(), "w");
    if(!stream)
        throw std::system_error(errno, std::generic_category(),
                                "cannot open '" + file.string() + "'");
    return {stream, [](std::FILE * f) { std::fclose(f); }};
}

prepared_instance prepare_instance(const std::filesystem::path & instance_file,
                                   double budget, std::string_view noun,
                                   const bool merge) {
    Instance raw_instance = parse_instance(instance_file);
    spdlog::info("Instance: {}",
                 std::filesystem::absolute(instance_file).string());
    print_instance_size<spdlog::level::info>(raw_instance);
    double options_total_cost = 0;
    for(auto && o : raw_instance.options())
        options_total_cost += raw_instance.option_cost(o);
    if(options_total_cost > 0)
        spdlog::info("Budget {:g}: {} of the options' total cost ({:g})",
                     budget,
                     format_percent(budget / options_total_cost * 100, false),
                     options_total_cost);

    auto [instance, quality_scale] =
        trivial_reformulate_instance(raw_instance, budget, merge);

    spdlog::debug("Removed {} options and {} species",
                  raw_instance.num_options() - instance.num_options(),
                  raw_instance.cases().size() - instance.cases().size());
    print_instance_size<spdlog::level::debug>(instance);
    if(instance.num_options() == 0) {
        spdlog::warn("Budget doesn't allow purchasing any options");
    }
    // the algorithm's scores, bounds and logs are in this unit
    const double score_scale = quality_scale * quality_scale;
    auto initial_cases_pc_num = gecot::compute_base_cases_pc_num(raw_instance);
    const double initial_score =
        raw_instance.eval_criterion(initial_cases_pc_num);
    const double base_score =
        instance.eval_criterion(gecot::compute_base_cases_pc_num(instance)) *
        score_scale;
    spdlog::debug("Qualities scaled by 1/{:g}: {}'s scores scaled by 1/{:g}",
                  quality_scale, noun, score_scale);
    spdlog::debug("Initial score: {:g} (was {:g})", initial_score / score_scale,
                  initial_score);

    return {std::move(raw_instance),         std::move(instance), score_scale,
            std::move(initial_cases_pc_num), initial_score,       base_score};
}

progress_watch::progress_watch(const prepared_instance & prepared) {
    const auto gain = [scale = prepared.score_scale,
                       initial = prepared.initial_score](const double score) {
        return format_gain(score * scale, initial);
    };
    // the max gain of the rounds only with a bound, the nodes only at -v
    gecot::log_table<spdlog::level::info, 20, 12, 12, 12> rounds(
        {"round", gecot::log_format::fixed(0)}, {"relaxation", gain},
        {"max gain", gain});
    gecot::log_table<spdlog::level::info, 20, 12, 12, 12, 12> search(
        {"gain", gain}, {"max gain", gain},
        {"guaranteed",
         [](const double share) { return std::format("{:.1f} %", share); }},
        {"nodes", gecot::log_format::fixed(0)});
    gecot::report_progress = [scale = prepared.score_scale,
                              base = prepared.base_score, gain, rounds, search,
                              looked = std::chrono::steady_clock::time_point{},
                              printed = std::chrono::steady_clock::time_point{},
                              printed_values = std::string{}, started = false](
                                 const gecot::progress & p) mutable {
        using namespace std::chrono_literals;
        if(p.end) {
            rounds.end();
            search.end();
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        // a solver calls up to 100000 times a second
        if(!p.round && now - looked < 500ms) return;
        looked = now;
        const bool detailed = spdlog::should_log(spdlog::level::debug);
        // a row at most every 5 s, none in the first 2 s; at -v, every
        // second or every round
        if(detailed ? !p.round && now - printed < 1s
                    : now - printed < 5s || elapsed_seconds() < 2)
            return;

        // a model can value a solution below its score, as the Benders
        // masters do at 0 first: below buying nothing, it says nothing
        const auto score =
            p.score && *p.score * scale >= base ? p.score : std::nullopt;
        const auto share =
            !p.round && score && p.bound
                ? guaranteed_gain_share(*score * scale, *p.bound * scale, base)
                : std::nullopt;
        if(!std::exchange(started, true))
            spdlog::info(
                "Ctrl-C stops the algorithm and keeps its best result so "
                "far");
        // no row of a search that knows nothing yet, nor delay for its next
        if(!p.round && !score && !p.bound) return;
        // a row when a value changed, else one a minute; the round and
        // the nodes always change
        auto shown = [&gain](const std::optional<double> & v) {
            return v ? gain(*v) : std::string("-");
        };
        std::string values = p.round
                                 ? shown(p.relaxation) + '|' + shown(p.bound)
                                 : shown(score) + '|' + shown(p.bound) + '|' +
                                       std::format("{}", share.value_or(-1));
        if(values == printed_values && now - printed < 60s) return;
        printed = now;
        printed_values = std::move(values);
        // the rows of a table close the other one, left open by an
        // algorithm that did not report the end of its progress
        if(p.round) {
            search.end();
            p.bound ? rounds.row(*p.round, p.relaxation, p.bound)
                    : rounds.row(*p.round, p.relaxation);
        } else {
            rounds.end();
            detailed ? search.row(score, p.bound, share, p.nodes)
                     : search.row(score, p.bound, share);
        }
    };
    gecot::stop_requested = false;
    std::signal(SIGINT, request_stop);
}

progress_watch::~progress_watch() {
    std::signal(SIGINT, SIG_DFL);
    gecot::report_progress = [](const gecot::progress &) {};
}
