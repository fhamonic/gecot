#include "commands/common.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <concepts>
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
#include <variant>
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

// The criterion on one line, its cases by their names and its numbers to 3
// digits: min(9.65e-07 deer, 1.18e-06 bike - 14.2). A sum or a product of a
// single formula is that formula, so a single case reads as its name.
std::string criterion_text(const Instance & instance,
                           const gecot::criterion_formula & criterion) {
    return std::visit(
        [&]<typename F>(const F & f) -> std::string {
            if constexpr(std::same_as<F, gecot::criterion_constant>)
                return std::format("{:.3g}", f);
            else if constexpr(std::same_as<F, gecot::criterion_var>)
                return instance.cases()[f].name();
            else {
                constexpr bool is_sum = std::same_as<F, gecot::criterion_sum>;
                constexpr bool is_min = std::same_as<F, gecot::criterion_min>;
                std::string text;
                for(auto && e : f.values) {
                    std::string term = criterion_text(instance, e);
                    if(!is_sum && !is_min &&
                       (std::holds_alternative<gecot::criterion_sum>(e) ||
                        (&e != &f.values.front() && term.starts_with('-'))))
                        term = '(' + term + ')';
                    if(&e == &f.values.front())
                        text = std::move(term);
                    else if(is_sum && term.starts_with('-'))
                        text += " - " + term.substr(1);
                    else
                        text += (is_sum ? " + " : is_min ? ", " : " ") + term;
                }
                return is_min ? "min(" + text + ')' : text;
            }
        },
        criterion);
}

}  // namespace

void print_soft_name() {
    std::print(
        "GECOT — Graph-based Ecological Connectivity "
        "Optimization Tool\nVersion: {}\n\n",
        PROJECT_VERSION);
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
    // from 999.995 on, two decimals would read 1000.00
    std::string s = percent != 0 && std::abs(percent) < 0.01
                        ? std::format("{:.2e}", percent)
                        : std::format("{:.{}f}", percent,
                                      std::abs(percent) < 999.995 ? 2 : 0);
    if(sign && percent >= 0) s.insert(0, "+");
    return s + " %";
}

std::string format_gain(double score, double initial_score) {
    if(initial_score <= 0) return format_score(score);
    return format_percent((score - initial_score) / initial_score * 100, true);
}

void check_finite_score(const double score, const std::string_view what) {
    if(!std::isfinite(score))
        throw std::invalid_argument(std::format(
            "{} is not a finite number: the weights or the products of the "
            "criterion are too large",
            what));
}

std::optional<double> guaranteed_gain_share(double score, double bound,
                                            double base_score) {
    if(bound <= base_score) return std::nullopt;
    // its share would read 100 %
    if(bound < score - bound_margin * std::max(1.0, std::abs(score)))
        return std::nullopt;
    // only after the test above, which a bound far under the score fails
    if(bound <= score + gecot::mip_gap * std::abs(score)) return 100.0;
    const double share =
        std::max((score - base_score) / (bound - base_score), 0.0);
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
                                   const std::string & budget_text,
                                   std::string_view noun, const bool merge) {
    Instance raw_instance = parse_instance(instance_file);
    spdlog::info(
        "Instance: {}",
        std::filesystem::absolute(instance_file).lexically_normal().string());
    print_instance_size<spdlog::level::info>(raw_instance);
    if(spdlog::should_log(spdlog::level::debug)) {
        const std::string criterion =
            criterion_text(raw_instance, raw_instance.criterion());
        if(std::ranges::none_of(raw_instance.cases(), [&](auto && c) {
               return c.name() == criterion;
           }))
            spdlog::debug("Criterion: {}", criterion);
    }
    const std::size_t num_dimensions = raw_instance.num_cost_dimensions();
    if(num_dimensions > 1 || !raw_instance.conflict_sets().empty())
        spdlog::info("Cost dimensions: {}, conflict sets: {}", num_dimensions,
                     raw_instance.conflict_sets().size());
    gecot::budget_t budget = parse_budget(raw_instance, budget_text);
    for(std::size_t r = 0; r < num_dimensions; ++r) {
        double options_total_cost = 0;
        for(auto && o : raw_instance.options())
            options_total_cost += raw_instance.option_cost(o, r);
        if(options_total_cost <= 0) continue;
        const auto share =
            format_percent(budget[r] / options_total_cost * 100, false);
        if(num_dimensions == 1)
            spdlog::info(
                "Budget {:.10g}: {} of the options' total cost ({:.10g})",
                budget[r], share, options_total_cost);
        else
            spdlog::info(
                "Budget {}={:.10g}: {} of the options' total ({:.10g})",
                raw_instance.cost_dimension_name(r), budget[r], share,
                options_total_cost);
    }

    auto [instance, quality_exponent, criterion_exponent, static_options] =
        trivial_reformulate_instance(raw_instance, budget, merge);

    if(spdlog::should_log(spdlog::level::debug)) {
        std::size_t bought = 0, merged = 0, removed_arcs = 0;
        for(auto && i : raw_instance.options()) bought += static_options[i];
        for(const auto & instance_case : instance.cases()) {
            const auto & raw_graph =
                raw_instance
                    .cases()[raw_instance.case_id_from_name(
                        instance_case.name())]
                    .graph();
            merged +=
                raw_graph.num_vertices() - instance_case.graph().num_vertices();
            removed_arcs +=
                raw_graph.num_arcs() - instance_case.graph().num_arcs();
        }
        bool changed = false;
        auto log_change = [&changed](const char * kind,
                                     const std::size_t count) {
            if(count == 0) return;
            spdlog::debug("{}: {}", kind, count);
            changed = true;
        };
        log_change(
            "Options over the budget, removed",
            raw_instance.num_options() - instance.num_options() - bought);
        log_change("Options of cost 0, bought", bought);
        log_change("Cases that the criterion does not read, removed",
                   raw_instance.cases().size() - instance.cases().size());
        log_change("Vertices merged into others by arcs of probability 1",
                   merged);
        log_change("Arcs inside a vertex, removed", removed_arcs);
        if(changed)
            print_instance_size<spdlog::level::debug>(instance);
        else
            spdlog::debug("The reformulation removes, buys and merges nothing");
    }
    if(instance.num_options() == 0) {
        spdlog::warn("Budget doesn't allow purchasing any options");
    }
    // the algorithm's scores, bounds and logs are in this unit
    const double case_score_scale = std::ldexp(1.0, -2 * quality_exponent);
    const double score_scale =
        std::ldexp(1.0, -2 * quality_exponent - criterion_exponent);
    auto initial_cases_pc_num = gecot::compute_base_cases_pc_num(raw_instance);
    const double initial_score =
        raw_instance.eval_criterion(initial_cases_pc_num);
    const auto base_cases_pc_num = gecot::compute_base_cases_pc_num(instance);
    const double base_value = instance.eval_criterion(base_cases_pc_num);
    const double base_score = base_value * score_scale;
    check_finite_score(initial_score, "the initial score");
    check_finite_score(base_score,
                       "the score with the options bought statically");
    spdlog::debug("Qualities scaled by 2^{}: {}'s scores by 2^{}",
                  quality_exponent, noun,
                  2 * quality_exponent + criterion_exponent);
    if(criterion_exponent != 0)
        spdlog::debug(
            "The criterion multiplied by 2^{} on top, to the size of its "
            "cases",
            criterion_exponent);
    // The relative tolerances of the solvers, a gap of 1e-10, and the margin
    // of 1e-6 of the check of a bound, apply to the whole value of the
    // criterion: an added constant that makes it 2^20 times its terms puts
    // that margin at their size.
    if(const double size =
           criterion_size(instance.criterion(), base_cases_pc_num);
       size > 0 && std::abs(base_value) > 0x1p20 * size)
        spdlog::warn("A constant {:.3g} times the other terms can hide gains",
                     std::abs(base_value) / size);
    if(score_scale != 1 && spdlog::should_log(spdlog::level::debug))
        spdlog::debug("Initial score {}, for the {} {}",
                      format_score(initial_score), noun,
                      format_score(initial_score / score_scale));

    return {std::move(raw_instance),
            std::move(instance),
            score_scale,
            case_score_scale,
            std::move(initial_cases_pc_num),
            initial_score,
            base_score,
            std::move(budget),
            std::move(static_options)};
}

void print_json_per_dimension(std::FILE * stream, const std::string & property,
                              const std::string & line,
                              const Instance & instance,
                              const std::vector<double> & values) {
    if(values.size() == 1) {
        std::print(stream, "\n    {}{}", line, values.front());
        return;
    }
    print_json_property(stream, property,
                        std::views::transform(
                            std::views::iota(std::size_t{0}, values.size()),
                            [&](const std::size_t r) {
                                return std::make_pair(
                                    instance.cost_dimension_name(r), values[r]);
                            }));
}

void print_json_head(
    std::FILE * stream, const std::filesystem::path & instance_file,
    const std::string & kind, const std::string & name,
    const std::vector<std::pair<std::string, std::string>> & parameters) {
    std::print(
        stream,
        "{{\n    \"gecot_version\":  \"{}\",\n    \"format_version\": {},\n    "
        "\"instance_path\":  {},\n    \"{}\":  \"{}\",",
        PROJECT_VERSION, result_format_version,
        json_string(std::filesystem::absolute(instance_file)
                        .lexically_normal()
                        .generic_string()),
        kind, name);
    print_json_property(stream, kind + "_parameters", parameters);
}

void print_json_instance(std::FILE * stream, const prepared_instance & prepared,
                         const long long computation_time_ms) {
    std::print(stream,
               ",\n    \"computation_time_ms\":  {},\n    "
               "\"initial_score\":  {},\n    \"base_score\":     {},",
               computation_time_ms, prepared.initial_score,
               prepared.base_score);
    print_json_property(
        stream, "initial_pc_num",
        std::views::transform(
            prepared.raw_instance.cases(),
            [&prepared](const auto & instance_case) {
                return std::make_pair(
                    instance_case.name(),
                    prepared.initial_cases_pc_num[instance_case.id()]);
            }));
    std::print(stream, ",");
    print_json_per_dimension(stream, "budget",
                             "\"budget\":         ", prepared.raw_instance,
                             prepared.budget.values);
}

progress_watch::progress_watch(const prepared_instance & prepared) {
    const auto gain = [scale = prepared.score_scale,
                       initial = prepared.initial_score](const double score) {
        return format_gain(score * scale, initial);
    };
    gecot::gain_text = gain;
    gecot::change_text = [scale = prepared.score_scale,
                          initial = prepared.initial_score](const double d) {
        return initial > 0 ? format_percent(d * scale / initial * 100, true)
                           : format_score(d * scale);
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
                              printed_values = std::string{}, started = false,
                              last_round = gecot::progress{}](
                                 const gecot::progress & report) mutable {
        using namespace std::chrono_literals;
        const auto now = std::chrono::steady_clock::now();
        const bool detailed = spdlog::should_log(spdlog::level::debug);
        auto row = [&](const gecot::progress & p, const bool closing) {
            // a model can value a solution below its score, as the Benders
            // masters do at 0 first: below buying nothing, it says nothing
            const auto score =
                p.score && *p.score * scale >= base ? p.score : std::nullopt;
            const auto share = !p.round && score && p.bound
                                   ? guaranteed_gain_share(
                                         *score * scale, *p.bound * scale, base)
                                   : std::nullopt;
            if(!std::exchange(started, true))
                spdlog::info(
                    "Ctrl-C stops the algorithm and keeps its best result so "
                    "far");
            // no row of a search that knows nothing yet, nor delay for its
            // next
            if(!p.round && !score && !p.bound) return;
            // a row when a value changed, else one a minute; the round and
            // the nodes always change
            auto shown = [&gain](const std::optional<double> & v) {
                return v ? gain(*v) : std::string("-");
            };
            std::string values =
                p.round ? shown(p.relaxation) + '|' + shown(p.bound)
                        : shown(score) + '|' + shown(p.bound) + '|' +
                              std::format("{}", share.value_or(-1));
            if(values == printed_values && (closing || now - printed < 60s))
                return;
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
        if(report.end) {
            // the state the step ended in, if its table shows another: a
            // search sends it with the end, a loop of rounds as its last round
            if(!printed_values.empty())
                row(report.score || report.bound ? report : last_round, true);
            rounds.end();
            search.end();
            printed_values.clear();
            last_round = {};
            return;
        }
        if(report.round) last_round = report;
        // a solver calls up to 100000 times a second
        if(!report.round && now - looked < 500ms) return;
        looked = now;
        // a row at most every 5 s, none in the first 2 s; at -v, every
        // second
        if(detailed ? now - printed < 1s
                    : now - printed < 5s || elapsed_seconds() < 2)
            return;
        row(report, false);
    };
    gecot::stop_requested = false;
    _previous_handler = std::signal(SIGINT, request_stop);
}

progress_watch::~progress_watch() {
    std::signal(SIGINT, _previous_handler);
    gecot::stop_requested = false;
    gecot::report_progress = [](const gecot::progress &) {};
    gecot::gain_text = gecot::change_text = [](const double score) {
        return std::format("{:g}", score);
    };
}
