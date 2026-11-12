#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include "gecot/utils/log_table.hpp"

namespace {

using gecot::log_table;
namespace log_format = gecot::log_format;

// the lines logged while it lives, without the margin of gecot's log lines
struct captured_log {
    std::ostringstream out;
    std::shared_ptr<spdlog::logger> previous = spdlog::default_logger();

    explicit captured_log(
        const spdlog::level::level_enum level = spdlog::level::trace) {
        auto logger = std::make_shared<spdlog::logger>(
            "test", std::make_shared<spdlog::sinks::ostream_sink_mt>(out));
        logger->set_pattern("%v");
        logger->set_level(level);
        spdlog::set_default_logger(std::move(logger));
    }
    ~captured_log() { spdlog::set_default_logger(std::move(previous)); }

    std::vector<std::string> lines() const {
        std::vector<std::string> lines;
        std::istringstream in(out.str());
        for(std::string line; std::getline(in, line);) lines.push_back(line);
        return lines;
    }
};

using strings = std::vector<std::string>;

// the margin, three columns and their two separators fill the 80 characters
static_assert(log_table<spdlog::level::info, 0, 40, 12, 11>::margin +
                  log_table<spdlog::level::info, 0, 40, 12, 11>::table_width ==
              80);
// a row has the columns of the table at most
template <typename T>
concept takes_four_values = requires(T table) { table.row(1, 2, 3, 4); };
static_assert(takes_four_values<log_table<spdlog::level::info, 0, 4, 4, 4, 4>>);
static_assert(!takes_four_values<log_table<spdlog::level::info, 0, 4, 4, 4>>);

}  // namespace

GTEST_TEST(log_table, wraps_and_centers_the_titles) {
    const captured_log log;
    log_table<spdlog::level::debug, 0, 9, 12, 13> table(
        {"µ value", log_format::scientific(3)},
        {"strong fiber (avg. size)", log_format::fixed(3)},
        {"useless fiber (avg. size)", log_format::fixed(3)});
    table.row(0.0, 1.5, 2.25);
    table.row(1.25e-3, 10, 20.0);
    // µ takes two bytes and one column: the lines are compared as text
    EXPECT_EQ(log.lines(),
              (strings{" µ value  | strong fiber | useless fiber",
                       "          | (avg. size)  |  (avg. size)",
                       "----------+--------------+--------------",
                       "0.000e+00 |        1.500 |         2.250",
                       "1.250e-03 |       10.000 |        20.000"}));
}

GTEST_TEST(log_table, repeats_the_header_after_its_period) {
    const captured_log log;
    log_table<spdlog::level::info, 2, 5> table({"n", log_format::fixed(0)});
    for(int n = 1; n <= 5; ++n) table.row(n);
    EXPECT_EQ(log.lines(),
              (strings{"  n", "-----", "    1", "    2", "  n", "-----",
                       "    3", "    4", "  n", "-----", "    5"}));
}

GTEST_TEST(log_table, prints_texts_and_absent_values) {
    const captured_log log;
    log_table<spdlog::level::info, 0, 6, 6, 6> table(
        {"id"}, {"value", log_format::fixed(1)}, {"count"});
    table.row("µ1", std::optional<double>{2.5}, std::optional<std::size_t>{});
    table.row(std::string("b"), std::optional<double>{}, std::size_t{12});
    EXPECT_EQ(
        log.lines(),
        (strings{"  id   | value  | count", "-------+--------+-------",
                 "    µ1 |    2.5 |      -", "     b |      - |     12"}));
}

GTEST_TEST(log_table, cuts_the_cells_wider_than_their_column) {
    const captured_log log;
    log_table<spdlog::level::info, 0, 8, 6> table(
        {"id"}, {"value", log_format::fixed(1)});
    // identifiers that share a prefix keep the end where they differ, a
    // number the digits that give its size
    table.row("crossing_3ffff_7ffff", 123456789.0);
    table.row("µµµµµµµµµ", -1.0);
    const strings lines = log.lines();
    ASSERT_EQ(lines.size(), std::size_t{4});
    EXPECT_EQ(lines[2], "…f_7ffff | 12345…");
    EXPECT_EQ(lines[3], "…µµµµµµµ |   -1.0");
    for(const std::string & line : lines)
        EXPECT_LE(gecot::detail::num_characters(line), table.table_width);
}

GTEST_TEST(log_table, shows_the_columns_that_a_row_fills) {
    const captured_log log;
    log_table<spdlog::level::info, 0, 4, 4, 4> table({"a"}, {"b"}, {"cc dd"});
    table.row(1, 2);
    table.row(3, 4);
    table.row(5, 6, 7);
    table.row(8, 9);
    EXPECT_EQ(log.lines(),
              (strings{" a   |  b", "-----+-----", "   1 |    2", "   3 |    4",
                       " a   |  b   |  cc", "     |      |  dd",
                       "-----+------+-----", "   5 |    6 |    7", " a   |  b",
                       "-----+-----", "   8 |    9"}));
}

GTEST_TEST(log_table, logs_and_counts_nothing_below_its_level) {
    const captured_log log(spdlog::level::info);
    log_table<spdlog::level::debug, 1, 4> table({"a"});
    table.row(1);
    table.row(2);
    EXPECT_TRUE(log.lines().empty());
    spdlog::default_logger()->set_level(spdlog::level::debug);
    table.row(3);
    EXPECT_EQ(log.lines(), (strings{" a", "----", "   3"}));
}

GTEST_TEST(log_table, restarts_with_its_header) {
    const captured_log log;
    log_table<spdlog::level::info, 0, 4> table({"a"});
    table.row(1);
    table.restart();
    table.row(2);
    EXPECT_EQ(log.lines(),
              (strings{" a", "----", "   1", " a", "----", "   2"}));
}
