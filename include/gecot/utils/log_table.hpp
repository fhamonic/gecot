#ifndef GECOT_UTILS_LOG_TABLE_HPP
#define GECOT_UTILS_LOG_TABLE_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <format>
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace gecot {

// formats for the numbers of a log_column
namespace log_format {
inline std::string plain(const double value) {
    return std::format("{}", value);
}
inline auto fixed(const int precision) {
    return [precision](const double value) {
        return std::format("{:.{}f}", value, precision);
    };
}
inline auto scientific(const int precision) {
    return [precision](const double value) {
        return std::format("{:.{}e}", value, precision);
    };
}
// digits significant digits, the zeros that end them kept: the values of a
// magnitude are aligned on their point
inline auto general(const int digits) {
    return [digits](const double value) {
        return std::format("{:#.{}g}", value, digits);
    };
}
}  // namespace log_format

namespace detail {
// The characters of a UTF-8 text: the columns it takes in a log, as long as
// none of them is wide or combining.
constexpr std::size_t num_characters(const std::string_view text) {
    return static_cast<std::size_t>(std::ranges::count_if(
        text, [](const char c) { return (c & 0xC0) != 0x80; }));
}
}  // namespace detail

// A column of a log_table, WIDTH characters wide: its title and the format of
// the numbers of its cells.
template <std::size_t WIDTH>
struct log_column {
    // The title is wrapped at its spaces and each of its lines is centered.
    // A word wider than the column would push the header past the width of
    // the table: such a title does not compile, nor one that is not a
    // constant.
    struct title_text {
        std::string_view text;
        consteval title_text(const char * const title) : text(title) {
            for(std::size_t begin = 0; begin < text.size();) {
                const std::size_t end =
                    std::min(text.find(' ', begin), text.size());
                if(detail::num_characters(text.substr(begin, end - begin)) >
                   WIDTH)
                    throw "a word of this title is wider than its column";
                begin = end + 1;
            }
        }
    };
    title_text title;
    std::function<std::string(double)> format = log_format::plain;
};

// Logs a table at level LVL, row by row, in columns of WIDTHS characters
// separated by " | ". The header, the titles between two rules, is logged
// before the first row, then again before a row that follows PERIOD rows
// without one (0: never). A last rule closes the table: at end(), before a
// row that has not as many columns as the row before it, or when the table
// is destroyed.
//
// No line passes 80 characters. The compiler checks that the widths, the
// separators and the margin of gecot's log lines fit in them, and that each
// word of a title fits in its column. A cell wider than its column is cut: a
// number keeps its first characters before "…", a text its last ones after
// it, which is where identifiers that share a prefix differ.
template <spdlog::level::level_enum LVL, std::size_t PERIOD,
          std::size_t... WIDTHS>
class log_table {
public:
    static constexpr std::size_t num_columns = sizeof...(WIDTHS);
    // The seconds since gecot started, "[%6.1fs] ", that setup_logging() in
    // src/commands/common.cpp puts before each line: 10 characters, 11 from
    // 10000 s on and 12 only after 27 hours.
    static constexpr std::size_t margin = 11;
    static constexpr std::size_t table_width =
        (WIDTHS + ... + 0) + 3 * (num_columns - 1);
    static_assert(num_columns > 0 && ((WIDTHS > 0) && ...),
                  "a table has columns, each a character wide at least");
    static_assert(margin + table_width <= 80,
                  "the columns, their separators and the margin of the log "
                  "lines do not fit in 80 characters");

private:
    static constexpr std::array<std::size_t, num_columns> _widths{WIDTHS...};
    std::array<std::function<std::string(double)>, num_columns> _formats;
    std::array<std::vector<std::string>, num_columns> _titles;  // their lines
    std::size_t _num_shown = 0;  // columns of the table begun, 0: none
    std::size_t _num_rows = 0;   // logged since its last header

    static std::string _rule(const std::size_t num_shown) {
        std::string rule;
        for(std::size_t c = 0; c < num_shown; ++c)
            rule.append(c > 0 ? "-+-" : "").append(_widths[c], '-');
        return rule;
    }

    // The last lines are filled first: filled from the first, "arcs per
    // target" in 10 characters is "arcs per" above "target", beside
    // "vertices" above "per target".
    static std::vector<std::string> _wrapped(const std::string_view title,
                                             const std::size_t width) {
        std::vector<std::string> lines(1);
        for(std::size_t end = title.size(); end > 0;) {
            const std::size_t space = title.rfind(' ', end - 1);
            const std::size_t begin =
                space == std::string_view::npos ? 0 : space + 1;
            const std::string_view word = title.substr(begin, end - begin);
            if(lines.back().empty())
                lines.back() = word;
            else if(detail::num_characters(lines.back()) + 1 +
                        detail::num_characters(word) <=
                    width)
                lines.back().insert(0, " ").insert(0, word);
            else
                lines.emplace_back(word);
            end = begin == 0 ? 0 : begin - 1;
        }
        std::ranges::reverse(lines);
        return lines;
    }

    // text right-aligned in width characters, or cut to them
    static std::string _fitted(const std::string_view text,
                               const std::size_t width, const bool number) {
        const std::size_t length = detail::num_characters(text);
        if(length <= width)
            return std::string(width - length, ' ').append(text);
        auto byte_of = [text](std::size_t character) {
            std::size_t byte = 0;
            while(byte < text.size() &&
                  ((text[byte] & 0xC0) == 0x80 || character-- > 0))
                ++byte;
            return byte;
        };
        return number
                   ? std::string(text.substr(0, byte_of(width - 1))).append("…")
                   : std::string("…").append(
                         text.substr(byte_of(length - width + 1)));
    }

    template <typename V>
    std::string _cell(const std::size_t column, const V & value) const {
        if constexpr(requires {
                         value.has_value();
                         *value;
                     })
            return value.has_value() ? _cell(column, *value)
                                     : _cell(column, "-");
        else if constexpr(std::is_arithmetic_v<V>)
            return _fitted(_formats[column](static_cast<double>(value)),
                           _widths[column], true);
        else
            return _fitted(std::string_view(value), _widths[column], false);
    }

    void _log_header(const std::size_t num_shown) {
        spdlog::log(LVL, "{}", _rule(num_shown));
        std::size_t num_lines = 0;
        for(std::size_t c = 0; c < num_shown; ++c)
            num_lines = std::max(num_lines, _titles[c].size());
        for(std::size_t l = 0; l < num_lines; ++l) {
            std::string line;
            for(std::size_t c = 0; c < num_shown; ++c) {
                const std::string_view text =
                    l < _titles[c].size() ? _titles[c][l] : std::string_view();
                const std::size_t space =
                    _widths[c] - detail::num_characters(text);
                line.append(c > 0 ? " | " : "")
                    .append(space / 2, ' ')
                    .append(text)
                    .append(space - space / 2, ' ');
            }
            line.erase(line.find_last_not_of(' ') + 1);
            spdlog::log(LVL, "{}", line);
        }
        spdlog::log(LVL, "{}", _rule(num_shown));
        _num_shown = num_shown;
        _num_rows = 0;
    }

public:
    explicit log_table(log_column<WIDTHS>... columns)
        : _formats{std::move(columns.format)...}
        , _titles{_wrapped(columns.title.text, WIDTHS)...} {
        // row() runs in the solvers' callbacks, where a throw ends gecot
        for(auto & format : _formats)
            if(!format) format = log_format::plain;
    }
    // A copy has no row: with those of its source, each copy that a
    // std::function makes of a table would log their closing rule.
    log_table(const log_table & other)
        : _formats(other._formats), _titles(other._titles) {}
    log_table & operator=(const log_table &) = delete;
    ~log_table() { end(); }

    // Logs a row of the first columns, with a value each: a number, that its
    // column formats; a text; or an optional of either, "-" when empty.
    // Nothing is logged, nor counted, while the level LVL is off.
    template <typename... V>
        requires(sizeof...(V) > 0 && sizeof...(V) <= num_columns)
    void row(const V &... values) {
        if(!spdlog::should_log(LVL)) return;
        if(_num_shown != sizeof...(V)) end();
        if(_num_shown == 0 || (PERIOD > 0 && _num_rows == PERIOD))
            _log_header(sizeof...(V));
        ++_num_rows;
        std::string line;
        std::size_t column = 0;
        ((line.append(column > 0 ? " | " : "").append(_cell(column, values)),
          ++column),
         ...);
        spdlog::log(LVL, "{}", line);
    }

    // Closes the table, if it has rows, before the lines that are not its
    // own: the next row begins another one, with its header.
    void end() {
        if(_num_shown == 0) return;
        spdlog::log(LVL, "{}", _rule(_num_shown));
        _num_shown = 0;
    }
};

}  // namespace gecot

#endif  // GECOT_UTILS_LOG_TABLE_HPP
