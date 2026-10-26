#ifndef GECOT_UTILS_PROGRESS_BAR_HPP
#define GECOT_UTILS_PROGRESS_BAR_HPP

#include <atomic>
#include <cstddef>
#include <cstdio>

#include <fmt/core.h>
#include <spdlog/spdlog.h>

namespace gecot {

template <spdlog::level::level_enum LVL, std::size_t BAR_LENGTH>
class progress_bar {
private:
    std::atomic<std::size_t> _current_ticks;
    std::size_t _final_ticks;

private:
    void _print_bar(std::size_t progress) {
        fmt::print("[{0:=^{1}}>{0: ^{2}}] {3:>3}%\r", "", progress,
                   BAR_LENGTH - progress, 100 * progress / BAR_LENGTH);
        fflush(stdout);
    }

public:
    progress_bar(const std::size_t num_ticks)
        : _current_ticks(0), _final_ticks(num_ticks) {
        if(spdlog::get_level() > LVL) return;
        _print_bar(0);
    }
    ~progress_bar() {
        if(spdlog::get_level() > LVL) return;
        fmt::print("{0: ^{1}}\r", "", BAR_LENGTH + 8);
        fflush(stdout);
    }
    void tick(const std::size_t incr = 1) {
        const std::size_t ticks = (_current_ticks += incr);
        if(spdlog::get_level() > LVL) return;
        if((BAR_LENGTH * ticks) % _final_ticks <= BAR_LENGTH) {
            _print_bar(BAR_LENGTH * ticks / _final_ticks);
        }
    }
    void reset() { _current_ticks = 0; }
};

}  // namespace gecot

#endif  // GECOT_UTILS_PROGRESS_BAR_HPP
