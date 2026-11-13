#ifndef GECOT_UTILS_PROGRESS_BAR_HPP
#define GECOT_UTILS_PROGRESS_BAR_HPP

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <mutex>
#include <print>

#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

namespace gecot {

// on stderr when it is a terminal, ticked from any thread
template <spdlog::level::level_enum LVL, std::size_t BAR_LENGTH>
class progress_bar {
private:
    std::atomic<std::size_t> _current_ticks;
    std::size_t _final_ticks;
    const bool _shown;
    // workers finish out of order: only a longer bar is printed
    std::atomic<std::size_t> _printed_length;
    std::mutex _print_mutex;

private:
    void _print_bar(std::size_t progress) {
        std::print(stderr, "[{0:=^{1}}>{0: ^{2}}] {3:>3}%\r", "", progress,
                   BAR_LENGTH - progress, 100 * progress / BAR_LENGTH);
        fflush(stderr);
    }

public:
    progress_bar(const std::size_t num_ticks)
        : _current_ticks(0)
        , _final_ticks(num_ticks)
        , _shown(num_ticks > 0 && spdlog::should_log(LVL) &&
                 spdlog::details::os::in_terminal(stderr))
        , _printed_length(0) {
        if(_shown) _print_bar(0);
    }
    ~progress_bar() {
        if(!_shown) return;
        std::print(stderr, "{0: ^{1}}\r", "", BAR_LENGTH + 8);
        fflush(stderr);
    }
    void tick(const std::size_t incr = 1) {
        if(!_shown) return;
        const std::size_t length =
            BAR_LENGTH * (_current_ticks += incr) / _final_ticks;
        if(length <= _printed_length) return;
        std::lock_guard lock(_print_mutex);
        if(length <= _printed_length) return;
        _printed_length = length;
        _print_bar(length);
    }
};

}  // namespace gecot

#endif  // GECOT_UTILS_PROGRESS_BAR_HPP
