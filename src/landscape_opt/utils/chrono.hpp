/**
 * @file chrono.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief Chrono class declaration
 * @version 0.1
 * @date 2020-10-27
 *
 * @copyright Copyright (c) 2020
 */
#ifndef CHRONO_HPP
#define CHRONO_HPP

#include <chrono>

/**
 * @brief A simple chronometer class
 */
class Chrono {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time,
        last_time;

public:
    Chrono()
        : start_time(std::chrono::high_resolution_clock::now())
        , last_time(std::chrono::high_resolution_clock::now()) {}

    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
        last_time = std::chrono::high_resolution_clock::now();
    }
private:
    template <class chrono_unit>
    int time() {
        std::chrono::time_point<std::chrono::high_resolution_clock>
            current_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<chrono_unit>(current_time -
                                                       start_time)
            .count();
    }
public:
    int timeUs() { return time<std::chrono::microseconds>(); }
    int timeMs() { return time<std::chrono::milliseconds>(); }
    int timeS() { return time<std::chrono::seconds>(); }

    template <class chrono_unit>
    int lapTime() {
        std::chrono::time_point<std::chrono::high_resolution_clock>
            current_time = std::chrono::high_resolution_clock::now();
        int time =
            std::chrono::duration_cast<chrono_unit>(current_time - last_time)
                .count();
        last_time = current_time;
        return time;
    }
    int lapTimeUs() { return lapTime<std::chrono::microseconds>(); }
    int lapTimeMs() { return lapTime<std::chrono::milliseconds>(); }
    int lapTimeS() { return lapTime<std::chrono::seconds>(); }
};

#endif  // CHRONO