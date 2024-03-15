/**
 * @file chronometer.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief chronometer class declaration
 * @version 0.1
 * @date 2020-10-27
 *
 * @copyright Copyright (c) 2020
 */
#ifndef CHRONOMETER_HPP
#define CHRONOMETER_HPP

#include <chrono>

/**
 * @brief A simple chronometer class
 */
class chronometer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time,
        last_time;

public:
    chronometer()
        : start_time(std::chrono::high_resolution_clock::now())
        , last_time(std::chrono::high_resolution_clock::now()) {}

    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
        last_time = std::chrono::high_resolution_clock::now();
    }

private:
    template <class chrono_unit>
    auto time() {
        std::chrono::time_point<std::chrono::high_resolution_clock>
            current_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<chrono_unit>(current_time -
                                                       start_time)
            .count();
    }

public:
    auto time_us() { return time<std::chrono::microseconds>(); }
    auto time_ms() { return time<std::chrono::milliseconds>(); }
    auto time_s() { return time<std::chrono::seconds>(); }

    template <class chrono_unit>
    auto lap_time() {
        std::chrono::time_point<std::chrono::high_resolution_clock>
            current_time = std::chrono::high_resolution_clock::now();
        auto time =
            std::chrono::duration_cast<chrono_unit>(current_time - last_time)
                .count();
        last_time = current_time;
        return time;
    }
    auto lap_time_us() { return lap_time<std::chrono::microseconds>(); }
    auto lap_time_ms() { return lap_time<std::chrono::milliseconds>(); }
    auto lap_time_s() { return lap_time<std::chrono::seconds>(); }
};

#endif  // CHRONOMETER_HPP