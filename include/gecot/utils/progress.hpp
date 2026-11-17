#ifndef GECOT_UTILS_PROGRESS_HPP
#define GECOT_UTILS_PROGRESS_HPP

#include <atomic>
#include <cstddef>
#include <format>
#include <functional>
#include <optional>
#include <string>

namespace gecot {

// a state of a running algorithm, its scores in the unit of the instance it
// was given
struct progress {
    std::optional<double> score = {};       // of its best solution so far
    std::optional<double> bound = {};       // proven, on the best score
    std::optional<double> relaxation = {};  // a relaxation's value, no bound
    std::optional<std::size_t> nodes = {};  // of the branch-and-bound
    std::optional<std::size_t> round = {};  // of a loop of rounds
    // of the cuts of a round: the smallest tolerance at which it adds none
    std::optional<double> violation = {};
    // The end of a step of the progress: sent before the algorithm logs
    // anything else, or these lines come before the rule that closes the
    // table of its progress. A search sends its last values with it; a loop
    // of rounds reports each relaxation it solves, the last one included, as
    // a round, then a bare end.
    bool end = false;
};

// Called by the algorithms while they run, possibly on a solver's thread but
// never twice at once. It must not throw: it can run inside a solver's C code.
inline std::function<void(const progress &)> report_progress =
    [](const progress &) {};

// The texts of the gains that the algorithms log, as the summary prints them:
// gain_text of a score, over the initial score of the instance read
// ("+22.05 %"), change_text of a change of score, in % of that initial score.
// progress_watch sets them, since an algorithm knows neither that score nor
// the unit of the instance read; without it, the values themselves. They
// allocate: never in a solver callback.
inline std::function<std::string(double)> gain_text = [](const double score) {
    return std::format("{:g}", score);
};
inline std::function<std::string(double)> change_text = gain_text;

// Set, also from a signal handler, to stop the running algorithm: it returns
// its best result so far, with status "interrupted". A MIP solver sees it only
// in its progress callback, which the solver may not call during its presolve
// or root relaxation (flow_mip on Quebec: 2 s after a stop asked at 1 s).
inline std::atomic<bool> stop_requested = false;

}  // namespace gecot

#endif  // GECOT_UTILS_PROGRESS_HPP
