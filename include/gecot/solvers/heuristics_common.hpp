#ifndef GECOT_SOLVERS_HEURISTICS_COMMON_HPP
#define GECOT_SOLVERS_HEURISTICS_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "melon/graph.hpp"
#include "melon/mapping.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/parallel_pc_num.hpp"
#include "gecot/utils/progress_bar.hpp"

namespace gecot {
namespace solvers {

// What static and greedy share, of docs/formulations/heuristics.md.

// --decremental reads a single budget and no conflict set
template <instance_c I>
void check_decremental(const I & instance, const std::string & name) {
    if(!has_plain_budget(instance))
        throw std::invalid_argument(
            name +
            " --decremental: conflict sets and several cost dimensions are "
            "not supported, run " +
            name + " without --decremental");
}

// a conflict set of the option i holds another option that the solution buys
template <instance_c I>
bool is_blocked(const I & instance, const auto & options_sets,
                const auto & solution, const option_t i) {
    return std::ranges::any_of(options_sets[i], [&](auto && l) {
        return std::ranges::any_of(
            instance.conflict_sets()[l],
            [&](auto && other) { return other != i && solution[other]; });
    });
}

// greedy takes as tied two gains per unit of cost closer than this fraction
// of the score, per unit of cost; static ranks without the window. A gain is
// the difference of two scores: two options that gain the same get gains
// that differ in their last digits, and without the window these digits
// would choose between them, where the rule is the first one.
inline constexpr double tie_window = 1e-12;

// The rank of an option of cost 0, whose gain per unit of cost would be
// infinite or undefined: first if it gains (1), last if it loses (-1), and
// otherwise with the options that cost (0), as a gain of 0.
inline int free_option_tier(const double cost, const double gain,
                            const double score) {
    if(cost != 0) return 0;
    const double eps = tie_window * std::abs(score);
    return (gain > eps) - (gain < -eps);
}
// what orders the options of a tier
inline double ratio_key(const double cost, const double ratio,
                        const double score) {
    return cost == 0 && free_option_tier(cost, ratio, score) == 0 ? 0.0 : ratio;
}

// The rank of the options by their ratios at a score, both read at each
// call: by tier, then by key. Orders i1 before i2 if i1 ranks lower.
template <typename Costs, typename Ratios>
struct option_ranking {
    const Costs & costs;
    const Ratios & ratios;
    const double & score;

    int tier(const option_t i) const {
        return free_option_tier(costs[i], ratios[i], score);
    }
    double key(const option_t i) const {
        return ratio_key(costs[i], ratios[i], score);
    }
    bool operator()(const option_t i1, const option_t i2) const {
        const int t1 = tier(i1), t2 = tier(i2);
        return t1 != t2 ? t1 < t2 : key(i1) < key(i2);
    }
};

// Where both heuristics start: the options that fit alone are bought with
// decremental, at the cost purchased, and are the free ones otherwise.
template <instance_c I>
struct heuristic_start {
    instance_solution_t<I> solution;
    std::vector<option_t> options, free_options;
    double purchased = 0.0;

    heuristic_start(const I & instance, const budget_t & budget,
                    const double feasibility_tol, const bool decremental)
        : solution(instance.create_option_map(false)) {
        for(const option_t & i : instance.options())
            if(fits_alone(instance, i, budget, feasibility_tol))
                (decremental ? options : free_options).emplace_back(i);
        for(const option_t & i : options) {
            solution[i] = true;
            purchased += instance.option_cost(i);
        }
    }
};

// The landscape of each case with some options bought, in which the
// heuristics evaluate the others.
template <instance_c I>
struct heuristic_landscape {
    const I & instance;
    // per case and option: its vertices with their quality gains, its arcs
    // with their improved probability
    decltype(compute_cases_vertex_options(
        std::declval<const I &>())) cases_vertex_options;
    decltype(compute_cases_arc_options(
        std::declval<const I &>())) cases_arc_options;
    // per case, with the options bought. Empty until set().
    decltype(std::declval<const I &>()
                 .template create_case_map<instance_source_quality_map_t<I>>())
        cases_source_quality_map;
    decltype(std::declval<const I &>()
                 .template create_case_map<instance_target_quality_map_t<I>>())
        cases_target_quality_map;
    decltype(std::declval<const I &>()
                 .template create_case_map<instance_probability_map_t<I>>())
        cases_probability_map;

    explicit heuristic_landscape(const I & t_instance)
        : instance(t_instance)
        , cases_vertex_options(compute_cases_vertex_options(instance))
        , cases_arc_options(compute_cases_arc_options(instance))
        , cases_source_quality_map(
              instance
                  .template create_case_map<instance_source_quality_map_t<I>>())
        , cases_target_quality_map(
              instance
                  .template create_case_map<instance_target_quality_map_t<I>>())
        , cases_probability_map(
              instance
                  .template create_case_map<instance_probability_map_t<I>>()) {}

    // The option i bought in the three maps of a case.
    void _buy(const auto & instance_case, const option_t i,
              auto & source_quality_map, auto & target_quality_map,
              auto & probability_map) const {
        const case_id_t k = instance_case.id();
        for(auto && [u, source_gain, target_gain] :
            cases_vertex_options[k][i]) {
            source_quality_map[u] += source_gain;
            target_quality_map[u] += target_gain;
        }
        for(auto && [a, improved_probability] : cases_arc_options[k][i])
            probability_map[a] =
                std::max(probability_map[a], improved_probability);
    }
    // The option i removed from the three maps of a case. solution: the
    // options bought, with or without i.
    void _remove(const auto & instance_case, const option_t i,
                 const auto & solution, auto & source_quality_map,
                 auto & target_quality_map, auto & probability_map) const {
        const case_id_t k = instance_case.id();
        for(auto && [u, source_gain, target_gain] :
            cases_vertex_options[k][i]) {
            source_quality_map[u] -= source_gain;
            target_quality_map[u] -= target_gain;
        }
        for(auto && [a, _] : cases_arc_options[k][i]) {
            probability_map[a] = instance_case.arc_probability_map()[a];
            for(auto && [improved_probability, j] :
                instance_case.arc_options_map()[a]) {
                if(!solution[j] || j == i) continue;
                probability_map[a] =
                    std::max(probability_map[a], improved_probability);
            }
        }
    }

    // The landscape with exactly these options bought. In their order: a
    // quality is a sum of gains, and its rounding follows the order.
    void set(auto && bought_options) {
        for(const auto & instance_case : instance.cases()) {
            const case_id_t k = instance_case.id();
            cases_source_quality_map[k] = instance_case.source_quality_map();
            cases_target_quality_map[k] = instance_case.target_quality_map();
            cases_probability_map[k] = instance_case.arc_probability_map();
            for(auto && i : bought_options) buy(instance_case, i);
        }
    }
    void buy(const auto & instance_case, const option_t i) {
        const case_id_t k = instance_case.id();
        _buy(instance_case, i, cases_source_quality_map[k],
             cases_target_quality_map[k], cases_probability_map[k]);
    }
    void buy(const option_t i) {
        for(const auto & instance_case : instance.cases())
            buy(instance_case, i);
    }
    // solution: the options bought, with or without i
    void remove(const option_t i, const auto & solution) {
        for(const auto & instance_case : instance.cases()) {
            const case_id_t k = instance_case.id();
            _remove(instance_case, i, solution, cases_source_quality_map[k],
                    cases_target_quality_map[k], cases_probability_map[k]);
        }
    }

    double score() const {
        return compute_score(instance, cases_source_quality_map,
                             cases_target_quality_map, cases_probability_map);
    }

    // options_cases_pc_num[i][k], for each of the options: the score of the
    // case k in its landscape after change(instance_case, i, its three maps)
    template <detail::range_of<option_t> O>
    void _scores(const O & options, auto & options_cases_pc_num,
                 auto && change) const {
        const auto & cases = instance.cases();
        progress_bar<spdlog::level::debug, 64> pb(options.size() *
                                                  cases.size());
        tbb::parallel_for(
            tbb::blocked_range2d(cases.begin(), cases.end(), options.begin(),
                                 options.end()),
            [&](const tbb::blocked_range2d<decltype(cases.begin()),
                                           decltype(options.begin())> &
                    cases_options_block) {
                for(const auto & instance_case : cases_options_block.rows()) {
                    const auto & options_block = cases_options_block.cols();
                    if(options_block.begin() == options_block.end()) continue;
                    const case_id_t k = instance_case.id();

                    auto source_quality_map = cases_source_quality_map[k];
                    auto target_quality_map = cases_target_quality_map[k];
                    auto probability_map = cases_probability_map[k];

                    for(auto it = options_block.begin();;) {
                        const option_t i = *it;
                        change(instance_case, i, source_quality_map,
                               target_quality_map, probability_map);

                        options_cases_pc_num[i][k] = parallel_pc_num(
                            instance_case.graph(), source_quality_map,
                            target_quality_map, probability_map);

                        pb.tick();
                        if(++it == options_block.end()) break;

                        // by assignment: (a + g) - g is not always a, and
                        // the score of the next option would depend on the
                        // ones before it in the block, so on the threads
                        for(auto && [u, source_gain, target_gain] :
                            cases_vertex_options[k][i]) {
                            source_quality_map[u] =
                                cases_source_quality_map[k][u];
                            target_quality_map[u] =
                                cases_target_quality_map[k][u];
                        }
                        for(auto && [a, _] : cases_arc_options[k][i])
                            probability_map[a] = cases_probability_map[k][a];
                    }
                }
            });
    }

    // options_cases_pc_num[i][k], for each of the free options: the score of
    // the case k with i bought too
    template <detail::range_of<option_t> O>
    void scores_with(const O & free_options,
                     auto & options_cases_pc_num) const {
        _scores(free_options, options_cases_pc_num,
                [this](const auto & instance_case, const option_t i,
                       auto &... maps) { _buy(instance_case, i, maps...); });
    }

    // options_cases_pc_num[i][k], for each of the taken options, which the
    // solution buys: the score of the case k without i
    template <melon::mapping<option_t> S, detail::range_of<option_t> O>
        requires std::convertible_to<melon::mapped_value_t<S, option_t>, bool>
    void scores_without(const S & solution, const O & taken_options,
                        auto & options_cases_pc_num) const {
        _scores(
            taken_options, options_cases_pc_num,
            [&](const auto & instance_case, const option_t i, auto &... maps) {
                _remove(instance_case, i, solution, maps...);
            });
    }
};

}  // namespace solvers
}  // namespace gecot

#endif  // GECOT_SOLVERS_HEURISTICS_COMMON_HPP
