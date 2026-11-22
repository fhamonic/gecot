#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/model_concepts.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/solvers/flow_preprocessing.hpp"
#include "gecot/solvers/solve_result.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// Backends whose candidate solutions can be cut by lazy constraints and
// handed back with the scores of their options (Benders). With the two
// tolerances and the MIP start, which the base sets through helpers that
// skip a missing one: a backend that lost a tolerance would keep its default
// without a word, and one that lost the start would only warn.
template <typename M>
concept lazy_constraint_model =
    mippp::has_optimality_tolerance<M> && mippp::has_feasibility_tolerance<M> &&
    mippp::has_mip_start<M> && mippp::has_candidate_solution_callback<M> &&
    mippp::has_lazy_constraints<mippp::candidate_solution_callback_handle_t<M>,
                                M> &&
    mippp::has_solution_injection<
        mippp::candidate_solution_callback_handle_t<M>, M>;

// What the Benders decompositions share: the search of benders.md, with or
// without lazy constraints.
struct benders_base : public flow_preprocessing {
    // off for comparison: the search of a Benders decomposition is then the
    // one of a solver without lazy constraints
    bool lazy_constraints = true;

    // The search of a Benders decomposition, steps 2 to 4 of benders.md.
    // cuts(candidate, y, add) writes in candidate, a point of the master,
    // its other variables at the scenario y (f_t and f^i_t, or theta_g),
    // calls add with the cut at y of each block that candidate overvalues,
    // and returns the scores of the cases at y. add ignores what may follow
    // the cut: the hook of a row that the rounds of multiflow_benders may
    // remove, which the search never does. With lazy constraints, in a
    // callback of the solver; without, the master solved again with the cuts
    // of its solution as rows, until it receives none. No solution if the
    // solver found none: the caller then returns _fallback_solution.
    template <typename M, instance_c I>
    std::pair<std::optional<instance_solution_t<I>>, solve_report>
    _benders_search(M & model, const spdlog::stopwatch & sw, const I & instance,
                    const budget_t & budget, const auto & x, const auto & z,
                    const auto & formula, auto && cuts) const {
        if constexpr(lazy_constraint_model<M>)
            if(lazy_constraints)
                return _callback_search(model, sw, instance, x, z, formula,
                                        cuts);
        return _resolve_search(model, sw, instance, budget, x, cuts);
    }

    // _benders_search in a callback of the solver, the cuts as lazy
    // constraints
    template <lazy_constraint_model M, instance_c I>
    std::pair<std::optional<instance_solution_t<I>>, solve_report>
    _callback_search(M & model, const spdlog::stopwatch & sw,
                     const I & instance, const auto & x, const auto & z,
                     const auto & formula, auto && cuts) const {
        std::size_t num_cuts = 0;
        model.set_candidate_solution_callback(
            [&](typename M::candidate_solution_callback_handle & handle) {
                auto candidate = handle.get_solution();
                const auto y = _bought_options(instance, candidate, x);
                const std::size_t num_cuts_before = num_cuts;
                const auto case_scores = cuts(candidate, y, [&](auto && cut,
                                                                auto &&...) {
                    ++num_cuts;
                    handle.add_lazy_constraint(
                        std::forward<decltype(cut)>(cut));
                });
                if(num_cuts > num_cuts_before)
                    _hand_back(handle, candidate, instance, x, y, z,
                               case_scores, formula);
            });
        auto [has_solution, report] = _solve(model, sw);
        report.stats.emplace_back("lazy_constraints", num_cuts);
        spdlog::debug("Cuts on the candidates of the search: {}", num_cuts);
        if(!has_solution) return {std::nullopt, report};
        return {_bought_options(instance, model.get_solution(), x), report};
    }

    // _benders_search by solving the master again, the cuts as rows
    template <typename M, instance_c I>
    std::pair<std::optional<instance_solution_t<I>>, solve_report>
    _resolve_search(M & model, const spdlog::stopwatch & sw, const I & instance,
                    const budget_t & budget, const auto & x,
                    auto && cuts) const {
        namespace status = mippp::status;
        std::size_t num_cuts = 0;
        const auto build_time = sw.elapsed();
        // the best scenario, by its score; the bound, the lowest of those
        // of the masters
        std::optional<instance_solution_t<I>> best;
        double best_score = 0;
        std::optional<double> bound;
        std::size_t num_solves = 0, nodes = 0;
        // the solver's score, that of a master which may overvalue its
        // solution, is not shown
        const bool solver_log = _set_search(model, [&](progress p) {
            p.score = best ? std::optional{best_score} : std::nullopt;
            if(bound) p.bound = std::min(*bound, p.bound.value_or(*bound));
            if(p.nodes) *p.nodes += nodes;
            return p;
        });
        solve_report report = _model_report(model, {}, build_time);
        // the score of y; values is the point of the master, which cuts
        // reads and writes in as the point of a callback
        auto score_of = [&](std::vector<double> values, const auto & y,
                            auto && add) {
            mippp::entity_mapping<mippp::model_variable_t<M>,
                                  std::vector<double>>
                candidate(std::move(values));
            return instance.eval_criterion(cuts(candidate, y, add));
        };
        // the MIP start is the first best, as the first incumbent of a
        // callback: its cuts are already rows, and a master that overvalues
        // another scenario would never return it
        if(mip_start)
            if(auto start = _mip_start_solution(instance);
               _start_defect(instance, start, budget).empty()) {
                best_score =
                    score_of(std::vector<double>(model.num_variables()), start,
                             [](auto &&, auto &&...) {});
                best = std::move(start);
            }
        // the times the masters proposed each scenario. The second time,
        // all its cuts are added: the first added only those of the blocks
        // that the master overvalued then, and another one can overvalue
        // another block. A third time, the master overvalues it only by the
        // tolerances of the solver.
        std::map<std::vector<bool>, int> proposed;
        std::string_view reason;
        for(bool optimal = false; !optimal;) {
            _set_time_left(model, sw);
            model.solve();
            ++num_solves;
            const auto & s = model.get_status();
            reason = _status_text(s);
            if(!status::solution_available(s)) break;
            if constexpr(mippp::has_dual_bound<M>)
                if(const double b = model.get_dual_bound();
                   b < model.infinity())
                    bound = std::min(b, bound.value_or(b));
            if constexpr(mippp::has_node_count<M>)
                nodes += static_cast<std::size_t>(model.get_node_count());
            const auto solution = model.get_solution();
            std::vector<double> values(model.num_variables());
            for(std::size_t j = 0; j < values.size(); ++j)
                values[j] = solution[mippp::model_variable_t<M>(
                    static_cast<decltype(x(0).id())>(j))];
            auto y = _bought_options(instance, solution, x);
            const int times = ++proposed[std::ranges::to<std::vector<bool>>(
                std::views::transform(
                    instance.options(),
                    [&y](const option_t i) { return bool{y[i]}; }))];
            // above any value: every block exceeds it, and receives its cut
            if(times == 2)
                std::ranges::fill(values,
                                  std::numeric_limits<double>::infinity());
            const std::size_t num_cuts_before = num_cuts;
            const double score =
                score_of(std::move(values), y, [&](auto && cut, auto &&...) {
                    ++num_cuts;
                    model.add_constraint(std::forward<decltype(cut)>(cut));
                });
            if(!best || score > best_score) {
                best_score = score;
                best = std::move(y);
                // the start of the next solve, if the loaded library has the
                // function: a HiGHS without Highs_setSparseSolution throws
                if constexpr(mippp::has_mip_start<M>) {
                    try {
                        model.add_mip_start(std::views::transform(
                            instance.options(), [&](const option_t i) {
                                return std::make_pair(x(i),
                                                      (*best)[i] ? 1.0 : 0.0);
                            }));
                    } catch(const mippp::feature_unavailable_error &) {
                    }
                }
            }
            // no row among the lines of the solver's log
            report_progress(
                solver_log ? progress{}
                           : progress{.score = best_score, .bound = bound});
            // the master's value is the score of its solution, or it has
            // no better solution than the best
            optimal = (std::holds_alternative<status::optimal>(s) &&
                       (num_cuts == num_cuts_before || times > 2)) ||
                      (bound &&
                       *bound <= best_score + mip_gap * std::abs(best_score));
            if(optimal)
                reason = "optimal";
            else if(!std::holds_alternative<status::optimal>(s))
                break;
            else if(stop_requested) {
                reason = "interrupted";
                break;
            } else if(time_limit && sw.elapsed().count() >= *time_limit) {
                reason = "time limit reached";
                break;
            }
        }
        report.status = std::string(reason);
        report.stats.emplace_back("lazy_constraints", num_cuts);
        report.stats.emplace_back("master_solves", num_solves);
        if constexpr(mippp::has_node_count<M>)
            report.stats.emplace_back("nodes", nodes);
        if(best) {
            report.best_bound = bound;
            report_progress({.score = best_score, .bound = bound, .end = true});
            spdlog::log(reason == "optimal" ? spdlog::level::debug
                                            : spdlog::level::warn,
                        "Solver stopped: {}", reason);
        } else
            _no_solution(reason);
        spdlog::debug(
            "Cuts on the candidates of the search: {}, master solved {} times",
            num_cuts, num_solves);
        return {std::move(best), report};
    }

    // Gives a candidate that received cuts back to the solver, as the
    // scenario y with the scores of its cases: the solver drops such a
    // candidate and may never propose its options again, although they are
    // a solution. The caller has already written in candidate the other
    // variables of its master at y (f_t and f^i_t, or theta_g): the solver
    // refuses a point that breaks a row.
    template <instance_c I>
    static void _hand_back(auto & handle, auto & candidate, const I & instance,
                           const auto & x, const auto & y, const auto & z,
                           const auto & case_scores, const auto & formula) {
        for(auto && i : instance.options()) candidate[x(i)] = y[i];
        for(auto && instance_case : instance.cases())
            candidate[z(instance_case.id())] = case_scores[instance_case.id()];
        formula.set_values(candidate, case_scores);
        handle.add_solution(candidate);
    }
};

}  // namespace solvers
}  // namespace gecot
