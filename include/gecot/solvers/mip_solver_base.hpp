#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>

#include "mippp/model_concepts.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/xpress/all.hpp"

#include "melon/graph.hpp"

#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/criterion.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/solvers/solve_result.hpp"
#include "gecot/utils/log_table.hpp"
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

// Backends whose models _set_numeric_focus can set; MOSEK has the setting
// for its MIP optimizer alone.
template <typename M>
inline constexpr bool has_numeric_focus =
    std::same_as<M, mippp::gurobi_milp> || std::same_as<M, mippp::gurobi_lp> ||
    std::same_as<M, mippp::cplex_milp> || std::same_as<M, mippp::cplex_lp> ||
    std::same_as<M, mippp::xpress_milp> || std::same_as<M, mippp::xpress_lp> ||
    std::same_as<M, mippp::mosek_milp>;

struct mip_solver_base {
    // -t, in several roles: the feasibility tolerance of the solver; the
    // share of a flow by which a master may overestimate it before a cut;
    // the slack of the budget, for a MIP start, for the tight big-Ms, and in
    // path_lp for the columns of enumerate and the solution returned; how
    // far under 1 an option of the relaxation still counts as bought
    double feasibility_tol = default_feasibility_tol;
    double probability_resolution = 1e-8;
    int num_mus = 0;
    bool decades_sweep = true;
    // For comparison, a preprocessing step switched off: the contraction of
    // the strong arcs, the removal of the useless arcs, the skipping of the
    // vertices that can never be targets, and in the graph of a target the
    // removal of the arcs that leave it and of the vertices no quality
    // reaches.
    bool contract_strong_arcs = true;
    bool remove_useless_arcs = true;
    bool all_targets = false;
    bool no_pruning = false;
    // for comparison, the search of a Benders decomposition as on a solver
    // without lazy constraints
    bool no_lazy_constraints = false;
    std::optional<std::vector<std::string>> mip_start;
    std::optional<double> time_limit;
    // the solver asked for more care in its numerical computations
    bool numeric_focus = false;

    template <typename M, typename V>
    struct formula_variable_visitor {
        std::reference_wrapper<M> model;
        std::reference_wrapper<const V> C_vars;
        std::vector<
            std::pair<mippp::model_variable_t<M>, const criterion_formula *>>
            variables;

        formula_variable_visitor(M & t_model, const V & t_C_vars)
            : model{t_model}, C_vars{t_C_vars} {}

        // The variable of a sum, a product or a min: free, not with the
        // default lower bound 0, as a + min(b - 10, 0) is negative below
        // b = 10.
        auto free_variable() { return model.get().add_variable({}); }

        // the variable of the formula e, kept for set_values
        auto variable(const criterion_formula & e) {
            const auto var = std::visit(*this, e);
            variables.emplace_back(var, &e);
            return var;
        }
        // Gives the variables of the formulas their values at the cases'
        // scores: a point handed to the solver is refused if a sum or a
        // product does not hold.
        void set_values(auto & point, const auto & case_scores) const {
            for(const auto & [var, e] : variables)
                point[var] = std::visit(formula_eval_visitor{case_scores}, *e);
        }

        auto operator()(const criterion_constant & c) {
            return model.get().add_variable(
                {.lower_bound = c, .upper_bound = c});
        }
        auto operator()(const criterion_var & v) { return C_vars.get()(v); }
        auto operator()(const criterion_sum & f) {
            using namespace mippp::operators;
            auto var = free_variable();
            std::vector<mippp::model_variable_t<M>> vars;
            for(auto && e : f.values) vars.emplace_back(variable(e));
            model.get().add_constraint(var == xsum(vars));
            return var;
        }
        // the literal constants of the product times its one other
        // factor, wherever they stand: the scaled criterion of
        // trivial_reformulate_instance puts its weight in front of [w, z]
        auto operator()(const criterion_product & f) {
            using namespace mippp::operators;
            double weight = 1;
            const criterion_formula * formula = nullptr;
            for(auto && e : f.values) {
                if(const auto * k = std::get_if<criterion_constant>(&e))
                    weight *= *k;
                else if(formula)
                    throw std::invalid_argument(
                        "mip doesn't support products of variables in the "
                        "criterion !");
                else
                    formula = &e;
            }
            if(!formula) return (*this)(criterion_constant{weight});
            auto var = free_variable();
            model.get().add_constraint(var == weight * variable(*formula));
            return var;
        }
        auto operator()(const criterion_min & f) {
            using namespace mippp::operators;
            auto var = free_variable();
            for(auto && e : f.values) {
                model.get().add_constraint(var <= variable(e));
            }
            return var;
        }
    };

    // The models bound the case scores, and the minimums of the criterion,
    // from above only. Under a criterion that decreases with a case score
    // their optimum leaves this score at 0 whatever the options bought, and
    // is returned as optimal.
    template <instance_c I>
    static void _check_criterion(const I & instance) {
        if(!has_non_decreasing_criterion(instance))
            throw std::invalid_argument(
                "the criterion is not non-decreasing in the case scores");
    }

    // a vertex that some options make a target
    template <case_c C>
    bool _is_target(const C & instance_case,
                    const melon::vertex_t<case_graph_t<C>> & t) const {
        return all_targets || instance_case.target_quality_map()[t] > 0 ||
               std::ranges::any_of(
                   instance_case.vertex_options_map()[t], [](auto && e) {
                       auto && [source_gain, target_gain, i] = e;
                       return target_gain > 0;
                   });
    }

    // Both kinds of arcs are labelled together: contracting or removing
    // alone saves no labelling time.
    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const budget_t & budget) const {
        using arcs = tbb::concurrent_vector<melon::arc_t<case_graph_t<C>>>;
        auto no_arcs = [&instance_case] {
            return melon::create_vertex_map<arcs>(instance_case.graph());
        };
        if(!contract_strong_arcs && !remove_useless_arcs)
            return std::make_pair(no_arcs(), no_arcs());
        auto affordable = [&instance, &budget](const option_t & i) {
            return fits_alone(instance, i, budget);
        };
        // on the cost dimension 0: dropping the others relaxes the problem
        auto labels = num_mus >= 1
                          ? compute_constrained_strong_and_useless_arcs(
                                instance, instance_case, budget[0], affordable,
                                probability_resolution, num_mus, decades_sweep,
                                all_targets)
                          : compute_strong_and_useless_arcs(
                                instance_case, affordable,
                                probability_resolution, all_targets);
        if(!contract_strong_arcs) labels.first = no_arcs();
        if(!remove_useless_arcs) labels.second = no_arcs();
        return labels;
    }

    // the arcs that a target of the case has in arcs_map, on average
    template <case_c C>
    double _average_size(const C & instance_case, const auto & arcs_map) const {
        std::size_t num_arcs = 0, num_targets = 0;
        for(auto && t : melon::vertices(instance_case.graph())) {
            if(!_is_target(instance_case, t)) continue;
            num_arcs += arcs_map[t].size();
            ++num_targets;
        }
        return static_cast<double>(num_arcs) /
               static_cast<double>(std::max<std::size_t>(num_targets, 1));
    }

    // of a case: its name, the _average_size of its strong and of its
    // useless arcs, then the values of the algorithm's own columns
    template <typename... V>
    using preprocessing_row = std::tuple<std::string, double, double, V...>;

    // Logs at -v the table of the cases' preprocessing, once they all are
    // preprocessed: a row logged after each case would be cut from the
    // others by the steps of the next one.
    template <typename... V, std::size_t... WIDTHS>
    static void _log_preprocessing(
        const std::vector<preprocessing_row<V...>> & cases,
        log_column<WIDTHS>... columns) {
        log_table<spdlog::level::debug, 0, 12, 11, 12, WIDTHS...> table(
            {"case"}, {"strong arcs per target", log_format::fixed(2)},
            {"useless arcs per target", log_format::fixed(2)},
            std::move(columns)...);
        for(const auto & values : cases)
            std::apply([&table](const auto &... v) { table.row(v...); },
                       values);
    }

    // the vertices and the arcs of the graphs G_t of a case
    struct graph_sizes {
        std::size_t num_graphs = 0, num_vertices = 0, num_arcs = 0;
        void add(const auto & graph) {
            ++num_graphs;
            num_vertices += graph.num_vertices();
            num_arcs += graph.num_arcs();
        }
    };
    // the row of a case for _log_graph_sizes
    template <case_c C>
    preprocessing_row<double, double> _sizes_row(
        const C & instance_case, const auto & strong_arcs_map,
        const auto & useless_arcs_map, const graph_sizes & sizes) const {
        const auto n =
            static_cast<double>(std::max<std::size_t>(sizes.num_graphs, 1));
        return {instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map),
                static_cast<double>(sizes.num_vertices) / n,
                static_cast<double>(sizes.num_arcs) / n};
    }
    // _log_preprocessing with the graphs G_t, on average
    static void _log_graph_sizes(
        const std::vector<preprocessing_row<double, double>> & cases) {
        _log_preprocessing(
            cases, log_column<10>{"vertices per target", log_format::fixed(2)},
            log_column<10>{"arcs per target", log_format::fixed(2)});
    }

    // sw starts with solve(): the time limit also counts the preprocessing
    template <typename M>
    void _set_time_left(M & model, const spdlog::stopwatch & sw) const {
        if constexpr(mippp::has_time_limit<M>)
            if(time_limit)
                model.set_time_limit(std::chrono::duration<double>(
                    std::max(0.0, time_limit.value() - sw.elapsed().count())));
    }
    // The settings of a search, its time limit apart, and its progress
    // callback, which reports the values of the solver through adjust.
    // Returns whether the solver prints its own log.
    template <typename M>
    bool _set_search(M & model, auto adjust) const {
        if constexpr(!mippp::has_time_limit<M>)
            if(time_limit)
                spdlog::warn(
                    "This solver has no time limit: --time-limit ignored");
        if constexpr(!mippp::has_optimality_tolerance<M>)
            spdlog::warn("This solver keeps its default optimality gap");
        if constexpr(std::same_as<M, mippp::gurobi_milp>) {
            const auto & GRB = model.native_api();
            const auto env = model.native_model().first;
            // absolute: at its default 1e-6, the bound ended 2e-8 below the
            // optimum on the scaled scores of trivial_reformulate_instance
            GRB.setdblparam(env, "OptimalityTol", 1e-9);
        }
        _warn_without_numeric_focus<M>("MIP");
        _set_numeric_focus(model);
        // the solver's own log, on stdout, which shows the progress itself
        const bool solver_log =
            mippp::has_verbosity<M> && spdlog::should_log(spdlog::level::trace);
        if constexpr(mippp::has_verbosity<M>) model.set_verbose(solver_log);
        if constexpr(mippp::has_progress_callback<M>)
            model.set_progress_callback(
                [solver_log, adjust](auto & handle) noexcept {
                    if(stop_requested) {
                        handle.interrupt();
                        return;
                    }
                    // a row would print among the lines of the solver's log:
                    // a progress without values logs only the line on Ctrl-C
                    if(solver_log) {
                        report_progress({});
                        return;
                    }
                    // infinite until the solver has one
                    auto finite = [](const double v) {
                        return std::isfinite(v) ? std::optional<double>{v}
                                                : std::nullopt;
                    };
                    progress p{.score = finite(handle.get_solution_value()),
                               .bound = finite(handle.get_dual_bound())};
                    if constexpr(mippp::has_node_count<
                                     std::remove_cvref_t<decltype(handle)>>)
                        p.nodes = handle.get_node_count();
                    report_progress(adjust(p));
                });
        spdlog::debug("Model: {} variables, {} constraints, {} nonzeros",
                      model.num_variables(), model.num_constraints(),
                      model.num_nonzeros());
        return solver_log;
    }
    static std::string_view _status_text(const auto & s) {
        namespace status = mippp::status;
        return std::visit(
            []<typename S>(const S &) -> std::string_view {
                if constexpr(std::derived_from<S, status::optimal>)
                    return "optimal";
                else if constexpr(std::derived_from<S, status::time_limit>)
                    return "time limit reached";
                else if constexpr(std::derived_from<S, status::limit_reached>)
                    return "limit reached";
                else if constexpr(std::derived_from<S, status::infeasible>)
                    return "infeasible model";
                else if constexpr(std::derived_from<S, status::unbounded>)
                    return "unbounded model";
                else if constexpr(std::derived_from<
                                      S, status::infeasible_or_unbounded>)
                    return "infeasible or unbounded model";
                else if constexpr(std::derived_from<S, status::interrupted>)
                    return "interrupted";
                else if constexpr(std::derived_from<S, status::failed>)
                    return "solver failure";
                else
                    return "unknown status";
            },
            s);
    }
    template <typename M>
    static solve_report _model_report(M & model, const std::string_view status,
                                      const auto build_time) {
        return {
            std::string(status),
            {},
            {{"variables", model.num_variables()},
             {"constraints", model.num_constraints()},
             {"nonzeros", model.num_nonzeros()},
             {"model_build_time_ms",
              std::chrono::duration_cast<std::chrono::milliseconds>(build_time)
                  .count()}}};
    }
    // the end of a search that found no solution: the caller then returns
    // _fallback_solution
    void _no_solution(const std::string_view reason) const {
        report_progress({.end = true});
        if(!mip_start)
            throw std::runtime_error(std::format(
                "the solver stopped without a solution: {}", reason));
        spdlog::warn("Solver stopped: {}, MIP start kept", reason);
    }

    // returns false if the solver found no solution, the caller then returns
    // _fallback_solution
    template <typename M>
    std::pair<bool, solve_report> _solve(M & model,
                                         const spdlog::stopwatch & sw) const {
        namespace status = mippp::status;
        const auto build_time = sw.elapsed();
        _set_time_left(model, sw);
        _set_search(model, std::identity{});
        model.solve();
        const auto & s = model.get_status();
        const std::string_view reason = _status_text(s);
        solve_report report = _model_report(model, reason, build_time);
        if(!status::solution_available(s)) {
            _no_solution(reason);
            return {false, report};
        }
        // as the solution, defined only after a stop or at the optimum
        progress last{.score = model.get_solution_value(), .end = true};
        if constexpr(mippp::has_dual_bound<M>)
            if(const double bound = model.get_dual_bound();
               bound < model.infinity())
                report.best_bound = last.bound = bound;
        if constexpr(mippp::has_node_count<M>) {
            last.nodes = model.get_node_count();
            report.stats.emplace_back("nodes", *last.nodes);
        }
        report_progress(last);
        spdlog::log(std::holds_alternative<status::optimal>(s)
                        ? spdlog::level::debug
                        : spdlog::level::warn,
                    "Solver stopped: {}", reason);
        return {true, report};
    }

    // The search of a Benders decomposition, steps 2 to 4 of benders.md.
    // cuts(candidate, y, add) writes in candidate, a point of the master,
    // its other variables at the scenario y (f_t and f^i_t, or theta_g),
    // calls add with the cut at y of each block that candidate overvalues,
    // and returns the scores of the cases at y. With lazy constraints, in a
    // callback of the solver; without, the master solved again with the cuts
    // of its solution as rows, until it receives none. No solution if the
    // solver found none: the caller then returns _fallback_solution.
    template <typename M, instance_c I>
    std::pair<std::optional<instance_solution_t<I>>, solve_report>
    _benders_search(M & model, const spdlog::stopwatch & sw, const I & instance,
                    const budget_t & budget, const auto & x, const auto & z,
                    const auto & formula, auto && cuts) const {
        namespace status = mippp::status;
        std::size_t num_cuts = 0;
        if constexpr(lazy_constraint_model<M>)
            if(!no_lazy_constraints) {
                model.set_candidate_solution_callback(
                    [&](typename M::candidate_solution_callback_handle &
                            handle) {
                        auto candidate = handle.get_solution();
                        const auto y = _bought_options(instance, candidate, x);
                        const std::size_t num_cuts_before = num_cuts;
                        const auto case_scores =
                            cuts(candidate, y, [&](auto && cut) {
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
                spdlog::debug("Cuts on the candidates of the search: {}",
                              num_cuts);
                if(!has_solution) return {std::nullopt, report};
                return {_bought_options(instance, model.get_solution(), x),
                        report};
            }

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
               is_feasible(instance, start, budget, feasibility_tol)) {
                best_score =
                    score_of(std::vector<double>(model.num_variables()), start,
                             [](auto &&) {});
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
                score_of(std::move(values), y, [&](auto && cut) {
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

    template <instance_c I>
    instance_solution_t<I> _fallback_solution(const I & instance,
                                              const budget_t & budget) const {
        auto solution = _mip_start_solution(instance);
        for(std::size_t r = 0; r < budget.size(); ++r)
            if(compute_solution_cost(instance, solution, r) >
               budget[r] + feasibility_tol)
                throw std::runtime_error("the MIP start exceeds the budget");
        if(!is_feasible(instance, solution, budget, feasibility_tol))
            throw std::runtime_error(
                "the MIP start buys two options of a conflict set");
        return solution;
    }

    // a row per cost dimension, then a row per conflict set
    template <typename M, instance_c I>
    static void _add_budget_rows(M & model, const I & instance,
                                 const budget_t & budget, const auto & x) {
        using namespace mippp;
        using namespace mippp::operators;
        if(budget.size() != instance.num_cost_dimensions())
            throw std::invalid_argument(
                "the budget needs a value per cost dimension");
        for(std::size_t r = 0; r < budget.size(); ++r)
            model.add_constraint(xsum(instance.options(), [&](auto && i) {
                                     return instance.option_cost(i, r) * x(i);
                                 }) <= budget[r]);
        for(auto && set : instance.conflict_sets())
            model.add_constraint(xsum(set, x) <= 1);
    }

    // the options_model of --big-m exact (compute_exact_big_M_map): a model
    // of M with the options and their rows, at the tolerances of the solver;
    // none for a solver without a dual bound
    template <typename M, instance_c I>
    auto _options_model(const I & instance, const budget_t & budget) const {
        if constexpr(mippp::has_dual_bound<M> && std::move_constructible<M>)
            return [this, &instance, &budget] {
                M model;
                _set_tolerances(model, mip_gap, feasibility_tol);
                const auto x = model.add_binary_variables(
                    instance.options().size(),
                    [](const option_t & i) { return i; });
                _add_budget_rows(model, instance, budget, x);
                return std::make_pair(std::move(model), x);
            };
        else
            return nullptr;
    }

    // gap: the relative MIP gap, which mippp calls the optimality tolerance;
    // Gurobi's OptimalityTol is another parameter, set in _solve. Backends
    // without these setters keep their defaults.
    // With numeric_focus, the setting of the solver for the numerical care
    // of model, at its first level above the default. The ids are those of
    // the solvers' headers, which mippp's tables do not name.
    template <typename M>
    void _set_numeric_focus(M & model) const {
        if(!numeric_focus) return;
        int error = 0;
        if constexpr(std::same_as<M, mippp::gurobi_milp> ||
                     std::same_as<M, mippp::gurobi_lp>)
            error = model.native_api().setintparam(model.native_model().first,
                                                   "NumericFocus", 1);
        else if constexpr(std::same_as<M, mippp::cplex_milp> ||
                          std::same_as<M, mippp::cplex_lp>)
            // CPXPARAM_Emphasis_Numerical, which has no other level
            error = model.native_api().setintparam(model.native_model().first,
                                                   1083, 1);
        else if constexpr(std::same_as<M, mippp::xpress_milp> ||
                          std::same_as<M, mippp::xpress_lp>)
            // XPRS_NUMERICALEMPHASIS: its default is -1, automatic, and 0
            // asks for speed
            error =
                model.native_api().setintcontrol(model.native_model(), 8416, 1);
        else if constexpr(std::same_as<M, mippp::mosek_milp>)
            // MSK_IPAR_MIO_NUMERICAL_EMPHASIS_LEVEL. The id is the position
            // of the name in an enum ordered by names, as are those that
            // mippp writes out for MOSEK (100, 110): a parameter added or
            // removed before it would move these too, which the versions
            // that mippp validates rule out.
            error = static_cast<int>(model.native_api().putintparam(
                model.native_model().second,
                static_cast<mippp::mosek::impl::v1::MSKiparame>(84), 1));
        if(error)
            throw std::runtime_error(std::format(
                "--numeric-focus: the solver refused its setting (error {})",
                error));
    }
    // once per kind of model, not per model: tree_lp has one per target
    template <typename M>
    void _warn_without_numeric_focus(const std::string_view models) const {
        if(numeric_focus && !has_numeric_focus<M>)
            spdlog::warn("--numeric-focus ignored by this solver's {} models",
                         models);
    }

    template <typename M>
    static void _set_tolerances(M & model, const double gap,
                                const double feasibility_tol) {
        if constexpr(mippp::has_optimality_tolerance<M>)
            model.set_optimality_tolerance(gap);
        if constexpr(mippp::has_feasibility_tolerance<M>)
            model.set_feasibility_tolerance(feasibility_tol);
    }

    // returns the start, if one is given, for the cuts a decomposition
    // adds at it
    template <typename M, instance_c I>
    std::optional<instance_solution_t<I>> _add_mip_start(M & model,
                                                         const I & instance,
                                                         const auto & x) const {
        if(!mip_start) return std::nullopt;
        instance_solution_t<I> start = _mip_start_solution(instance);
        if constexpr(mippp::has_mip_start<M>)
            model.add_mip_start(std::views::transform(
                instance.options(),
                [&](auto i) { return std::make_pair(x(i), start[i]); }));
        else
            spdlog::warn(
                "This solver has no MIP start: used if it finds no solution");
        return start;
    }

    // a binary variable at the value the solver returns: 0 or 1 within its
    // integrality tolerance
    static bool _is_bought(const double value) { return value > 0.5; }

    template <instance_c I>
    static instance_solution_t<I> _bought_options(const I & instance,
                                                  const auto & model_solution,
                                                  const auto & x) {
        instance_solution_t<I> solution = instance.create_option_map(false);
        for(auto && i : instance.options())
            solution[i] = _is_bought(model_solution[x(i)]);
        return solution;
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

    template <instance_c I>
    instance_solution_t<I> _mip_start_solution(const I & instance) const {
        instance_solution_t<I> solution = instance.create_option_map(false);
        for(const std::string & option_name : mip_start.value()) {
            if(!instance.contains_option(option_name)) continue;
            solution[instance.option_from_name(option_name)] = true;
        }
        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
