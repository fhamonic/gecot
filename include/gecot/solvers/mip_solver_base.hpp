#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <format>
#include <functional>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/model_concepts.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/xpress/all.hpp"

#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/criterion.hpp"
#include "gecot/helper.hpp"
#include "gecot/solvers/solve_result.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// Backends whose models _set_numeric_focus can set; MOSEK has the setting
// for its MIP optimizer alone.
template <typename M>
inline constexpr bool has_numeric_focus =
    std::same_as<M, mippp::gurobi_milp> || std::same_as<M, mippp::gurobi_lp> ||
    std::same_as<M, mippp::cplex_milp> || std::same_as<M, mippp::cplex_lp> ||
    std::same_as<M, mippp::xpress_milp> || std::same_as<M, mippp::xpress_lp> ||
    std::same_as<M, mippp::mosek_milp>;

// What every algorithm on a solver shares: its session with the solver, the
// criterion and the options of a model, the MIP start.
struct mip_solver_base {
    // -t, in several roles: the feasibility tolerance of the solver; the
    // share of a flow by which a master may overestimate it before a cut;
    // the slack of the budget, for a MIP start, for the tight big-Ms, and in
    // path_lp for the columns of enumerate and the solution returned; how
    // far under 1 an option of the relaxation still counts as bought
    double feasibility_tol = default_feasibility_tol;
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

    // sw starts with solve(): the time limit also counts the preprocessing.
    // share: the part of the limit given to the solves of a step.
    bool _time_up(const spdlog::stopwatch & sw, const double share = 1) const {
        return time_limit && sw.elapsed().count() > share * time_limit.value();
    }
    template <typename M>
    void _set_time_left(M & model, const spdlog::stopwatch & sw,
                        const double share = 1) const {
        if constexpr(mippp::has_time_limit<M>)
            if(time_limit)
                model.set_time_limit(std::chrono::duration<double>(std::max(
                    0.0, share * time_limit.value() - sw.elapsed().count())));
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

    // why the start is no solution, in the words of a message; empty if it
    // is one
    template <instance_c I>
    std::string_view _start_defect(const I & instance, const auto & start,
                                   const budget_t & budget) const {
        for(std::size_t r = 0; r < budget.size(); ++r)
            if(compute_solution_cost(instance, start, r) >
               budget[r] + feasibility_tol)
                return "exceeds the budget";
        if(!is_feasible(instance, start, budget, feasibility_tol))
            return "buys two options of a conflict set";
        return {};
    }
    template <instance_c I>
    instance_solution_t<I> _fallback_solution(const I & instance,
                                              const budget_t & budget) const {
        auto solution = _mip_start_solution(instance);
        if(const auto defect = _start_defect(instance, solution, budget);
           !defect.empty())
            throw std::runtime_error(std::format("the MIP start {}", defect));
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
