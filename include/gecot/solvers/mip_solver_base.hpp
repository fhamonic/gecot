#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <format>
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
#include "mippp/solvers/gurobi/all.hpp"

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

// backends whose candidate solutions can be cut by lazy constraints and
// handed back with the scores of their options (Benders)
template <typename M>
concept lazy_constraint_model =
    mippp::has_candidate_solution_callback<M> &&
    mippp::has_lazy_constraints<mippp::candidate_solution_callback_handle_t<M>,
                                M> &&
    mippp::has_solution_injection<
        mippp::candidate_solution_callback_handle_t<M>, M>;

struct mip_solver_base {
    double feasibility_tol = 0.0;
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
    std::optional<std::vector<std::string>> mip_start;
    std::optional<double> time_limit;

    // the variables of the sums, products and mins are free, {} rather than
    // the default lower bound 0: a + min(b - 10, 0) is negative below b = 10
    template <typename M, typename V>
    struct formula_variable_visitor {
        std::reference_wrapper<M> model;
        std::reference_wrapper<const V> C_vars;
        std::vector<
            std::pair<mippp::model_variable_t<M>, const criterion_formula *>>
            variables;

        formula_variable_visitor(M & t_model, const V & t_C_vars)
            : model{t_model}, C_vars{t_C_vars} {}

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
            auto var = model.get().add_variable({});
            std::vector<mippp::model_variable_t<M>> vars;
            for(auto && e : f.values) vars.emplace_back(variable(e));
            model.get().add_constraint(var == xsum(vars));
            return var;
        }
        auto operator()(const criterion_product & f) {
            using namespace mippp::operators;
            if(!std::holds_alternative<criterion_constant>(f.values[0]) ||
               f.values.size() != 2)
                throw std::invalid_argument(
                    "mip doesn't support products of variables in the "
                    "criterion "
                    "!");

            auto var = model.get().add_variable({});
            model.get().add_constraint(
                var == std::get<criterion_constant>(f.values[0]) *
                           variable(f.values[1]));
            return var;
        }
        auto operator()(const criterion_min & f) {
            using namespace mippp::operators;
            auto var = model.get().add_variable({});
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
               std::ranges::any_of(instance_case.vertex_options_map()[t],
                                   [](auto && e) {
                                       auto && [sqg, tqg, o] = e;
                                       return tqg > 0;
                                   });
    }

    // Both kinds of arcs are labelled together: contracting or removing
    // alone saves no labelling time.
    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const double budget) const {
        using arcs = tbb::concurrent_vector<melon::arc_t<case_graph_t<C>>>;
        auto no_arcs = [&instance_case] {
            return melon::create_vertex_map<arcs>(instance_case.graph());
        };
        if(!contract_strong_arcs && !remove_useless_arcs)
            return std::make_pair(no_arcs(), no_arcs());
        auto affordable = [&instance, budget](const option_t & o) {
            return instance.option_cost(o) <= budget;
        };
        auto labels = num_mus >= 1
                          ? compute_constrained_strong_and_useless_arcs(
                                instance, instance_case, budget, affordable,
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
            {"species"}, {"strong arcs per target", log_format::fixed(2)},
            {"useless arcs per target", log_format::fixed(2)},
            std::move(columns)...);
        for(const auto & values : cases)
            std::apply([&table](const auto &... v) { table.row(v...); },
                       values);
    }

    // sw starts with solve(): the time limit also counts the preprocessing
    // returns false if the solver found no solution, the caller then returns
    // _fallback_solution
    template <typename M>
    std::pair<bool, solve_report> _solve(M & model,
                                         const spdlog::stopwatch & sw) const {
        namespace status = mippp::status;
        const auto build_time = sw.elapsed();
        if constexpr(mippp::has_time_limit<M>) {
            if(time_limit)
                model.set_time_limit(std::chrono::duration<double>(
                    std::max(0.0, time_limit.value() - build_time.count())));
        } else if(time_limit)
            spdlog::warn("This solver has no time limit: --time-limit ignored");
        if constexpr(!mippp::has_optimality_tolerance<M>)
            spdlog::warn("This solver keeps its default optimality gap");
        if constexpr(std::same_as<M, mippp::gurobi_milp>) {
            const auto & GRB = model.native_api();
            const auto env = model.native_model().first;
            // absolute: at its default 1e-6, the bound ended 2e-8 below the
            // optimum on the scaled scores of trivial_reformulate_instance
            GRB.setdblparam(env, "OptimalityTol", 1e-9);
            // pruned better solutions before the scores were scaled
            GRB.setintparam(env, "NumericFocus", 1);
        }
        if constexpr(mippp::has_progress_callback<M>)
            model.set_progress_callback([](auto & handle) noexcept {
                if(stop_requested) {
                    handle.interrupt();
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
                report_progress(p);
            });
        // the solver's own log, on stdout
        if constexpr(mippp::has_verbosity<M>)
            model.set_verbose(spdlog::should_log(spdlog::level::trace));
        spdlog::info("Solving the optimization model");
        spdlog::debug(
            "The model has {} variables, {} constraints and {} "
            "nonzeros",
            model.num_variables(), model.num_constraints(),
            model.num_nonzeros());
        model.solve();
        report_progress({.end = true});
        const auto & s = model.get_status();
        const std::string_view reason = std::visit(
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
        solve_report report{
            std::string(reason),
            {},
            {{"variables", model.num_variables()},
             {"constraints", model.num_constraints()},
             {"nonzeros", model.num_nonzeros()},
             {"model_build_time_ms",
              std::chrono::duration_cast<std::chrono::milliseconds>(build_time)
                  .count()}}};
        if(!status::solution_available(s)) {
            if(!mip_start)
                throw std::runtime_error(std::format(
                    "the solver stopped without a solution: {}", reason));
            spdlog::warn(
                "Solver stopped without a solution: {}, returning "
                "the MIP start",
                reason);
            return {false, report};
        }
        // as the solution, defined only after a stop or at the optimum
        if constexpr(mippp::has_dual_bound<M>)
            if(const double bound = model.get_dual_bound();
               bound < model.infinity())
                report.best_bound = bound;
        if constexpr(mippp::has_node_count<M>)
            report.stats.emplace_back("nodes", model.get_node_count());
        spdlog::log(std::holds_alternative<status::optimal>(s)
                        ? spdlog::level::debug
                        : spdlog::level::warn,
                    "Solver stopped: {}", reason);
        return {true, report};
    }

    template <instance_c I>
    instance_solution_t<I> _fallback_solution(const I & instance,
                                              const double budget) const {
        auto solution = _mip_start_solution(instance);
        if(compute_solution_cost(instance, solution) > budget + feasibility_tol)
            throw std::runtime_error("the MIP start exceeds the budget");
        return solution;
    }

    // backends without these setters keep their defaults
    template <typename M>
    static void _set_tolerances(M & model, const double optimality_tol,
                                const double feasibility_tol) {
        if constexpr(mippp::has_optimality_tolerance<M>)
            model.set_optimality_tolerance(optimality_tol);
        if constexpr(mippp::has_feasibility_tolerance<M>)
            model.set_feasibility_tolerance(feasibility_tol);
    }

    template <typename M, instance_c I>
    void _add_mip_start(M & model, const I & instance,
                        const auto & X_vars) const {
        if(!mip_start) return;
        if constexpr(mippp::has_mip_start<M>) {
            instance_solution_t<I> mip_start_solution =
                _mip_start_solution(instance);
            model.add_mip_start(
                std::views::transform(instance.options(), [&](auto o) {
                    return std::make_pair(X_vars(o), mip_start_solution[o]);
                }));
        } else
            spdlog::warn("This solver has no MIP start: --mip-start ignored");
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
