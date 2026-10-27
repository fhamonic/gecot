#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <variant>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/criterion.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_constrained_strong_and_useless_arcs.hpp"
#include "gecot/preprocessing/compute_strong_and_useless_arcs.hpp"
#include "gecot/solvers/solve_result.hpp"

namespace gecot {
namespace solvers {

struct mip_solver_base {
    using model_type = mippp::gurobi_milp;
    // using model_type = mippp::cplex_milp;
    // using model_type = mippp::copt_milp;

    double feasibility_tol = 0.0;
    double probability_resolution = 1e-8;
    int num_mus = 0;
    std::optional<std::vector<std::string>> mip_start;
    std::optional<double> time_limit;

    template <typename M, typename V>
    struct formula_variable_visitor {
        std::reference_wrapper<M> model;
        std::reference_wrapper<const V> C_vars;

        formula_variable_visitor(M & t_model, const V & t_C_vars)
            : model{t_model}, C_vars{t_C_vars} {}

        auto operator()(const criterion_constant & c) {
            return model.get().add_variable(
                {.lower_bound = c, .upper_bound = c});
        }
        auto operator()(const criterion_var & v) { return C_vars.get()(v); }
        auto operator()(const criterion_sum & f) {
            using namespace mippp::operators;
            auto var = model.get().add_variable();
            std::vector<mippp::model_variable_t<M>> vars;
            for(auto && e : f.values) vars.emplace_back(std::visit(*this, e));
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

            auto var = model.get().add_variable();
            model.get().add_constraint(
                var == std::get<criterion_constant>(f.values[0]) *
                           std::visit(*this, f.values[1]));
            return var;
        }
        auto operator()(const criterion_min & f) {
            using namespace mippp::operators;
            auto var = model.get().add_variable();
            for(auto && e : f.values) {
                model.get().add_constraint(var <= std::visit(*this, e));
            }
            return var;
        }
    };

    template <instance_c I, case_c C>
    auto _compute_strong_and_useless_arcs(const I & instance,
                                          const C & instance_case,
                                          const double budget) const {
        if(num_mus >= 1) {
            return compute_constrained_strong_and_useless_arcs(
                instance, instance_case, budget,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution, num_mus);
        } else {
            return compute_strong_and_useless_arcs(
                instance_case,
                [&instance, budget](const option_t & o) {
                    return instance.option_cost(o) <= budget;
                },
                probability_resolution);
        }
    }

    // sw starts with solve(): the time limit also counts the preprocessing
    // returns false if the solver found no solution, the caller then returns
    // _fallback_solution
    template <typename M>
    std::pair<bool, solve_report> _solve(M & model,
                                         const spdlog::stopwatch & sw) const {
        namespace status = mippp::status;
        const auto build_time = sw.elapsed();
        if(time_limit)
            model.set_time_limit(std::chrono::duration<double>(
                std::max(0.0, time_limit.value() - build_time.count())));
        // with scores near 1e11, gurobi's absolute tolerances otherwise prune
        // better solutions; scaling the models is the real fix
        if constexpr(std::same_as<M, mippp::gurobi_milp>)
            model.native_api().setintparam(model.native_model().first,
                                           "NumericFocus", 1);
        model.solve();
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
        if constexpr(std::same_as<M, mippp::gurobi_milp>) {
            const auto & GRB = model.native_api();
            const auto native_model = model.native_model().second;
            double value;
            if(!GRB.getdblattr(native_model, "ObjBound", &value) &&
               value < model.infinity())
                report.best_bound = value;
            if(!GRB.getdblattr(native_model, "NodeCount", &value))
                report.stats.emplace_back("nodes", value);
        }
        if(!status::solution_available(s)) {
            if(!mip_start)
                throw std::runtime_error(fmt::format(
                    "the solver stopped without a solution: {}", reason));
            spdlog::warn(
                "Solver stopped without a solution: {}, returning "
                "the MIP start",
                reason);
            return {false, report};
        }
        spdlog::log(std::holds_alternative<status::optimal>(s)
                        ? spdlog::level::info
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
