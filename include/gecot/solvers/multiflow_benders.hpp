#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/parallel_for.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_big_M_map.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/benders_base.hpp"
#include "gecot/solvers/multiflow_target_block.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The decomposition of docs/formulations/benders.md with a flow per source
// and target. Each block of _with_relaxation and solve() builds what the
// section it names states.
struct multiflow_benders : public benders_base {
    // off for comparison: no cut on the linear relaxation before the
    // search
    bool root_cuts = true;
    // the share of the way to each new optimum of the relaxation that the
    // center of the rounds moves, from the first optimum; 1: each optimum
    // separated itself, without the center
    double in_out_ratio = 0.5;
    // off: the sets A_s are not kept but rebuilt at each separation: the
    // same cuts, slower, for instances whose sets do not fit in memory
    bool stored_arcs = true;
    // A cut is added to the relaxation when f_t exceeds it by more than
    // root_cut_tol of it, and by root_cut_abs_tol: the rounds end when none
    // is. Neither follows feasibility_tol. Under 1e-7, the default -t, a
    // round can add again a cut that the relaxation holds: the test reads x
    // clamped to [0,1], and f_t as the solver returns it.
    double root_cut_tol = 1e-7;
    static constexpr double root_cut_abs_tol = 1e-7;
    // The rounds also end once their bound is within root_cut_gap of their
    // floor, relative to it, as path_lp's relative_gap; 0: never.
    double root_cut_gap = 0;
    // the rounds take at most this share of --time-limit, which leaves the
    // search the other half
    static constexpr double root_time_share = 0.5;

    // the sources of every block; returns their number
    template <typename B>
    std::size_t _compute_sources(std::vector<B> & blocks) const {
        tbb::parallel_for(std::size_t{0}, blocks.size(),
                          [&](const std::size_t b) {
                              blocks[b].compute_sources(stored_arcs);
                          });
        std::size_t num_sources = 0, num_source_arcs = 0;
        for(const B & block : blocks) {
            num_sources += block.sources.size();
            for(const auto & source : block.sources)
                num_source_arcs += source.arcs.size();
        }
        spdlog::debug("Targets: {}, source-target pairs: {}, stored arcs: {}",
                      blocks.size(), num_sources, num_source_arcs);
        return num_sources;
    }

    template <typename B>
    std::vector<typename B::cut> _separate(
        const std::vector<B> & blocks,
        const melon::mapping<option_t> auto & x_hat) const {
        std::vector<typename B::cut> cuts(blocks.size());
        tbb::parallel_for(std::size_t{0}, blocks.size(),
                          [&](const std::size_t b) {
                              cuts[b] = blocks[b].separate(x_hat, !stored_arcs);
                          });
        return cuts;
    }
    // The search, steps 2 and 3: the cut at the scenario y of each block
    // whose f_t in candidate exceeds f_t(y), which it writes there, its own
    // variable from its thread. Not inline in solve(): there, the loop is
    // compiled again for each backend and each add of _benders_search.
    template <typename B>
    std::vector<std::optional<typename B::cut>> _overvalued_cuts(
        const std::vector<B> & blocks, const melon::mapping<option_t> auto & y,
        auto & candidate) const {
        std::vector<std::optional<typename B::cut>> cuts(blocks.size());
        tbb::parallel_for(
            std::size_t{0}, blocks.size(), [&](const std::size_t b) {
                const B & block = blocks[b];
                const double flow = block.flow_at(y);
                if(std::exchange(candidate[block.f_t], flow) - flow <=
                   feasibility_tol * flow)
                    return;
                cuts[b] = block.separate(y, !stored_arcs);
            });
        return cuts;
    }

    // The value of the point x̂ separated, with the flows of the cuts, in the
    // relaxation with all the cuts: x̂ is within the budget, and no cut is
    // under its flow. None if a flow did not settle: it is then a bound.
    template <instance_c I, typename B>
    static std::optional<double> _floor_at(
        const I & instance, const std::vector<B> & blocks,
        const std::vector<double> & x_hat,
        const std::vector<typename B::cut> & cuts) {
        if(!std::ranges::all_of(cuts, &B::cut::settled)) return std::nullopt;
        auto case_scores = instance.create_case_map(0.0);
        for(const auto & [block, cut] : std::views::zip(blocks, cuts)) {
            const double flow = std::min(cut.value, block.flow_bound);
            double score = block.target_quality * flow;
            for(const auto & [target_gain, i, big_M, f_ti] :
                block.target_quality_gains)
                score += target_gain * std::min(flow, big_M * x_hat[i]);
            case_scores[block.case_id] += score;
        }
        return instance.eval_criterion(case_scores);
    }

    // of the rounds on the relaxation: their number, that of their cuts, and
    // the value of the last relaxation solved, none if no round ended
    struct root_rounds {
        std::size_t rounds = 0, cuts = 0;
        std::optional<double> value;
        // x* of that relaxation, within [0, 1]: what multiflow_rounding
        // rounds
        std::vector<double> optimum;
    };
    // The rounds of cuts on the linear relaxation, for which the options x
    // are continuous: each solves the master, separates its optimum, or the
    // center that moves in_out_ratio of the way to it, and adds as a row
    // cut_constraint(block, cut) for each block whose f_t exceeds its cut.
    template <typename M, instance_c I, typename B>
    root_rounds _root_rounds(M & model, const spdlog::stopwatch & sw,
                             const I & instance, const auto & x,
                             const std::vector<B> & blocks,
                             auto && cut_constraint) const {
        namespace status = mippp::status;
        root_rounds root;
        // a Ctrl-C before the rounds stops the whole run
        const bool stopped_before = stop_requested;
        for(auto && i : instance.options()) model.set_continuous(x(i));
        // the relaxation with all the cuts lies between root_floor and
        // root.value
        std::optional<double> root_floor;
        std::vector<double> center(instance.options().size());
        double in_out_coef = in_out_ratio;
        while(root_cuts && !stop_requested && !_time_up(sw, root_time_share)) {
            _set_time_left(model, sw, root_time_share);
            model.solve();
            if(!mippp::is_a<status::optimal>(model.get_status())) break;
            ++root.rounds;
            root.value = model.get_solution_value();
            // a copy, which the rows added below leave readable
            const auto relaxation = model.get_solution();
            auto optimum = [&](const option_t i) {
                return std::clamp(relaxation[x(i)], 0.0, 1.0);
            };
            root.optimum = std::ranges::to<std::vector>(
                std::views::transform(instance.options(), optimum));
            if(root.rounds == 1) {
                for(auto && i : instance.options()) center[i] = optimum(i);
            } else {
                for(auto && i : instance.options())
                    center[i] = (1 - in_out_coef) * center[i] +
                                in_out_coef * optimum(i);
            }
            const auto cuts = _separate(blocks, center);
            if(const auto floor = _floor_at(instance, blocks, center, cuts))
                root_floor = std::max(root_floor.value_or(*floor), *floor);
            // the smallest --root-cut-tolerance at which no cut is added
            double violation = 0;
            std::size_t num_added = 0;
            for(const auto & [block, cut] : std::views::zip(blocks, cuts)) {
                const double f_at_optimum = relaxation[block.f_t];
                const double cut_at_optimum = std::ranges::fold_left(
                    std::views::transform(instance.options(),
                                          [&](const option_t i) {
                                              return cut.coefficients[i] *
                                                     optimum(i);
                                          }),
                    cut.constant, std::plus{});
                if(f_at_optimum > cut_at_optimum + root_cut_abs_tol)
                    violation = std::max(
                        violation,
                        (f_at_optimum - root_cut_abs_tol) / cut_at_optimum - 1);
                // not `<=` then skip: a NaN --root-cut-tolerance, which the
                // CLI's range lets through, would add every cut forever
                if(f_at_optimum >
                   cut_at_optimum * (1 + root_cut_tol) + root_cut_abs_tol) {
                    model.add_constraint(cut_constraint(block, cut));
                    ++num_added;
                }
            }
            report_progress({.bound = root.value,
                             .relaxation = root_floor,
                             .round = root.rounds,
                             .violation = violation});
            root.cuts += num_added;
            if(root_cut_gap > 0 && root_floor &&
               *root.value - *root_floor <= root_cut_gap * *root_floor)
                break;
            if(num_added) continue;
            if(in_out_coef == 1.0) break;
            in_out_coef = 1.0;
        }
        for(auto && i : instance.options()) model.set_binary(x(i));
        report_progress({.end = true});
        spdlog::debug("Rounds on the relaxation: {}, cuts: {}", root.rounds,
                      root.cuts);
        // A Ctrl-C during the rounds ends them only: what follows them goes
        // on, and the next Ctrl-C stops it
        if(stop_requested && !stopped_before) {
            stop_requested = false;
            spdlog::info(
                "Ctrl-C: the rounds on the relaxation end there, the "
                "next Ctrl-C stops what follows them");
        }
        return root;
    }

    // Builds the master and its blocks, adds the cuts of the MIP start and
    // runs the rounds on the relaxation, then returns the solution that
    // search(model, sw, x, z, formula, blocks, cut_constraint, root) finds
    // and its report, which receives the statistics of the blocks and of the
    // rounds. multiflow_rounding shares all but the search.
    template <mippp::has_column_generation M, instance_c I>
    solve_result<I> _with_relaxation(const I & instance,
                                     const budget_t & budget,
                                     auto && search) const {
        using namespace mippp;
        using namespace mippp::operators;
        // the subproblem of a target: its graph, its sources, its cuts, and
        // its variables in the master
        using target_block = multiflow_target_block<model_variable_t<M>>;
        using cut = typename target_block::cut;
        _check_criterion(instance);
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);
        // the rounds on the relaxation solve the model before _solve does
        _set_numeric_focus(model);

        // Cases and criterion, in flow.md: the scores z_k under their
        // criterion (Linearizing the criterion), the options x_i within the
        // budgets and the conflict sets
        const auto z = model.add_variables(instance.cases().size());
        model.set_maximization();
        formula_variable_visitor formula{model, z};
        model.set_objective(formula.variable(instance.criterion()));
        const auto x = model.add_binary_variables(instance.options());
        _add_budget_rows(model, instance, budget, x);

        std::vector<target_block> blocks;
        // with the vertices and the arcs of the graphs G_t, on average
        std::vector<preprocessing_row<double, double>> preprocessing;
        for(auto && instance_case : instance.cases()) {
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions of the '{}' graph",
                          instance_case.name());

            // flow_benders, the master: the score of the case, a row that
            // the columns of its f_t and f^i_t fill
            const case_id_t k = instance_case.id();
            const auto z_row = model.add_constraint(z(k) <= 0);
            progress_bar<spdlog::level::info, 64> pb(
                instance_case.graph().num_vertices());
            graph_sizes sizes;
            for(const auto & original_t :
                melon::vertices(instance_case.graph())) {
                pb.tick();
                if(!_is_target(instance_case, original_t)) continue;
                // the big-Ms of t only: those of the arcs are per source
                contracted_flow_graph G_t =
                    compute_contracted_flow_graph<false>(
                        instance, instance_case, budget, original_t,
                        strong_arcs_map[original_t],
                        useless_arcs_map[original_t], big_Ms, feasibility_tol,
                        pruning, _options_model<M>(instance, budget));
                sizes.add(G_t.graph);
                target_block & block = blocks.emplace_back(
                    instance_case, original_t, std::move(G_t));
                // multiflow_benders: f_t within M^t_t, without which the
                // relaxation is unbounded until t receives a cut, and f^i_t
                // for each positive target quality gain. A lowered constant
                // of i covers only the scenarios that buy i: no bound of f_t.
                block.f_t = model.add_column(
                    {std::make_pair(z_row, -block.target_quality)},
                    {.upper_bound = block.flow_bound});
                for(auto & [target_gain, i, big_M, f_ti] :
                    block.target_quality_gains) {
                    f_ti =
                        model.add_column({std::make_pair(z_row, -target_gain)});
                    model.add_constraint(f_ti <= block.f_t);
                    model.add_constraint(f_ti <= big_M * x(i));
                }
            }
            preprocessing.push_back(_sizes_row(instance_case, strong_arcs_map,
                                               useless_arcs_map, sizes));
        }
        _log_graph_sizes(preprocessing);
        const std::size_t num_sources = _compute_sources(blocks);

        // multiflow_benders: the cut of a block as a row. A lazy sum that
        // reads the cut: only valid while the cut lives.
        auto cut_constraint = [&](const target_block & block, const cut & c) {
            return block.f_t <=
                   c.constant + xsum(std::views::filter(
                                         instance.options(),
                                         [&c](const option_t i) {
                                             return c.coefficients[i] != 0;
                                         }),
                                     [&](const option_t i) {
                                         return c.coefficients[i] * x(i);
                                     });
        };
        // The search, step 1: the cuts of the MIP start
        if(const auto start = _add_mip_start(model, instance, x))
            for(const auto & [block, c] :
                std::views::zip(blocks, _separate(blocks, *start)))
                model.add_constraint(cut_constraint(block, c));

        spdlog::info("Solving the optimization model");
        // Cuts on the relaxation, before the search
        const root_rounds root =
            _root_rounds(model, sw, instance, x, blocks, cut_constraint);

        auto [solution, report] =
            search(model, sw, x, z, formula, blocks, cut_constraint, root);
        report.stats.emplace_back("sources", num_sources);
        report.stats.emplace_back("root_rounds", root.rounds);
        report.stats.emplace_back("root_cuts", root.cuts);
        // the relaxation with its cuts bounds the optimum too, better than
        // the solver's bound when the search is cut short
        if(root.value) {
            report.best_bound =
                std::min(report.best_bound.value_or(*root.value), *root.value);
            report.score_stats.emplace_back("root_relaxation_value",
                                            *root.value);
        }
        if(!solution) return {_fallback_solution(instance, budget), report};
        return {std::move(*solution), report};
    }

    template <mippp::has_column_generation M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        return _with_relaxation<M>(
            instance, budget,
            [&](auto & model, const auto & sw, const auto & x, const auto & z,
                const auto & formula, const auto & blocks,
                const auto & cut_constraint, const root_rounds &) {
                // The search, steps 2 to 4: the flows of a candidate y and the
                // cut at y of each target whose f_t exceeds its flow. The flows
                // need no cut, and the cuts are made for these targets only.
                auto cuts_at = [&](auto & candidate, const auto & y,
                                   auto && add) {
                    const auto overvalued =
                        _overvalued_cuts(blocks, y, candidate);
                    auto case_scores = instance.create_case_map(0.0);
                    for(const auto & [block, block_cut] :
                        std::views::zip(blocks, overvalued)) {
                        // f_t(y), which _overvalued_cuts left in candidate
                        const double flow = candidate[block.f_t];
                        double quality = block.target_quality;
                        for(const auto & [target_gain, i, big_M, f_ti] :
                            block.target_quality_gains) {
                            candidate[f_ti] = y[i] * flow;
                            quality += y[i] * target_gain;
                        }
                        case_scores[block.case_id] += quality * flow;
                        if(block_cut) add(cut_constraint(block, *block_cut));
                    }
                    return case_scores;
                };

                return _benders_search(model, sw, instance, budget, x, z,
                                       formula, cuts_at);
            });
    }
};

}  // namespace solvers
}  // namespace gecot
