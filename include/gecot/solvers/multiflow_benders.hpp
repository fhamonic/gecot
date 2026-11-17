#pragma once

#include <algorithm>
#include <cstddef>
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
// and target. Each block of solve() builds what the section it names states.
struct multiflow_benders : public benders_base {
    // off for comparison: no cut on the linear relaxation before the
    // search, and each optimum of the relaxation separated itself, without
    // the center
    bool root_cuts = true;
    bool in_out = true;
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

    // the subproblem of a target: its graph, its sources, its cuts
    using target_block = multiflow_target_block<>;

    // the sources of every block; returns their number
    std::size_t _compute_sources(std::vector<target_block> & blocks) const {
        tbb::parallel_for(std::size_t{0}, blocks.size(),
                          [&](const std::size_t b) {
                              blocks[b].compute_sources(stored_arcs);
                          });
        std::size_t num_sources = 0, num_source_arcs = 0;
        for(const target_block & block : blocks) {
            num_sources += block.sources.size();
            for(const auto & source : block.sources)
                num_source_arcs += source.arcs.size();
        }
        spdlog::debug("Targets: {}, source-target pairs: {}, stored arcs: {}",
                      blocks.size(), num_sources, num_source_arcs);
        return num_sources;
    }

    std::vector<target_block::cut> _separate(
        const std::vector<target_block> & blocks,
        const std::vector<double> & x_hat) const {
        std::vector<target_block::cut> cuts(blocks.size());
        tbb::parallel_for(std::size_t{0}, blocks.size(),
                          [&](const std::size_t b) {
                              cuts[b] = blocks[b].separate(x_hat, !stored_arcs);
                          });
        return cuts;
    }

    // The value of the point x̂ separated, with the flows of the cuts, in the
    // relaxation with all the cuts: x̂ is within the budget, and no cut is
    // under its flow. None if a flow did not settle: it is then a bound.
    template <instance_c I>
    static std::optional<double> _floor_at(
        const I & instance, const std::vector<target_block> & blocks,
        const std::vector<double> & x_hat,
        const std::vector<target_block::cut> & cuts) {
        auto case_scores = instance.create_case_map(0.0);
        for(std::size_t b = 0; b < blocks.size(); ++b) {
            if(!cuts[b].settled) return std::nullopt;
            const double flow = std::min(cuts[b].value, blocks[b].flow_bound);
            double score = blocks[b].target_quality * flow;
            for(const auto & [target_gain, i, big_M] :
                blocks[b].target_quality_gains)
                score += target_gain * std::min(flow, big_M * x_hat[i]);
            case_scores[blocks[b].case_id] += score;
        }
        return instance.eval_criterion(case_scores);
    }

    // of the rounds on the relaxation: their number, that of their cuts, and
    // the value of the last relaxation solved, none if no round ended
    struct root_rounds {
        std::size_t rounds = 0, cuts = 0;
        std::optional<double> value;
    };
    // The rounds of cuts on the linear relaxation, for which the options x
    // are continuous: each solves the master, separates its optimum, or the
    // center that moves halfway to it, and adds as a row cut_constraint(b,
    // cut) for each f[b] that exceeds its cut.
    template <typename M, instance_c I>
    root_rounds _root_rounds(M & model, const spdlog::stopwatch & sw,
                             const I & instance, const auto & x, const auto & f,
                             const std::vector<target_block> & blocks,
                             auto && cut_constraint) const {
        namespace status = mippp::status;
        root_rounds root;
        for(auto && i : instance.options()) model.set_continuous(x(i));
        // the relaxation with all the cuts lies between root_floor and
        // root.value
        std::optional<double> root_floor;
        std::vector<double> point(instance.options().size()), center,
            f_relaxed(blocks.size());
        for(bool centered = in_out;
            root_cuts && !stop_requested && !_time_up(sw, root_time_share);) {
            _set_time_left(model, sw, root_time_share);
            model.solve();
            if(!std::holds_alternative<status::optimal>(model.get_status()))
                break;
            ++root.rounds;
            root.value = model.get_solution_value();
            {
                // readable until the model changes
                const auto relaxation = model.get_solution();
                for(auto && i : instance.options())
                    point[i] = std::clamp(relaxation[x(i)], 0.0, 1.0);
                for(std::size_t b = 0; b < blocks.size(); ++b)
                    f_relaxed[b] = relaxation[f[b]];
            }
            if(centered) {
                if(center.empty()) center = point;
                for(auto && i : instance.options())
                    center[i] = (center[i] + point[i]) / 2;
            }
            const auto & separated = centered ? center : point;
            const auto cuts = _separate(blocks, separated);
            if(const auto floor = _floor_at(instance, blocks, separated, cuts))
                root_floor = std::max(root_floor.value_or(*floor), *floor);
            // the smallest --root-cut-tolerance at which no cut is added
            double violation = 0;
            std::size_t num_added = 0;
            for(std::size_t b = 0; b < blocks.size(); ++b) {
                double cut_at_optimum = cuts[b].constant;
                for(auto && i : instance.options())
                    cut_at_optimum += cuts[b].coefficients[i] * point[i];
                if(f_relaxed[b] > cut_at_optimum + root_cut_abs_tol)
                    violation = std::max(
                        violation,
                        (f_relaxed[b] - root_cut_abs_tol) / cut_at_optimum - 1);
                // not `<=` then skip: a NaN --root-cut-tolerance, which the
                // CLI's range lets through, would add every cut forever
                if(f_relaxed[b] >
                   cut_at_optimum * (1 + root_cut_tol) + root_cut_abs_tol) {
                    model.add_constraint(cut_constraint(b, cuts[b]));
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
            if(!centered) break;
            centered = false;
        }
        for(auto && i : instance.options()) model.set_binary(x(i));
        report_progress({.end = true});
        spdlog::debug("Rounds on the relaxation: {}, cuts: {}", root.rounds,
                      root.cuts);
        return root;
    }

    template <mippp::has_column_generation M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using cut = target_block::cut;
        _check_criterion(instance);
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);
        // the rounds on the relaxation solve the model before _solve does
        _set_numeric_focus(model);

        // Cases and criterion, in flow.md: the scores z_k under their
        // criterion (Linearizing the criterion), the options x_i within the
        // budgets and the conflict sets
        const auto z = model.add_variables(
            instance.cases().size(), [](const case_id_t & k) { return k; });
        model.set_maximization();
        formula_variable_visitor formula{model, z};
        model.set_objective(formula.variable(instance.criterion()));
        const auto x = model.add_binary_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        _add_budget_rows(model, instance, budget, x);

        std::vector<target_block> blocks;
        // f_t block by block, and the f^i_t: block by block, in the order of
        // the block's target quality gains
        std::vector<model_variable_t<M>> f, f_gains;
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
                // multiflow_benders: f_t within M^t_t, without which the
                // relaxation is unbounded until t receives a cut, and f^i_t
                // for each positive target quality gain, in the order of
                // the block's target_quality_gains. A lowered constant of i
                // covers only the scenarios that buy i: no bound of f_t.
                f.push_back(model.add_column(
                    {std::make_pair(
                        z_row,
                        -instance_case.target_quality_map()[original_t])},
                    {.upper_bound = G_t.big_M_map.value()[G_t.t]}));
                for(const auto & [source_gain, target_gain, i] :
                    instance_case.vertex_options_map()[original_t]) {
                    if(!(target_gain > 0)) continue;
                    const auto f_ti = f_gains.emplace_back(model.add_column(
                        {std::make_pair(z_row, -target_gain)}));
                    model.add_constraint(f_ti <= f.back());
                    model.add_constraint(f_ti <= G_t.target_big_M(i) * x(i));
                }
                sizes.add(G_t.graph);
                blocks.emplace_back(instance_case, original_t, std::move(G_t));
            }
            preprocessing.push_back(_sizes_row(instance_case, strong_arcs_map,
                                               useless_arcs_map, sizes));
        }
        _log_graph_sizes(preprocessing);
        const std::size_t num_sources = _compute_sources(blocks);

        // multiflow_benders: the cut of the block b as a row. A lazy sum
        // that reads the cut: only valid while the cut lives.
        auto cut_constraint = [&](const std::size_t b, const cut & c) {
            return f[b] <=
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
        if(const auto start = _add_mip_start(model, instance, x)) {
            std::vector<double> point(instance.options().size());
            for(auto && i : instance.options()) point[i] = (*start)[i];
            const auto cuts = _separate(blocks, point);
            for(std::size_t b = 0; b < blocks.size(); ++b)
                model.add_constraint(cut_constraint(b, cuts[b]));
        }

        spdlog::info("Solving the optimization model");
        // Cuts on the relaxation, before the search
        const root_rounds root =
            _root_rounds(model, sw, instance, x, f, blocks, cut_constraint);

        // The search, steps 2 to 4: the flows of a candidate y and the cut at
        // y of each target whose f_t exceeds its flow
        auto cuts_at = [&](auto & candidate, const auto & bought, auto && add) {
            std::vector<double> y(instance.options().size());
            for(auto && i : instance.options()) y[i] = bought[i];
            const auto cuts = _separate(blocks, y);
            auto case_scores = instance.create_case_map(0.0);
            auto f_gain = f_gains.begin();
            for(std::size_t b = 0; b < blocks.size(); ++b) {
                const double flow = cuts[b].value;
                double quality = blocks[b].target_quality;
                for(const auto & [target_gain, i, big_M] :
                    blocks[b].target_quality_gains) {
                    candidate[*f_gain++] = y[i] * flow;
                    quality += y[i] * target_gain;
                }
                case_scores[blocks[b].case_id] += quality * flow;
                if(std::exchange(candidate[f[b]], flow) - flow <=
                   feasibility_tol * flow)
                    continue;
                add(cut_constraint(b, cuts[b]));
            }
            return case_scores;
        };

        auto [solution, report] = _benders_search(model, sw, instance, budget,
                                                  x, z, formula, cuts_at);
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
};

}  // namespace solvers
}  // namespace gecot
