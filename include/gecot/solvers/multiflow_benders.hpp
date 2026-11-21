#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <random>
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
    double in_out_ratio = 0.9;
    // the share of the way from the center to the new optimum at which the
    // rounds separate, before the center moves; none: in_out_ratio, the
    // new center
    std::optional<double> in_out_separation;
    // The rounds separate the point with every option raised to at least
    // in_out_lift, the "2ε trick" of Fischetti, Ljubić and Sinnl (2017), until
    // they add no cut, and the point itself from then on; 0: never.
    double in_out_lift = 1e-7;
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
    // Experimental: the n_t sources of each target not affine in ⌈n_t /
    // source_group_size⌉ groups, by grouping; 0: no group. A target of two
    // groups or more has a variable f_tg per group, within the sum of their
    // w̄_s Π̄(s, t), which receives the sum of their parts in each cut, and
    // the row f_t <= w^x_t + Σ_g f_tg, which bounds f_t in place of the cuts:
    // each group keeps the least of its own parts, where f_t kept the least
    // of their sums. A cut on f_t still comes with them when they leave the
    // point over it. With stored_arcs, the sources whose A_s has no improved
    // copy have an affine flow, written exactly in that row, in no group. At
    // convergence the relaxation is the same: over a finite set of cuts, the
    // groups combine the parts of different cuts, which f_t alone cannot.
    std::size_t source_group_size = 0;
    source_grouping grouping = source_grouping::voronoi;
    // of random, with the index of the target
    std::size_t grouping_seed = 0;
    // In a target with groups, the part of a group is added when the point's
    // f_tg exceeds it by the tolerance of the test of f_t, relative, and by
    // source_cut_abs_tol or --feasibility-tolerance, the larger, or when it
    // is under the group's envelope at the point, its bound and its parts so
    // far, by that relative tolerance and source_cut_abs_tol shared by the
    // groups of the target. The cut on f_t is added too when no part cuts the
    // point off, or when the parts leave f_t over the tolerances.
    static constexpr double source_cut_abs_tol = 1e-7;

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
                              cuts[b] = blocks[b].separate(
                                  x_hat, !stored_arcs,
                                  !blocks[b].groups.empty());
                          });
        return cuts;
    }
    // of a block at a candidate: the f_t of the candidate, its cut if it is
    // overvalued, and the flow of each of its groups at the scenario
    template <typename B>
    struct overvalued_block {
        double f_t = 0;
        std::optional<typename B::cut> cut;
        std::vector<double> group_flows;
    };
    // The search, steps 2 and 3: the cut at the scenario y of each block
    // whose f_t in candidate exceeds f_t(y), which it writes there, its own
    // variable from its thread. Not inline in solve(): there, the loop is
    // compiled again for each backend and each add of _benders_search. The
    // f_tg are left to the caller.
    template <typename B>
    std::vector<overvalued_block<B>> _overvalued_cuts(
        const std::vector<B> & blocks, const melon::mapping<option_t> auto & y,
        auto & candidate) const {
        std::vector<overvalued_block<B>> result(blocks.size());
        tbb::parallel_for(
            std::size_t{0}, blocks.size(), [&](const std::size_t b) {
                const B & block = blocks[b];
                auto & [f_t, cut, group_flows] = result[b];
                const bool grouped = !block.groups.empty();
                std::vector<double> flows;
                if(grouped) flows.resize(block.sources.size());
                const double flow =
                    block.flow_at(y, [&](const std::size_t k, const double f) {
                        if(grouped) flows[k] = f;
                    });
                group_flows.assign(block.groups.size(), 0.0);
                for(std::size_t g = 0; g < block.groups.size(); ++g)
                    for(const std::size_t k : block.groups[g].members)
                        group_flows[g] += flows[k];
                f_t = std::exchange(candidate[block.f_t], flow);
                if(f_t - flow <= feasibility_tol * flow) return;
                cut = block.separate(y, !stored_arcs, grouped);
            });
        return result;
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
    // point in_out_separation of the way to it from the center, which then
    // moves in_out_ratio of the way to it, and adds as rows
    // add_cut(block, cut, ...) for each block whose f_t exceeds its cut.
    template <typename M, instance_c I, typename B>
    root_rounds _root_rounds(M & model, const spdlog::stopwatch & sw,
                             const I & instance, const auto & x,
                             const std::vector<B> & blocks,
                             auto && add_cut) const {
        namespace status = mippp::status;
        root_rounds root;
        // a Ctrl-C before the rounds stops the whole run
        const bool stopped_before = stop_requested;
        for(auto && i : instance.options()) model.set_continuous(x(i));
        // the relaxation with all the cuts lies between root_floor and
        // root.value
        std::optional<double> root_floor;
        std::vector<double> center(instance.options().size()),
            separated(instance.options().size());
        double center_coef = in_out_ratio;
        double separation_coef = in_out_separation.value_or(in_out_ratio);
        double lift = in_out_lift;
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
                for(auto && i : instance.options())
                    separated[i] = center[i] = optimum(i);
            } else {
                for(auto && i : instance.options()) {
                    separated[i] = (1 - separation_coef) * center[i] +
                                   separation_coef * optimum(i);
                    center[i] = (1 - center_coef) * center[i] +
                                center_coef * optimum(i);
                }
            }
            if(lift > 0)
                for(auto && i : instance.options())
                    separated[i] = std::max(separated[i], lift);
            auto cuts = _separate(blocks, separated);
            // A lifted point can be over the budget: its value can exceed the
            // relaxation's, and --root-cut-gap would then end the rounds early
            if(lift == 0)
                if(const auto floor =
                       _floor_at(instance, blocks, separated, cuts))
                    root_floor = std::max(root_floor.value_or(*floor), *floor);
            // the smallest --root-cut-tolerance at which no cut is added
            double violation = 0;
            std::size_t num_added = 0;
            for(auto && [block, cut] : std::views::zip(blocks, cuts)) {
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
                   cut_at_optimum * (1 + root_cut_tol) + root_cut_abs_tol)
                    num_added += add_cut(
                        block, cut, optimum,
                        [&](const auto & v) { return relaxation[v]; },
                        root_cut_tol, f_at_optimum,
                        [&](auto && row) {
                            model.add_constraint(
                                std::forward<decltype(row)>(row));
                        });
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
            // Not on a lifted round: a lifted cut exceeds the flow at x* by
            // up to its coefficients times the lift, and the rounds that add
            // none would end with the bound above the relaxation's value
            if(center_coef == 1.0 && separation_coef == 1.0 && lift == 0) break;
            center_coef = separation_coef = 1.0;
            lift = 0;
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
    // search(model, sw, x, z, formula, blocks, add_cut, root) finds
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

        // the groups of each block, their f_tg, and the row
        // f_t <= w^x_t + Σ_g f_tg that they fill
        std::size_t num_group_flows = 0, num_affine = 0, largest_group = 0;
        if(source_group_size > 0) {
            tbb::parallel_for(
                std::size_t{0}, blocks.size(), [&](const std::size_t b) {
                    // seed_seq keeps 32 bits of each value
                    std::seed_seq seed{grouping_seed & 0xffffffff,
                                       grouping_seed >> 32, b};
                    std::mt19937_64 rng(seed);
                    blocks[b].make_groups(source_group_size, grouping,
                                          stored_arcs, rng);
                });
            for(target_block & block : blocks) {
                if(block.groups.empty()) continue;
                const auto [fixed_constant, fixed_terms] = block.fixed_part();
                const auto link_row = model.add_constraint(
                    block.f_t <= fixed_constant +
                                     xsum(fixed_terms, [&](const auto & term) {
                                         return term.second * x(term.first);
                                     }));
                for(auto & group : block.groups) {
                    group.f = model.add_column({std::make_pair(link_row, -1.0)},
                                               {.upper_bound = group.bound});
                    largest_group =
                        std::max(largest_group, group.members.size());
                }
                num_group_flows += block.groups.size();
                num_affine += block.affine_sources.size();
            }
            spdlog::debug(
                "Source groups: {} variables, the largest of {} sources, {} "
                "affine sources",
                num_group_flows, largest_group, num_affine);
        }
        // the parts of each group of each block added so far, its envelope
        std::vector<std::vector<std::vector<typename target_block::source_cut>>>
            group_parts(blocks.size());
        for(std::size_t b = 0; b < blocks.size(); ++b)
            group_parts[b].resize(blocks[b].groups.size());
        // multiflow_benders: the cut c of a block as rows, each through add:
        // the row of f_t, or with groups those of the parts of the groups
        // that the point, its f_tg read by value_of, exceeds by tol,
        // or that are under the group's envelope there by tol, and the row of
        // f_t when the point, f_value its f_t, exceeds the cut by tol and no
        // part cuts it off, or the master with these parts still allows f_t
        // over the cut there: a candidate of the search may break rows
        // already added, which must then cut it off again. Returns their
        // number. Lazy sums that read the cut: only valid while the cut lives.
        auto add_cut = [&](const target_block & block, cut & c, auto && point,
                           auto && value_of, const double tol,
                           const double f_value,
                           auto && add) -> std::size_t {
            auto f_t_row = [&] {
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
            if(block.groups.empty()) {
                add(f_t_row());
                return 1;
            }
            auto & parts =
                group_parts[static_cast<std::size_t>(&block - blocks.data())];
            const double abs_tol =
                source_cut_abs_tol /
                static_cast<double>(std::max<std::size_t>(1, parts.size()));
            std::size_t num_rows = 0;
            bool cut_off = false;  // by a part
            // what the master allows f_t at point over the cut: Σ of the
            // envelopes of the groups not cut over their parts
            double slack = 0;
            for(std::size_t g = 0; g < parts.size(); ++g) {
                auto part = block.group_part(g, c);
                const double at = part.at(point);
                const double over = value_of(block.groups[g].f) - at;
                double envelope = block.groups[g].bound;
                for(const auto & old : parts[g])
                    envelope = std::min(envelope, old.at(point));
                // the envelope shares the absolute tolerance among the
                // groups; a part cuts the point off only by all of it and
                // by the solver's tolerance, within which the solver could
                // keep the point
                const double part_tol = tol * std::abs(at) + abs_tol;
                const double cut_off_tol =
                    tol * std::abs(at) +
                    std::max(source_cut_abs_tol, feasibility_tol);
                if(over > cut_off_tol || at < envelope - part_tol) {
                    add(block.groups[g].f <=
                        part.constant +
                            xsum(part.coefficients, [&](const auto & term) {
                                return term.second * x(term.first);
                            }));
                    ++num_rows;
                    cut_off = cut_off || over > cut_off_tol;
                    parts[g].push_back(std::move(part));
                } else
                    slack += envelope - at;
            }
            double cut_at = c.constant;
            for(auto && i : instance.options())
                cut_at += c.coefficients[i] * point(i);
            const double cut_tol = cut_at * (1 + tol) + source_cut_abs_tol;
            if(f_value > cut_tol && (!cut_off || cut_at + slack > cut_tol)) {
                add(f_t_row());
                ++num_rows;
            }
            return num_rows;
        };
        // The search, step 1: the cuts of the MIP start
        if(const auto start = _add_mip_start(model, instance, x)) {
            auto start_cuts = _separate(blocks, *start);
            for(auto && [block, c] : std::views::zip(blocks, start_cuts))
                add_cut(
                    block, c,
                    [&](const option_t i) { return (*start)[i] ? 1.0 : 0.0; },
                    [](const auto &) {
                        return -std::numeric_limits<double>::infinity();
                    },
                    0.0, -std::numeric_limits<double>::infinity(),
                    [&](auto && row) {
                        model.add_constraint(std::forward<decltype(row)>(row));
                    });
        }

        spdlog::info("Solving the optimization model");
        // Cuts on the relaxation, before the search
        const root_rounds root =
            _root_rounds(model, sw, instance, x, blocks, add_cut);

        auto [solution, report] =
            search(model, sw, x, z, formula, blocks, add_cut, root);
        report.stats.emplace_back("sources", num_sources);
        if(source_group_size > 0) {
            report.stats.emplace_back("source_groups", num_group_flows);
            report.stats.emplace_back("affine_sources", num_affine);
        }
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
                const auto & add_cut, const root_rounds &) {
                // The search, steps 2 to 4: the flows of a candidate y and the
                // cut at y of each target whose f_t exceeds its flow. The flows
                // need no cut, and the cuts are made for these targets only.
                auto cuts_at = [&](auto & candidate, const auto & y,
                                   auto && add) {
                    auto overvalued = _overvalued_cuts(blocks, y, candidate);
                    auto case_scores = instance.create_case_map(0.0);
                    for(auto && [block, over] :
                        std::views::zip(blocks, overvalued)) {
                        auto & [f_value, block_cut, group_flows] = over;
                        // f_t(y), which _overvalued_cuts left in candidate
                        const double flow = candidate[block.f_t];
                        double quality = block.target_quality;
                        for(const auto & [target_gain, i, big_M, f_ti] :
                            block.target_quality_gains) {
                            candidate[f_ti] = y[i] * flow;
                            quality += y[i] * target_gain;
                        }
                        case_scores[block.case_id] += quality * flow;
                        if(block_cut)
                            add_cut(
                                block, *block_cut,
                                [&](const option_t i) {
                                    return y[i] ? 1.0 : 0.0;
                                },
                                [&](const auto & v) { return candidate[v]; },
                                feasibility_tol, f_value, add);
                        // the f_tg at y
                        for(std::size_t g = 0; g < group_flows.size(); ++g)
                            candidate[block.groups[g].f] = group_flows[g];
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
