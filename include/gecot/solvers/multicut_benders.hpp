#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/algorithm/network_voronoi.hpp"
#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/preprocessing/compute_contracted_graph.hpp"

#include "gecot/solvers/benders_base.hpp"

namespace gecot {
namespace solvers {

// The decomposition of docs/formulations/benders.md with a cut per group of
// targets. Each block of solve() builds what the section it names states.
struct multicut_benders : public benders_base {
    // groups per case; 0: a group per target
    std::size_t num_groups = 2;
    // voronoi: the cells of a network Voronoi diagram, so that a group
    // holds targets close to each other; consecutive: groups of consecutive
    // targets; random: of targets drawn at random, a control
    enum class grouping { voronoi, consecutive, random };
    grouping groups = grouping::voronoi;
    std::size_t random_seed = 0;

    // The targets of a case in the order of their groups, and the position
    // of the first target of each group, then their number. The kernels of
    // the Voronoi cells are targets of value far from each other, on the
    // graph without options: the target of most value, then each time the
    // one of most value not connected to the kernels so far.
    template <case_c C>
    auto _grouped_targets(const C & instance_case) const {
        using vertex = melon::vertex_t<case_graph_t<C>>;
        const auto & graph = instance_case.graph();
        auto is_target = melon::create_vertex_map<bool>(graph, false);
        std::vector<vertex> targets;
        for(const auto & t : melon::vertices(graph))
            if(_is_target(instance_case, t)) {
                is_target[t] = true;
                targets.push_back(t);
            }
        const std::size_t n = targets.size();
        const std::size_t G = num_groups ? std::min(num_groups, n) : n;
        std::vector<std::size_t> firsts;
        if(groups != grouping::voronoi || G <= 1 || G == n) {
            if(groups == grouping::random)
                std::ranges::shuffle(targets, std::mt19937_64{random_seed});
            for(std::size_t g = 0; g < G; ++g) firsts.push_back(g * n / G);
            firsts.push_back(n);
            return std::make_pair(std::move(targets), std::move(firsts));
        }
        auto length_map = melon::create_arc_map<double>(graph);
        for(const auto & a : melon::arcs(graph))
            length_map[a] = -std::log(instance_case.arc_probability_map()[a]);
        // the value of each target without options, its best quality counted
        auto value_map = melon::create_vertex_map<double>(graph, 0.0);
        for(const vertex t : targets) {
            double quality = instance_case.target_quality_map()[t];
            for(auto && [source_gain, target_gain, i] :
                instance_case.vertex_options_map()[t])
                quality += target_gain;
            value_map[t] =
                quality *
                pc_num_vertex_in_flow(graph, instance_case.source_quality_map(),
                                      instance_case.arc_probability_map(), t);
        }
        auto group_map = melon::create_vertex_map<std::size_t>(graph, 0);
        std::vector<vertex> kernels{*std::ranges::max_element(
            targets, {}, [&](const vertex t) { return value_map[t]; })};
        for(;;) {
            // the next kernel: the target of most value not connected to
            // the kernels, v_t (1 - P), P the probability of its most
            // reliable path from a kernel. Never a kernel: set_kernels
            // requires them distinct.
            auto unreached = is_target;
            vertex next = kernels.front();
            double next_value = -1;
            auto propose = [&](const vertex t, const double value) {
                if(value <= next_value || std::ranges::contains(kernels, t))
                    return;
                next = t;
                next_value = value;
            };
            for(const auto & [v, entry] :
                melon::network_voronoi(graph, length_map, kernels)) {
                if(!is_target[v]) continue;
                unreached[v] = false;
                group_map[v] = static_cast<std::size_t>(std::ranges::distance(
                    kernels.begin(), std::ranges::find(kernels, entry.second)));
                propose(v, value_map[v] * (1 - std::exp(-entry.first)));
            }
            for(const vertex t : targets)
                if(unreached[t]) {
                    group_map[t] = 0;
                    propose(t, value_map[t]);
                }
            if(kernels.size() == G) break;
            kernels.push_back(next);
        }
        std::ranges::stable_sort(targets, {},
                                 [&](const vertex t) { return group_map[t]; });
        for(std::size_t j = 0; j < n; ++j)
            if(j == 0 || group_map[targets[j]] != group_map[targets[j - 1]])
                firsts.push_back(j);
        firsts.push_back(n);
        return std::make_pair(std::move(targets), std::move(firsts));
    }

    // v_t(y) and the potentials of cut^v_{t,y}: those of the cut of a target
    // times the target quality q^y_t
    template <case_c C>
    static auto _value_and_potentials(const contracted_graph_data<C> & G_t,
                                      const auto & y) {
        const auto & instance_case = G_t.instance_case.get();
        double quality = instance_case.target_quality_map()[G_t.original_t];
        for(auto && [source_gain, target_gain, i] :
            instance_case.vertex_options_map()[G_t.original_t])
            if(y[i]) quality += target_gain;
        auto [flow, rho] = G_t.flow_and_potentials(y);
        for(auto && u : melon::vertices(G_t.graph)) rho[u] *= quality;
        return std::make_pair(quality * flow, std::move(rho));
    }

    template <typename M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        using namespace mippp;
        using namespace mippp::operators;
        using target_graph = contracted_graph_data<instance_case_t<I>>;
        _check_criterion(instance);
        spdlog::stopwatch sw;
        M model;
        _set_tolerances(model, mip_gap, feasibility_tol);

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

        // theta_g and the targets [first, last) of its case that g holds
        struct group {
            model_variable_t<M> theta;
            std::size_t first, last;
        };
        // the graphs G_t of the targets of each case, in the order of their
        // groups
        auto targets =
            instance.template create_case_map<std::vector<target_graph>>();
        auto cases_groups =
            instance.template create_case_map<std::vector<group>>();
        // with the number of targets of each group
        std::vector<preprocessing_row<std::string>> preprocessing;
        for(auto && instance_case : instance.cases()) {
            const case_id_t k = instance_case.id();
            const auto [strong_arcs_map, useless_arcs_map] =
                _compute_strong_and_useless_arcs(instance, instance_case,
                                                 budget);
            spdlog::debug("Computing contractions and big-Ms of the '{}' graph",
                          instance_case.name());
            const auto [target_vertices, firsts] =
                _grouped_targets(instance_case);
            {
                progress_bar<spdlog::level::info, 64> pb(
                    target_vertices.size());
                for(const auto & t : target_vertices) {
                    targets[k].push_back(compute_contracted_graph_data(
                        instance, instance_case, budget, t, strong_arcs_map[t],
                        useless_arcs_map[t], big_Ms, feasibility_tol, pruning,
                        _options_model<M>(instance, budget)));
                    pb.tick();
                }
            }

            // multicut_benders, the master: theta_g for each group of the
            // case, and its score z_k at most their sum
            for(std::size_t g = 0; g + 1 < firsts.size(); ++g)
                cases_groups[k].push_back(
                    {model.add_variable(), firsts[g], firsts[g + 1]});
            model.add_constraint(z(k) <= xsum(std::views::transform(
                                             cases_groups[k], &group::theta)));

            preprocessing.emplace_back(
                instance_case.name(),
                _average_size(instance_case, strong_arcs_map),
                _average_size(instance_case, useless_arcs_map),
                std::format("{:n}", std::views::transform(
                                        cases_groups[k], [](const group & g) {
                                            return g.last - g.first;
                                        })));
        }
        _log_preprocessing(preprocessing, log_column<22>{"targets per group"});

        // multicut_benders: cut^v_{t,y}(x), the cut of the target with the
        // potentials rho of y that count the target quality, and the constant
        // of each target quality gain that y does not buy
        auto cut = [&](const target_graph & G_t, const auto & y,
                       const auto & rho) {
            return ordered_sum(
                G_t.cut(x, y, rho),
                xsum(G_t.instance_case.get()
                         .vertex_options_map()[G_t.original_t],
                     [&](auto && gain) {
                         auto && [source_gain, target_gain, i] = gain;
                         return G_t.target_big_M(i) *
                                (y[i] ? 0.0 : target_gain) * x(i);
                     }));
        };
        // visit(k, g, v_g, cut) for each group g of each case k, with
        // v_g the sum of the v_t(y) of its targets and its cut at y. The cut
        // is a lazy sum that reads values: it is only valid inside visit.
        auto for_each_group = [&](const auto & y, auto && visit) {
            for(auto && instance_case : instance.cases()) {
                const case_id_t k = instance_case.id();
                const auto values =
                    std::ranges::to<std::vector>(std::views::transform(
                        targets[k], [&](const target_graph & G_t) {
                            return _value_and_potentials(G_t, y);
                        }));
                for(const group & g : cases_groups[k]) {
                    double value = 0;
                    for(std::size_t n = g.first; n < g.last; ++n)
                        value += values[n].first;
                    visit(k, g, value,
                          g.theta <= xsum(std::views::iota(g.first, g.last),
                                          [&](const std::size_t n) {
                                              return cut(targets[k][n], y,
                                                         values[n].second);
                                          }));
                }
            }
        };
        auto seed_cuts = [&](const auto & y) {
            for_each_group(
                y, [&](case_id_t, const group &, double, auto && group_cut) {
                    model.add_constraint(
                        std::forward<decltype(group_cut)>(group_cut));
                });
        };
        // The search, step 1: the cuts of the scenario that buys nothing,
        // then those of the MIP start
        seed_cuts(instance.create_option_map(false));
        if(const auto start = _add_mip_start(model, instance, x))
            seed_cuts(*start);

        // The search, steps 2 to 4: the values of a candidate y and the cut
        // at y of each group whose theta_g exceeds its value
        auto cuts = [&](auto & candidate, const auto & y, auto && add) {
            auto case_scores = instance.create_case_map(0.0);
            for_each_group(y, [&](const case_id_t k, const group & g,
                                  const double value, auto && group_cut) {
                case_scores[k] += value;
                if(std::exchange(candidate[g.theta], value) - value <=
                   feasibility_tol * value)
                    return;
                add(std::forward<decltype(group_cut)>(group_cut));
            });
            return case_scores;
        };

        spdlog::info("Solving the optimization model");
        auto [solution, report] =
            _benders_search(model, sw, instance, budget, x, z, formula, cuts);
        if(!solution) return {_fallback_solution(instance, budget), report};
        return {std::move(*solution), report};
    }
};

}  // namespace solvers
}  // namespace gecot
