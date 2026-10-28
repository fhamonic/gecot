#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <flat_map>
#include <iterator>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include "mippp/utility/column_manager.hpp"

#include "melon/algorithm/biobjective_dijkstra.hpp"
#include "melon/algorithm/dijkstra.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"
#include "melon/utility/static_digraph_builder.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"

#include "gecot/solvers/mip_solver_base.hpp"

namespace gecot {
namespace solvers {

// Path formulation relaxation: the contribution of each (source, target) pair
// is a column, a set of purchased options with the value
// q_s(S) * q_t(S) * P_st(S), the qualities with their gains and the most
// reliable path probability under S. By default the columns of a pair are
// computed once: option costs and duals are nonnegative, so a higher
// probability with a subset of the options is better whatever the duals. Each
// round, the pricing subproblem of a pair keeps the column of best reduced
// cost, among its columns or by a search (search_pricing).
struct path_formulation_rr : public mip_solver_base {
    using variable_t = mippp::model_variable<int, double>;
    using constraint_t = mippp::model_constraint<int>;
    using vertex = melon::vertex_t<melon::static_digraph>;

    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict
    // false: each pair's columns are enumerated once; true: each round, a
    // biobjective (probability, duals) search prices each pair, without the
    // budget, for instances whose options each improve a single vertex
    bool search_pricing = false;
    std::size_t max_activations = 0;  // per round, 0: every improving column
    // stops once the Lagrangian bound is within this gap of the master's
    // value, relative to it; 0: at convergence only
    double relative_gap = 0;

    struct column {
        double value;
        std::vector<option_t> options;  // sorted
    };

    // the non-dominated (probability, options) labels of the paths from s,
    // within the budget, at every vertex
    template <instance_c I, case_c C>
    static auto _path_labels(const I & instance, const C & instance_case,
                             const auto s, const double budget) {
        struct label {
            double prob;
            std::vector<option_t> options;  // sorted
            double cost;
            bool alive = true;
        };
        const auto & graph = instance_case.graph();
        const auto & probability_map = instance_case.arc_probability_map();
        const auto & arc_options_map = instance_case.arc_options_map();
        auto labels_map = melon::create_vertex_map<std::vector<label>>(graph);
        std::priority_queue<
            std::tuple<double, std::remove_cv_t<decltype(s)>, std::size_t>>
            heap;
        auto relax = [&](const auto v, label && l) {
            auto & labels = labels_map[v];
            for(const auto & e : labels)
                if(e.alive && e.prob >= l.prob &&
                   std::ranges::includes(l.options, e.options))
                    return;
            for(auto & e : labels)
                if(e.alive && l.prob >= e.prob &&
                   std::ranges::includes(e.options, l.options))
                    e.alive = false;
            heap.emplace(l.prob, v, labels.size());
            labels.push_back(std::move(l));
        };
        relax(s, {1.0, {}, 0.0});
        while(!heap.empty()) {
            const auto [prob, u, i] = heap.top();
            heap.pop();
            if(!labels_map[u][i].alive) continue;
            const label l = labels_map[u][i];
            for(const auto & a : melon::out_arcs(graph, u)) {
                const auto v = melon::arc_target(graph, a);
                if(probability_map[a] > 0)
                    relax(v, {l.prob * probability_map[a], l.options, l.cost});
                for(const auto & [improved_prob, o] : arc_options_map[a]) {
                    if(std::ranges::binary_search(l.options, o)) {
                        relax(v, {l.prob * improved_prob, l.options, l.cost});
                        continue;
                    }
                    const double cost = l.cost + instance.option_cost(o);
                    if(cost > budget) continue;
                    auto options = l.options;
                    options.insert(std::ranges::upper_bound(options, o), o);
                    relax(v,
                          {l.prob * improved_prob, std::move(options), cost});
                }
            }
        }
        return labels_map;
    }

    // the non-dominated columns of the pair (s, t): its path labels with each
    // set of the quality options of s and t, within the budget
    template <instance_c I, case_c C>
    static std::vector<column> _pair_columns(const I & instance,
                                             const C & instance_case,
                                             const auto s, const auto t,
                                             const auto & labels,
                                             const double budget) {
        const auto & vertex_options_map = instance_case.vertex_options_map();
        std::vector<option_t> endpoint_options;
        for(const auto & [sqg, tqg, o] : vertex_options_map[s])
            if(sqg > 0) endpoint_options.push_back(o);
        for(const auto & [sqg, tqg, o] : vertex_options_map[t])
            if(tqg > 0) endpoint_options.push_back(o);
        std::ranges::sort(endpoint_options);
        endpoint_options.erase(std::ranges::unique(endpoint_options).begin(),
                               endpoint_options.end());
        if(endpoint_options.size() > 20)
            throw std::invalid_argument(
                "path_rr: too many quality options at a vertex");

        std::vector<column> columns;
        for(const auto & l : labels) {
            if(!l.alive) continue;
            for(std::size_t mask = 0;
                mask < (std::size_t{1} << endpoint_options.size()); ++mask) {
                auto options = l.options;
                for(std::size_t i = 0; i < endpoint_options.size(); ++i)
                    if(mask >> i & 1) options.push_back(endpoint_options[i]);
                std::ranges::sort(options);
                options.erase(std::ranges::unique(options).begin(),
                              options.end());
                double cost = 0, qs = instance_case.source_quality_map()[s],
                       qt = instance_case.target_quality_map()[t];
                for(const option_t o : options) cost += instance.option_cost(o);
                if(cost > budget) continue;
                for(const auto & [sqg, tqg, o] : vertex_options_map[s])
                    if(std::ranges::binary_search(options, o)) qs += sqg;
                for(const auto & [sqg, tqg, o] : vertex_options_map[t])
                    if(std::ranges::binary_search(options, o)) qt += tqg;
                column c{qs * qt * l.prob, std::move(options)};
                if(c.value <= 0 ||
                   std::ranges::any_of(columns, [&c](const column & e) {
                       return e.value >= c.value &&
                              std::ranges::includes(c.options, e.options);
                   }))
                    continue;
                std::erase_if(columns, [&c](const column & e) {
                    return c.value >= e.value &&
                           std::ranges::includes(e.options, c.options);
                });
                columns.push_back(std::move(c));
            }
        }
        return columns;
    }

    // The case graph plus an improved copy of each vertex that is the center
    // of an option, the copy entered when the option is bought: a path pays
    // the option's dual once, on entering the copy, and leaves it through the
    // improved arcs. Exact when each option improves only the arcs incident to
    // one vertex and the qualities of that vertex, which is checked.
    struct split_graph {
        std::size_t num_vertices;  // of the case graph
        melon::static_digraph graph;
        std::vector<double> probability;
        std::vector<int> entry_option;   // the option paid on entering, or -1
        std::vector<int> center_option;  // per vertex of the case graph, or -1
        std::vector<vertex> improved_copy;  // per center vertex
        std::vector<int> copy_option;       // per improved copy
    };
    template <instance_c I, case_c C>
    static split_graph _split_graph(const I & instance,
                                    const C & instance_case) {
        const auto & graph = instance_case.graph();
        const std::size_t n = graph.num_vertices();
        std::vector<std::optional<std::vector<std::size_t>>> candidates(
            instance.options().size());
        auto restrict_to = [&](const option_t o, std::vector<std::size_t> c) {
            std::ranges::sort(c);
            if(!candidates[o]) {
                candidates[o] = std::move(c);
                return;
            }
            std::vector<std::size_t> both;
            std::ranges::set_intersection(*candidates[o], c,
                                          std::back_inserter(both));
            candidates[o] = std::move(both);
        };
        for(const auto & a : melon::arcs(graph)) {
            if(instance_case.arc_options_map()[a].size() > 1)
                throw std::invalid_argument(
                    "path_rr --pricing search: an arc is improved by several "
                    "options");
            for(const auto & [improved_prob, o] :
                instance_case.arc_options_map()[a])
                restrict_to(o, {melon::arc_source(graph, a),
                                melon::arc_target(graph, a)});
        }
        for(const auto & v : melon::vertices(graph))
            for(const auto & [sqg, tqg, o] :
                instance_case.vertex_options_map()[v])
                restrict_to(o, {v});
        split_graph sg{
            n, {}, {}, {}, std::vector<int>(n, -1), std::vector<vertex>(n, 0),
            {}};
        for(const option_t o : instance.options()) {
            if(!candidates[o]) continue;
            const auto it = std::ranges::find_if(
                *candidates[o],
                [&](const std::size_t v) { return sg.center_option[v] < 0; });
            if(it == candidates[o]->end())
                throw std::invalid_argument(
                    "path_rr --pricing search: option '" +
                    instance.option_name(o) +
                    "' does not improve a single vertex and its arcs only");
            sg.center_option[*it] = static_cast<int>(o);
            sg.improved_copy[*it] =
                static_cast<vertex>(n + sg.copy_option.size());
            sg.copy_option.push_back(static_cast<int>(o));
        }
        melon::static_digraph_builder<melon::static_digraph, double, int>
            builder(n + sg.copy_option.size());
        auto copies = [&](const vertex v) {
            std::vector<std::pair<vertex, int>> c{{v, -1}};
            if(sg.center_option[v] >= 0)
                c.emplace_back(sg.improved_copy[v], sg.center_option[v]);
            return c;
        };
        for(const auto & a : melon::arcs(graph))
            for(const auto & [cu, ou] : copies(melon::arc_source(graph, a)))
                for(const auto & [cv, ov] :
                    copies(melon::arc_target(graph, a))) {
                    double p = instance_case.arc_probability_map()[a];
                    for(const auto & [improved_prob, o] :
                        instance_case.arc_options_map()[a])
                        if(static_cast<int>(o) == ou ||
                           static_cast<int>(o) == ov)
                            p = improved_prob;
                    if(p > 0) builder.add_arc({cu, cv}, p, ov);
                }
        std::tie(sg.graph, sg.probability, sg.entry_option) = builder.build();
        return sg;
    }

    // the quality of a vertex as a source or a target, on one of its copies
    template <case_c C>
    static double _quality(const C & instance_case, const split_graph & sg,
                           const vertex v, const bool improved,
                           const bool as_source) {
        double q = as_source ? instance_case.source_quality_map()[v]
                             : instance_case.target_quality_map()[v];
        if(improved)
            for(const auto & [sqg, tqg, o] :
                instance_case.vertex_options_map()[v])
                if(static_cast<int>(o) == sg.center_option[v])
                    q += as_source ? sqg : tqg;
        return q;
    }
    // the copies a path from s starts at, with their starting values
    template <case_c C, typename F>
    static auto _starts(const C & instance_case, const split_graph & sg,
                        const vertex s, F && allowed) {
        std::vector<std::pair<vertex, double>> starts;
        if(const double q = _quality(instance_case, sg, s, false, true); q > 0)
            starts.emplace_back(s, q);
        if(sg.center_option[s] >= 0 && allowed(sg.center_option[s]))
            if(const double q = _quality(instance_case, sg, s, true, true);
               q > 0)
                starts.emplace_back(sg.improved_copy[s], q);
        return starts;
    }
    // the copies a path to t ends at, with their target qualities
    template <case_c C, typename F>
    static auto _ends(const C & instance_case, const split_graph & sg,
                      const vertex t, F && allowed) {
        std::vector<std::pair<vertex, double>> ends{
            {t, _quality(instance_case, sg, t, false, false)}};
        if(sg.center_option[t] >= 0 && allowed(sg.center_option[t]))
            ends.emplace_back(sg.improved_copy[t],
                              _quality(instance_case, sg, t, true, false));
        return ends;
    }

    struct reliability_traits {
        using semiring = melon::most_reliable_path_semiring<double>;
        using heap = melon::updatable_d_ary_heap<
            4, std::pair<melon::vertex_t<melon::static_digraph>, double>,
            typename semiring::less_t,
            melon::vertex_map_t<melon::static_digraph, std::size_t>,
            melon::maps::element<1>, melon::maps::element<0>>;
        static constexpr bool store_paths = true;
        static constexpr bool store_distances = true;
    };
    // for each target, the best column from s that buys only options that
    // allowed(o) accepts
    template <case_c C, typename F>
    static std::vector<std::optional<column>> _best_columns(
        const C & instance_case, const split_graph & sg, const vertex s,
        const auto & targets, F && allowed) {
        auto algo = melon::dijkstra(
            reliability_traits{}, sg.graph,
            melon::maps::function([&sg, &allowed](const auto a) {
                const int o = sg.entry_option[a];
                return o < 0 || allowed(o) ? sg.probability[a] : 0.0;
            }));
        const auto starts = _starts(instance_case, sg, s, allowed);
        for(const auto & [v, q] : starts) algo.add_source(v, q);
        algo.run();
        std::vector<std::optional<column>> best(targets.size());
        for(std::size_t i = 0; i < targets.size(); ++i) {
            for(const auto & [v, q] :
                _ends(instance_case, sg, targets[i], allowed)) {
                const vertex u0 = v;
                if(!algo.reached(u0)) continue;
                const double value = algo.dist(u0) * q;
                if(value <= 0 || (best[i] && best[i]->value >= value)) continue;
                column c{value, {}};
                for(auto u = u0;;) {
                    if(u >= sg.num_vertices)
                        c.options.push_back(static_cast<option_t>(
                            sg.copy_option[u - sg.num_vertices]));
                    if(std::ranges::any_of(starts, [&](const auto & e) {
                           return e.first == u && e.second == algo.dist(u);
                       }))
                        break;
                    u = algo.pred_vertex(u);
                }
                std::ranges::sort(c.options);
                best[i] = std::move(c);
            }
        }
        return best;
    }

    struct pricing_traits {
        using blue_semiring = melon::most_reliable_path_semiring<double>;
        using red_semiring = melon::shortest_path_semiring<double>;
        using label = std::pair<double, double>;
        using heap = melon::d_ary_heap<
            2, std::pair<melon::vertex_t<melon::static_digraph>, label>,
            typename blue_semiring::less_t, melon::maps::element<1, 0>>;
    };
    // the dual of the option paid on entering a copy, for the pair priced
    struct entry_duals {
        const split_graph * sg;
        const std::vector<double> * duals;
        double operator()(const melon::arc_t<melon::static_digraph> a) const {
            const int o = sg->entry_option[a];
            return o < 0 ? 0.0 : (*duals)[static_cast<std::size_t>(o)];
        }
    };
    using pricing_algorithm = decltype(melon::biobjective_dijkstra(
        pricing_traits{}, std::declval<const melon::static_digraph &>(),
        std::declval<const std::vector<double> &>(),
        melon::maps::function(std::declval<entry_duals>())));

    // the rows of a pair are created with its first columns in the master:
    // until then they hold no column, are redundant and have a zero dual
    struct pair_data {
        case_id_t case_id;
        vertex source, target;
        std::vector<column> columns;  // enumerated, or empty when searched
        double max_value;             // bounds the searched columns
        std::optional<column> start_column;
        std::vector<double> unproductive_duals = {};
        std::optional<constraint_t> uniqueness_constraint = {};
        std::flat_map<option_t, constraint_t> purchase_constraints = {};
    };

    struct column_seed {
        std::size_t pair;
        std::vector<option_t> options;  // sorted
        double value;                   // not part of the key
        bool operator==(const column_seed & other) const {
            return pair == other.pair && options == other.options;
        }
    };
    struct column_seed_hash {
        std::size_t operator()(const column_seed & seed) const noexcept {
            std::size_t h = seed.pair;
            for(const option_t o : seed.options)
                h ^= o + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            return h;
        }
    };

    template <typename LP, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        namespace colgen = mippp::colgen;
        spdlog::stopwatch sw;
        auto solution = instance.create_option_map(false);

        using namespace mippp;
        using namespace mippp::operators;
        LP model;
        if constexpr(mippp::has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();

        const auto X_vars = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                 return instance.option_cost(o) * X_vars(o);
                             }) <= budget);

        std::optional<instance_solution_t<I>> start;
        if(mip_start &&
           compute_solution_cost(instance, _mip_start_solution(instance)) >
               budget + feasibility_tol)
            spdlog::warn("MIP start exceeds the budget: ignored");
        else if(mip_start)
            start = _mip_start_solution(instance);
        auto in_start = [&start](const auto o) {
            return start && (*start)[static_cast<option_t>(o)];
        };

        // the pairs that options can change, the others summed per case
        const auto & cases = instance.cases();
        std::vector<pair_data> pairs;
        std::vector<split_graph> split_graphs;
        std::vector<variable_t> contribution_vars;
        std::vector<constraint_t> contribution_constraints;
        for(auto && instance_case : instance.cases()) {
            const auto & graph = instance_case.graph();
            auto endpoint = [&](const auto v, const auto & quality_map,
                                const bool as_source) {
                return quality_map[v] > 0 ||
                       std::ranges::any_of(
                           instance_case.vertex_options_map()[v],
                           [as_source](const auto & e) {
                               const auto & [sqg, tqg, o] = e;
                               return (as_source ? sqg : tqg) > 0;
                           });
            };
            std::vector<vertex> sources, targets;
            for(const auto & v : melon::vertices(graph)) {
                if(endpoint(v, instance_case.source_quality_map(), true))
                    sources.push_back(v);
                if(endpoint(v, instance_case.target_quality_map(), false))
                    targets.push_back(v);
            }
            if(search_pricing)
                split_graphs.push_back(_split_graph(instance, instance_case));
            // per source, the constant pairs' values and the other pairs, kept
            // in order so the model is the same from run to run
            std::vector<std::pair<double, std::vector<pair_data>>> source_pairs(
                sources.size());
            tbb::parallel_for(std::size_t{0}, sources.size(), [&](auto i) {
                const vertex s = sources[i];
                auto add = [&](const vertex t, std::vector<column> columns,
                               const double base_value, const double max_value,
                               std::optional<column> start_column) {
                    if(max_value <= base_value) {
                        source_pairs[i].first += base_value;
                        return;
                    }
                    if(!start_column && base_value > 0)
                        start_column = column{base_value, {}};
                    source_pairs[i].second.push_back(
                        {instance_case.id(), s, t, std::move(columns),
                         max_value, std::move(start_column)});
                };
                if(!search_pricing) {
                    const auto labels_map =
                        _path_labels(instance, instance_case, s, budget);
                    for(const vertex t : targets) {
                        auto columns = _pair_columns(instance, instance_case, s,
                                                     t, labels_map[t], budget);
                        double base_value = 0, max_value = 0;
                        std::optional<column> start_column;
                        for(const column & c : columns) {
                            if(c.options.empty()) base_value = c.value;
                            max_value = std::max(max_value, c.value);
                            if(std::ranges::all_of(c.options, in_start) &&
                               (!start_column || c.value > start_column->value))
                                start_column = c;
                        }
                        add(t, std::move(columns), base_value, max_value,
                            std::move(start_column));
                    }
                    return;
                }
                const split_graph & sg = split_graphs.back();
                const auto base = _best_columns(instance_case, sg, s, targets,
                                                [](auto) { return false; });
                const auto max = _best_columns(instance_case, sg, s, targets,
                                               [](auto) { return true; });
                const auto from_start =
                    start
                        ? _best_columns(instance_case, sg, s, targets, in_start)
                        : std::vector<std::optional<column>>(targets.size());
                for(std::size_t j = 0; j < targets.size(); ++j)
                    add(targets[j], {}, base[j] ? base[j]->value : 0.0,
                        max[j] ? max[j]->value : 0.0, from_start[j]);
            });
            double constant = 0;
            for(auto & [value, case_pairs] : source_pairs) {
                constant += value;
                std::ranges::move(case_pairs, std::back_inserter(pairs));
            }
            contribution_vars.push_back(model.add_variable());
            contribution_constraints.push_back(
                model.add_constraint(contribution_vars.back() <= constant));
        }

        model.set_objective(std::visit(
            formula_variable_visitor{
                model,
                [&](const case_id_t & i) { return contribution_vars[i]; }},
            instance.criterion()));

        auto log_lambda = [&, option_name_max_length = std::ranges::max(
                                  std::ranges::views::transform(
                                      instance.options(), [&](auto && o) {
                                          return instance.option_name(o).size();
                                      }))]() {
            const auto model_solution = model.get_solution();
            spdlog::info("path formulation relaxation has value: {}",
                         model.get_solution_value());
            for(auto && option : instance.options()) {
                const auto & option_name = instance.option_name(option);
                spdlog::info("    {:<{}} {}", option_name,
                             option_name_max_length,
                             model_solution[X_vars(option)]);
            }
        };

        colgen::column_manager<
            LP, column_seed, colgen::property_list<colgen::reduced_cost>,
            colgen::property_list<colgen::reduced_cost, colgen::age>,
            column_seed_hash>
            columns;
        auto add_column = [&](LP &, const column_seed & seed) {
            pair_data & p = pairs[seed.pair];
            if(!p.uniqueness_constraint)
                p.uniqueness_constraint = model.add_constraint(
                    empty_linear_expression<variable_t, double> <= 1);
            for(const option_t o : seed.options)
                if(!p.purchase_constraints.contains(o))
                    p.purchase_constraints.emplace(
                        o, model.add_constraint(
                               empty_linear_expression<variable_t, double> <=
                               X_vars(o)));
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_constraints[p.case_id], -seed.value)),
                std::views::single(
                    std::make_pair(*p.uniqueness_constraint, 1.0)),
                std::views::transform(seed.options, [&p](const option_t o) {
                    return std::make_pair(p.purchase_constraints.at(o), 1.0);
                })));
        };
        // the master starts with the columns that buy nothing or, with
        // --mip-start, the best columns within the start's options
        for(std::size_t i = 0; i < pairs.size(); ++i) {
            if(!pairs[i].start_column) continue;
            column_seed seed{i, pairs[i].start_column->options,
                             pairs[i].start_column->value};
            const auto var = add_column(model, seed);
            columns.emplace_master_column(std::move(seed), var);
        }

        auto uniqueness_dual = [&](const pair_data & p,
                                   const auto & dual_solution) {
            return p.uniqueness_constraint
                       ? dual_solution[*p.uniqueness_constraint]
                       : 0.0;
        };
        auto purchase_dual = [&](const pair_data & p, const option_t o,
                                 const auto & dual_solution) {
            const auto it = p.purchase_constraints.find(o);
            return it == p.purchase_constraints.end()
                       ? 0.0
                       : dual_solution[it->second];
        };
        auto reduced_cost = [&](const column_seed & seed,
                                const auto & dual_solution) {
            const pair_data & p = pairs[seed.pair];
            double rc = dual_solution[contribution_constraints[p.case_id]] *
                            seed.value -
                        uniqueness_dual(p, dual_solution);
            for(const option_t o : seed.options)
                rc -= purchase_dual(p, o, dual_solution);
            return rc;
        };
        // in the master for more than evict_age rounds and unattractive, so
        // nonbasic
        const colgen::all<colgen::conjunction<
            colgen::above<colgen::age>, colgen::negative<colgen::reduced_cost>>>
            unattractive{colgen::above<colgen::age>{evict_age},
                         colgen::negative<colgen::reduced_cost>{1e-6}};
        if constexpr(!mippp::has_remove_variable<LP>)
            if(evict_age)
                spdlog::warn(
                    "This solver cannot remove columns: --evict-age ignored");

        // one search algorithm per thread and case, reused across pairs
        struct pricing_state {
            std::vector<double> duals;
            std::vector<std::optional<pricing_algorithm>> algorithms;
        };
        tbb::enumerable_thread_specific<pricing_state> pricing_states;
        std::atomic<std::size_t> num_searches = 0;

        solve_report report{"converged"};
        std::size_t iterations = 1;  // LP solves
        std::size_t num_evicted = 0;
        auto time_up = [&] {
            return time_limit && sw.elapsed().count() > time_limit.value();
        };
        // per pair, a bound on the best reduced cost and 0: with the master's
        // value, the Lagrangian bound of a complete pricing round
        std::vector<double> pricing_bounds(pairs.size());
        // not the report's best_bound: the solution returned, the master's
        // options cast to bool, can exceed the budget
        std::optional<double> best_bound;
        for(;; ++iterations) {
            model.solve();
            const auto dual_solution = model.get_dual_solution();
            const double master_value = model.get_solution_value();

            log_lambda();

            // the pricing test, at the master's dual tolerance: a reduced cost
            // above margin, a value net of the option duals above threshold
            auto margin = [&](const pair_data & p) {
                return 1e-7 * uniqueness_dual(p, dual_solution) + 1e-6;
            };
            auto threshold = [&](const pair_data & p) {
                return uniqueness_dual(p, dual_solution) + margin(p);
            };
            std::atomic<bool> cut_short = false;
            tbb::concurrent_vector<column_seed> priced_columns;
            tbb::parallel_for(std::size_t{0}, pairs.size(), [&](auto i) {
                pair_data & p = pairs[i];
                const double pi_c =
                    dual_solution[contribution_constraints[p.case_id]];
                if(!search_pricing) {
                    std::optional<std::pair<double, std::size_t>> best;
                    for(std::size_t j = 0; j < p.columns.size(); ++j) {
                        double rc = pi_c * p.columns[j].value -
                                    uniqueness_dual(p, dual_solution);
                        for(const option_t o : p.columns[j].options)
                            rc -= purchase_dual(p, o, dual_solution);
                        if(!best || rc > best->first) best.emplace(rc, j);
                    }
                    pricing_bounds[i] = best ? std::max(best->first, 0.0) : 0.0;
                    if(best && best->first > margin(p))
                        priced_columns.push_back(
                            {i, p.columns[best->second].options,
                             p.columns[best->second].value});
                    return;
                }
                // what the skips and the stopped searches leave unpriced has
                // a reduced cost of at most margin(p)
                pricing_bounds[i] = margin(p);
                if(pi_c * p.max_value <= threshold(p)) return;
                std::vector<double> key{pi_c,
                                        uniqueness_dual(p, dual_solution)};
                for(const auto & [o, row] : p.purchase_constraints)
                    key.push_back(dual_solution[row]);
                if(key == p.unproductive_duals) return;
                if(time_up()) {
                    cut_short = true;
                    return;
                }
                ++num_searches;
                pricing_state & state = pricing_states.local();
                const split_graph & sg = split_graphs[p.case_id];
                if(state.duals.empty())
                    state.duals.assign(instance.options().size(), 0.0);
                if(state.algorithms.size() <= p.case_id)
                    state.algorithms.resize(p.case_id + 1);
                auto & algo = state.algorithms[p.case_id];
                if(!algo)
                    algo.emplace(
                        pricing_traits{}, sg.graph, sg.probability,
                        melon::maps::function(entry_duals{&sg, &state.duals}));
                for(const auto & [o, row] : p.purchase_constraints)
                    state.duals[o] = dual_solution[row];
                const auto & instance_case = *std::ranges::next(
                    cases.begin(), static_cast<std::ptrdiff_t>(p.case_id));
                auto any = [](auto) { return true; };
                const auto starts = _starts(instance_case, sg, p.source, any);
                const auto ends = _ends(instance_case, sg, p.target, any);
                auto start_dual = [&](const std::size_t v) {
                    return v < sg.num_vertices
                               ? 0.0
                               : state.duals[static_cast<std::size_t>(
                                     sg.copy_option[v - sg.num_vertices])];
                };
                algo->reset();
                for(const auto & [v, q] : starts)
                    algo->add_source(static_cast<vertex>(v), q, start_dual(v));
                double q_max = 0;
                for(const auto & [v, q] : ends) q_max = std::max(q_max, q);
                // the labels come by decreasing probability and their duals
                // only grow: none after this bound can improve the master
                std::optional<
                    std::tuple<double, vertex, typename pricing_traits::label>>
                    best;
                for(; !algo->finished(); algo->advance()) {
                    const auto [v, label] = algo->current();
                    if(pi_c * q_max * label.first <= threshold(p)) break;
                    for(const auto & [e, q] : ends) {
                        if(e != v) continue;
                        const double rc = pi_c * label.first * q -
                                          label.second -
                                          uniqueness_dual(p, dual_solution);
                        if(!best || rc > std::get<0>(*best))
                            best.emplace(rc, v, label);
                    }
                }
                if(!best || std::get<0>(*best) <= margin(p)) {
                    p.unproductive_duals = std::move(key);
                } else {
                    // back through the fronts: each label extends a label
                    // of an in-neighbour exactly
                    auto [rc, v, label] = *best;
                    pricing_bounds[i] = rc;
                    column_seed seed{i, {}, 0.0};
                    for(const auto & [e, q] : ends)
                        if(e == v) seed.value = label.first * q;
                    std::vector<bool> visited(sg.graph.num_vertices(), false);
                    while(!std::ranges::any_of(starts, [&](const auto & st) {
                        return st.first == v && st.second == label.first &&
                               start_dual(v) == label.second;
                    })) {
                        if(v >= sg.num_vertices)
                            seed.options.push_back(static_cast<option_t>(
                                sg.copy_option[v - sg.num_vertices]));
                        visited[v] = true;
                        bool found = false;
                        for(const auto & a : melon::in_arcs(sg.graph, v)) {
                            const vertex u = melon::arc_source(sg.graph, a);
                            if(visited[u]) continue;
                            for(const auto & l : algo->pareto_front(u)) {
                                if(pricing_traits::blue_semiring::plus(
                                       l.first, sg.probability[a]) !=
                                       label.first ||
                                   pricing_traits::red_semiring::plus(
                                       l.second, entry_duals{&sg, &state.duals}(
                                                     a)) != label.second)
                                    continue;
                                v = u;
                                label = l;
                                found = true;
                                break;
                            }
                            if(found) break;
                        }
                        if(!found)
                            throw std::logic_error(
                                "path_rr: a pricing label without "
                                "predecessor");
                    }
                    if(v >= sg.num_vertices)
                        seed.options.push_back(static_cast<option_t>(
                            sg.copy_option[v - sg.num_vertices]));
                    std::ranges::sort(seed.options);
                    priced_columns.push_back(std::move(seed));
                }
                for(const auto & [o, row] : p.purchase_constraints)
                    state.duals[o] = 0.0;
            });
            if(!cut_short) {
                const double bound =
                    master_value + std::accumulate(pricing_bounds.begin(),
                                                   pricing_bounds.end(), 0.0);
                spdlog::info("Lagrangian bound: {}", bound);
                if(!best_bound || bound < best_bound.value())
                    best_bound = bound;
                if(relative_gap > 0 && best_bound.value() - master_value <=
                                           relative_gap * master_value) {
                    spdlog::info(
                        "Column generation stopped: relative gap "
                        "reached");
                    report.status = "relative gap reached";
                    break;
                }
            }
            columns.emplace_pool_columns(priced_columns |
                                         std::views::as_rvalue);
            columns.update_columns([&](const column_seed & seed) {
                return colgen::priced{reduced_cost(seed, dual_solution)};
            });
            columns.purge_pool([](const auto & entry) {
                return entry.second.template get<colgen::reduced_cost>() <= 0;
            });
            auto improving = [&](const auto & entry) {
                const auto & [seed, state] = entry;
                return state.template get<colgen::reduced_cost>() >
                       margin(pairs[seed.pair]);
            };
            const std::size_t num_activated =
                max_activations
                    ? columns
                          .manage_columns(
                              model,
                              colgen::at_most_k_best(
                                  max_activations,
                                  [](const auto & a, const auto & b) {
                                      return a.second.template get<
                                                 colgen::reduced_cost>() >
                                             b.second.template get<
                                                 colgen::reduced_cost>();
                                  },
                                  improving),
                              add_column)
                          .num_activated
                    : columns
                          .manage_columns(model, colgen::all{improving},
                                          add_column)
                          .num_activated;
            if(!num_activated && !cut_short) {
                spdlog::info("Column generation stopped: converged");
                break;
            }
            if(time_up()) {
                spdlog::warn("Column generation stopped: time limit reached");
                report.status = "time limit reached";
                // as in tree_rr: the solvers lose the changed master's solution
                if(num_activated) model.solve();
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(mippp::has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns
                                       .manage_columns(model, colgen::none{},
                                                       unattractive, add_column)
                                       .num_evicted;
        }

        const auto model_solution = model.get_solution();
        for(const auto & i : instance.options()) {
            solution[i] = model_solution[X_vars(i)];
        }

        log_lambda();

        std::size_t num_columns = 0;
        for(const pair_data & p : pairs) num_columns += p.columns.size();
        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pairs", pairs.size()},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted},
                        {"relaxation_value", model.get_solution_value()}};
        if(best_bound)
            report.stats.emplace_back("lagrangian_bound", best_bound.value());
        if(search_pricing)
            report.stats.emplace_back("searches", num_searches.load());
        else
            report.stats.emplace_back("columns", num_columns);
        return {solution, report};
    }
};

}  // namespace solvers
}  // namespace gecot
