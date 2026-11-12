#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <flat_map>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "mippp/utility/column_manager.hpp"

#include "melon/algorithm/dijkstra.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/graph.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"

#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The path formulation relaxation, by column generation.
//
// Buying the options X within the budget, c(X) <= B, a case k scores the sum
// over its pairs of vertices p = (s, t) of
//     v_p(X) = q_s(X) q_t(X) P_st(X),
// the source and target qualities with their gains times the probability of
// the most reliable path from s to t, and the criterion φ combines the cases'
// scores. The relaxation lets each pair p of case k buy its own set of
// options S, a column of value v_pS, as long as it is paid for:
//
//     max  φ(z)
//     s.t. z_k <= C_k + Σ_{p∈k} Σ_S v_pS y_pS    π_k   contribution
//          Σ_S y_pS <= 1                        σ_p   uniqueness
//          Σ_{S∋o} y_pS <= x_o                  λ_po  purchase
//          Σ_o c_o x_o <= B                     μ     budget
//          x, y >= 0
//
// with C_k the value of the pairs that no column improves. Buying X, the
// columns y_pX = 1 are feasible, so the relaxation bounds the optimum.
//
// The master holds a few columns per pair, and each row of a pair once a
// column of the master uses it: until then it is redundant, of dual 0. The
// pricing of a pair finds its best reduced cost
//     rc_pS = π_k v_pS - σ_p - Σ_{o∈S} λ_po,    rc_p = max_S rc_pS,
// and raising each σ_p by max(0, rc_p) makes the master's duals feasible for
// the relaxation's dual, so after a complete pricing round
//     L = master_value + Σ_p max(0, rc_p),
// the Lagrangian bound, bounds the relaxation. In this dual the row of x_o
// has the slack -d_o, with d_o = Σ_p λ_po - c_o μ <= 0 the reduced cost of
// x_o, so a solution that buys o scores at most L + d_o.
//
// Both pricings label the paths to t, backwards from t: a label (S, w, W) is
// a path to t that buys the options S, of value w = q_t(S) P(S) without the
// source quality, and W = Σ_{o∈S} of a weight:
// - search (default): per pair and round, W = Σ_{o∈S} λ_po. Since λ >= 0, a
//   label a is worth no less than a label b if w_a >= w_b and W_a plus the λ
//   of the options of b that a lacks and b could use again is at most W_b;
//   at s, each quality option of s is bought if π_k g_o w > λ_po, g_o its
//   gain. Like the LP above, it ignores the budget: its columns can cost
//   more than B.
// - enumerate: per target, once, W = c(S) <= B, which can tighten the
//   relaxation: a label is worth no less than those of no more value that
//   buy at least its options whatever the duals, so each pair keeps the
//   non-dominated columns, an argmax for any duals.
struct path_formulation_lp : public mip_solver_base {
    using variable_t = mippp::model_variable<int, double>;
    using constraint_t = mippp::model_constraint<int>;
    using vertex = melon::vertex_t<melon::static_digraph>;
    using option_set = std::vector<option_t>;  // sorted
    // the options S of a column of a pair and its value v_pS
    using valued_set = std::pair<option_set, double>;

    std::size_t evict_age = 0;  // in pricing rounds, 0: never evict
    // true: the search pricing; false: the enumerate pricing, for few
    // options per path
    bool search_pricing = true;
    std::size_t max_activations = 0;  // per round, 0: every improving column
    // stops once the least L so far is within this gap of the master's value,
    // relative to it; 0: at convergence only
    double relative_gap = 0;

    static option_set _union(const option_set & A, const option_set & B) {
        option_set S;
        std::ranges::set_union(A, B, std::back_inserter(S));
        return S;
    }
    static auto _in(const option_set & S) {
        return [&S](const auto o) { return std::ranges::binary_search(S, o); };
    }
    // q_v as a source or a target, plus the gains of the options bought
    template <case_c C>
    static double _quality(const C & instance_case, const vertex v,
                           const bool as_source, auto && bought) {
        double q = as_source ? instance_case.source_quality_map()[v]
                             : instance_case.target_quality_map()[v];
        for(const auto & [sqg, tqg, o] : instance_case.vertex_options_map()[v])
            if(bought(o)) q += as_source ? sqg : tqg;
        return q;
    }
    // the allowed options with a gain for v as a source or a target
    template <case_c C>
    static option_set _quality_options(const C & instance_case, const vertex v,
                                       const bool as_source, auto && allowed) {
        option_set S;
        for(const auto & [sqg, tqg, o] : instance_case.vertex_options_map()[v])
            if((as_source ? sqg : tqg) > 0 && allowed(o)) S.push_back(o);
        std::ranges::sort(S);
        S.erase(std::ranges::unique(S).begin(), S.end());
        return S;
    }
    static auto _subsets(const option_set & S) {
        if(S.size() > 20)
            throw std::invalid_argument(
                "path_lp: too many quality options at a vertex");
        return std::views::iota(std::size_t{0}, std::size_t{1} << S.size()) |
               std::views::transform([&S](const std::size_t mask) {
                   option_set A;
                   for(std::size_t i = 0; i < S.size(); ++i)
                       if(mask >> i & 1) A.push_back(S[i]);
                   return A;
               });
    }

    // a path to t that buys the options S, of value w = q_t(S) P(S), with
    // W = Σ_{o∈S} weight(o), and the options of S that it can still use
    struct label {
        double value, weight;
        option_set live;
    };
    // The labels of the paths to t that buy allowed options, by decreasing
    // value, then increasing weight. At t, a label per set of the quality
    // options of t. Along an arc a, a label takes p_a, or p'_oa > p_a for an
    // option o, buying o unless it is live; it keeps live the options that
    // reusable(o, u) accepts at the vertex u it reaches. A label of u is
    // dropped unless keep(l, u), or if dominates(e, l, u) for a label e that
    // left u, and so of no less value, which requires e.weight <= l.weight.
    // visit(u, l, options) sees each label as it leaves u, options() giving
    // the options S of its path, and returns whether it goes further.
    template <case_c C>
    static void _labels(const C & instance_case, const vertex t,
                        auto && allowed, auto && weight, auto && reusable,
                        auto && keep, auto && dominates, auto && visit) {
        const auto & graph = instance_case.graph();
        // a label at the vertex at, extending the label pred by an arc on
        // which it bought the options bought, at t without pred
        struct node {
            label l;
            vertex at;
            std::size_t pred;
            option_set bought;
        };
        constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
        std::vector<node> nodes;
        // the (weight, node) of the labels that left each vertex, by
        // decreasing weight: a label that leaves u has the least weight there
        // when the weights alone decide
        auto left_map = melon::create_vertex_map<
            std::vector<std::pair<double, std::size_t>>>(graph);
        std::priority_queue<std::tuple<double, double, std::size_t>> heap;
        auto dominated = [&](const label & l, const vertex u) {
            for(const auto & [w, n] : std::views::reverse(left_map[u])) {
                if(w > l.weight) return false;
                if(dominates(nodes[n].l, l, u)) return true;
            }
            return false;
        };
        auto live_at = [&](const vertex u, const option_set & S) {
            option_set live;
            for(const option_t o : S)
                if(reusable(o, u)) live.push_back(o);
            return live;
        };
        auto relax = [&](const vertex u, label && l, const std::size_t pred,
                         option_set && bought) {
            if(l.value <= 0 || !keep(l, u) || dominated(l, u)) return;
            heap.emplace(l.value, -l.weight, nodes.size());
            nodes.push_back({std::move(l), u, pred, std::move(bought)});
        };
        const option_set target_options =
            _quality_options(instance_case, t, false, allowed);
        for(option_set A : _subsets(target_options)) {
            label l{_quality(instance_case, t, false, _in(A)), 0.0,
                    live_at(t, A)};
            for(const option_t o : A) l.weight += weight(o);
            relax(t, std::move(l), none, std::move(A));
        }
        while(!heap.empty()) {
            const std::size_t n = std::get<2>(heap.top());
            heap.pop();
            const vertex v = nodes[n].at;
            if(dominated(nodes[n].l, v)) continue;
            const label l = nodes[n].l;  // relax can reallocate
            auto & left = left_map[v];
            left.emplace(std::ranges::upper_bound(left, std::pair{l.weight, n},
                                                  std::ranges::greater{}),
                         l.weight, n);
            if(!visit(v, l, [&] {
                   option_set S;
                   for(std::size_t m = n; m != none; m = nodes[m].pred)
                       S = _union(S, nodes[m].bought);
                   return S;
               }))
                continue;
            for(const auto & a : melon::in_arcs(graph, v)) {
                const vertex u = melon::arc_source(graph, a);
                const double p = instance_case.arc_probability_map()[a];
                relax(u, {l.value * p, l.weight, live_at(u, l.live)}, n, {});
                for(const auto & [improved_prob, o] :
                    instance_case.arc_options_map()[a]) {
                    if(improved_prob <= p || !allowed(o)) continue;
                    if(std::ranges::binary_search(l.live, o))
                        relax(u,
                              {l.value * improved_prob, l.weight,
                               live_at(u, l.live)},
                              n, {});
                    else
                        relax(u,
                              {l.value * improved_prob, l.weight + weight(o),
                               live_at(u, _union(l.live, {o}))},
                              n, {o});
                }
            }
        }
    }

    // Where a label that owns the option o can use it again for free, past
    // the vertex u it is at: anywhere if the arcs of o share no vertex, else
    // at most through an in-arc of u at a vertex they all share, since the
    // label went through those vertices when it bought o.
    struct option_reach {
        // the vertices all the arcs of o share, if o improves arcs
        std::vector<std::optional<std::vector<vertex>>> shared;
        melon::vertex_map_t<melon::static_digraph, option_set> in_options;
        bool operator()(const option_t o, const vertex u) const {
            return shared[o] &&
                   (shared[o]->empty() ||
                    (std::ranges::contains(*shared[o], u) &&
                     std::ranges::binary_search(in_options[u], o)));
        }
    };
    template <instance_c I, case_c C>
    static option_reach _option_reach(const I & instance,
                                      const C & instance_case) {
        const auto & graph = instance_case.graph();
        option_reach reach{std::vector<std::optional<std::vector<vertex>>>(
                               instance.options().size()),
                           melon::create_vertex_map<option_set>(graph)};
        for(const auto & a : melon::arcs(graph))
            for(const auto & [improved_prob, o] :
                instance_case.arc_options_map()[a]) {
                std::vector<vertex> ends{melon::arc_source(graph, a),
                                         melon::arc_target(graph, a)};
                std::ranges::sort(ends);
                auto & shared = reach.shared[o];
                if(shared)
                    std::erase_if(*shared, [&ends](const vertex v) {
                        return !std::ranges::binary_search(ends, v);
                    });
                else
                    shared = std::move(ends);
                reach.in_options[melon::arc_target(graph, a)].push_back(o);
            }
        for(const auto & v : melon::vertices(graph)) {
            auto & options = reach.in_options[v];
            std::ranges::sort(options);
            options.erase(std::ranges::unique(options).begin(), options.end());
        }
        return reach;
    }

    // The non-dominated columns of the pair (s, t) within the budget: each
    // label (S, w) from s to t with each set of the quality options of s.
    template <instance_c I, case_c C>
    static std::vector<valued_set> _pair_columns(
        const I & instance, const C & instance_case, const vertex s,
        const std::vector<valued_set> & labels, const double budget) {
        const option_set source_options =
            _quality_options(instance_case, s, true, [](auto) { return true; });
        std::vector<valued_set> columns;
        for(const auto & [S, w] : labels)
            for(const option_set & A : _subsets(source_options)) {
                valued_set c{_union(S, A), 0.0};
                double cost = 0;
                for(const option_t o : c.first) cost += instance.option_cost(o);
                if(cost > budget) continue;
                c.second = _quality(instance_case, s, true, _in(c.first)) * w;
                if(c.second <= 0 ||
                   std::ranges::any_of(columns, [&c](const valued_set & e) {
                       return e.second >= c.second &&
                              std::ranges::includes(c.first, e.first);
                   }))
                    continue;
                std::erase_if(columns, [&c](const valued_set & e) {
                    return c.second >= e.second &&
                           std::ranges::includes(e.first, c.first);
                });
                columns.push_back(std::move(c));
            }
        return columns;
    }

    // a pair p = (s, t) of a case that some column improves
    struct pair_data {
        case_id_t case_id;
        vertex source, target;
        // its best column with every option, or within the budget for
        // enumerate, whose value bounds v_pS
        valued_set max_column;
        std::vector<valued_set> columns;  // enumerate: the non-dominated
        // its best columns without options and within the start's options
        std::vector<valued_set> start_columns;
    };

    // The pairs of each case, by target, between the vertices of positive
    // quality or quality gain as a source and as a target, and the C_k: the
    // sum of the values of the pairs no column improves, whose best column,
    // with every option or within the budget for enumerate, is worth no more
    // than their column without.
    template <instance_c I>
    std::pair<std::vector<pair_data>, std::vector<double>> _pairs(
        const I & instance, const double budget,
        const std::optional<instance_solution_t<I>> & start) const {
        auto none = [](auto) { return false; };
        auto all = [](auto) { return true; };
        auto in_start = [&start](auto o) { return start && (*start)[o]; };
        std::vector<pair_data> pairs;
        std::vector<double> constants;
        for(auto && instance_case : instance.cases()) {
            const auto & graph = instance_case.graph();
            auto is_source = melon::create_vertex_map<char>(graph, 0);
            std::vector<vertex> sources, targets;
            for(const auto & v : melon::vertices(graph)) {
                if(_quality(instance_case, v, true, all) > 0) {
                    is_source[v] = 1;
                    sources.push_back(v);
                }
                if(_quality(instance_case, v, false, all) > 0)
                    targets.push_back(v);
            }
            // per target, the constant pairs' values and the other pairs, kept
            // in order so the master is the same from run to run
            std::vector<std::pair<double, std::vector<pair_data>>> target_pairs(
                targets.size());
            tbb::parallel_for(std::size_t{0}, targets.size(), [&](auto i) {
                const vertex t = targets[i];
                auto & [constant, t_pairs] = target_pairs[i];
                auto add = [&](const vertex s,
                               const std::optional<valued_set> & max_column,
                               std::vector<valued_set> columns,
                               const std::optional<valued_set> & base,
                               const std::optional<valued_set> & from_start) {
                    const double base_value = base ? base->second : 0.0;
                    if(!max_column || max_column->second <= base_value) {
                        constant += base_value;
                        return;
                    }
                    std::vector<valued_set> start_columns;
                    for(const auto & c : {base, from_start})
                        if(c) start_columns.push_back(*c);
                    t_pairs.push_back({instance_case.id(), s, t, *max_column,
                                       std::move(columns),
                                       std::move(start_columns)});
                };
                if(!search_pricing) {
                    // every option bought stays live
                    auto labels =
                        melon::create_vertex_map<std::vector<valued_set>>(
                            graph);
                    _labels(
                        instance_case, t, all,
                        [&](const option_t o) {
                            return instance.option_cost(o);
                        },
                        [](auto, auto) { return true; },
                        [budget](const label & l, vertex) {
                            return l.weight <= budget;
                        },
                        [](const label & a, const label & b, vertex) {
                            return a.value >= b.value &&
                                   std::ranges::includes(b.live, a.live);
                        },
                        [&](const vertex u, const label & l, auto && options) {
                            if(is_source[u])
                                labels[u].emplace_back(options(), l.value);
                            return true;
                        });
                    for(const vertex s : sources) {
                        auto columns = _pair_columns(instance, instance_case, s,
                                                     labels[s], budget);
                        std::optional<valued_set> max_column, base, from_start;
                        for(const valued_set & c : columns) {
                            if(!max_column || c.second > max_column->second)
                                max_column = c;
                            if(c.first.empty()) base = c;
                            if(std::ranges::all_of(c.first, in_start) &&
                               (!from_start || c.second > from_start->second))
                                from_start = c;
                        }
                        add(s, max_column, std::move(columns), base,
                            from_start);
                    }
                    return;
                }
                // per vertex, the best column of its pair with t that buys
                // allowed options, all free: the first label to reach it,
                // with the allowed quality options of the source
                auto best_columns = [&](auto && allowed) {
                    auto best =
                        melon::create_vertex_map<std::optional<valued_set>>(
                            graph);
                    _labels(
                        instance_case, t, allowed, [](auto) { return 0.0; },
                        [](auto, auto) { return false; },
                        [](const label &, vertex) { return true; },
                        [](const label & a, const label & b, vertex) {
                            return a.value >= b.value;
                        },
                        [&](const vertex u, const label & l, auto && options) {
                            if(!is_source[u] || best[u]) return true;
                            option_set S = _union(
                                options(), _quality_options(instance_case, u,
                                                            true, allowed));
                            const double v =
                                _quality(instance_case, u, true, _in(S)) *
                                l.value;
                            if(v > 0) best[u].emplace(std::move(S), v);
                            return true;
                        });
                    return best;
                };
                const auto base = best_columns(none);
                const auto max = best_columns(all);
                const auto from_start = start ? best_columns(in_start) : base;
                for(const vertex s : sources)
                    add(s, max[s], {}, base[s], from_start[s]);
            });
            double constant = 0;
            for(auto & [value, t_pairs] : target_pairs) {
                constant += value;
                std::ranges::move(t_pairs, std::back_inserter(pairs));
            }
            constants.push_back(constant);
        }
        return {std::move(pairs), std::move(constants)};
    }

    template <instance_c I>
    struct relaxation {
        instance_solution_t<I> solution;
        solve_report report;
        // per option, a bound on the score of the solutions that buy it:
        // the least L + d_o over the complete pricing rounds
        std::vector<double> option_bounds;
    };

    template <typename LP, instance_c I>
    solve_result<I> solve(const I & instance, const double budget) const {
        auto [solution, report, option_bounds] =
            this->template relax<LP>(instance, budget);
        return {std::move(solution), std::move(report)};
    }

    template <typename LP, instance_c I>
    relaxation<I> relax(const I & instance, const double budget) const {
        namespace colgen = mippp::colgen;
        using namespace mippp;
        using namespace mippp::operators;
        _check_criterion(instance);
        spdlog::stopwatch sw;
        auto time_up = [&] {
            return time_limit && sw.elapsed().count() > time_limit.value();
        };

        std::optional<instance_solution_t<I>> start;
        if(mip_start) {
            if(compute_solution_cost(instance, _mip_start_solution(instance)) >
               budget + feasibility_tol)
                spdlog::warn("MIP start exceeds the budget: ignored");
            else
                start = _mip_start_solution(instance);
        }
        // enumerate's columns fit in budget + feasibility_tol, as the start
        // and the solution returned do, or L can be below their score
        const auto [pairs, constants] =
            _pairs(instance, budget + feasibility_tol, start);
        std::vector<option_reach> reaches;
        // per case, source s of a pair and vertex u, the probability of the
        // most reliable s-u path with every option: the share of its value
        // that a label at u keeps at best on its way to s
        std::vector<std::vector<std::vector<double>>> source_reaches;
        if(search_pricing)
            for(const auto & instance_case : instance.cases()) {
                reaches.push_back(_option_reach(instance, instance_case));
                const auto & graph = instance_case.graph();
                auto probability_map = instance_case.arc_probability_map();
                for(const auto & a : melon::arcs(graph))
                    for(const auto & [improved_prob, o] :
                        instance_case.arc_options_map()[a])
                        probability_map[a] =
                            std::max(probability_map[a], improved_prob);
                auto & case_reaches =
                    source_reaches.emplace_back(graph.num_vertices());
                std::vector<vertex> sources;
                for(const pair_data & p : pairs)
                    if(p.case_id == instance_case.id() &&
                       case_reaches[p.source].empty()) {
                        case_reaches[p.source].assign(graph.num_vertices(),
                                                      0.0);
                        sources.push_back(p.source);
                    }
                tbb::parallel_for(
                    std::size_t{0}, sources.size(), [&](const std::size_t i) {
                        for(const auto & [u, prob] : melon::dijkstra(
                                detail::pc_num_dijkstra_traits<
                                    std::decay_t<decltype(graph)>, double>{},
                                graph, probability_map, sources[i]))
                            case_reaches[sources[i]][u] = prob;
                    });
            }

        // The master: x, the budget row, and for each case k, z_k with its
        // contribution row; the rows of a pair come with the first column
        // that uses them.
        LP model;
        if constexpr(has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        model.set_maximization();
        const auto x = model.add_variables(
            instance.options().size(), [](const option_t & o) { return o; });
        const auto budget_row =
            model.add_constraint(xsum(instance.options(), [&](auto && o) {
                                     return instance.option_cost(o) * x(o);
                                 }) <= budget);
        std::vector<variable_t> z;
        std::vector<constraint_t> contribution_rows;
        for(const double C_k : constants) {
            z.push_back(model.add_variable());
            contribution_rows.push_back(model.add_constraint(z.back() <= C_k));
        }
        model.set_objective(std::visit(
            formula_variable_visitor{model,
                                     [&](const case_id_t & k) { return z[k]; }},
            instance.criterion()));
        std::vector<std::optional<constraint_t>> uniqueness_rows(pairs.size());
        std::vector<std::flat_map<option_t, constraint_t>> purchase_rows(
            pairs.size());

        // The master's value and x at its last solve: the solvers give no
        // solution for a master changed since, or stopped by the time limit.
        double master_value = 0;
        auto x_values = instance.create_option_map(0.0);
        auto log_master = [&, option_name_max_length = std::ranges::max(
                                  std::ranges::views::transform(
                                      instance.options(), [&](auto && o) {
                                          return instance.option_name(o).size();
                                      }))]() {
            spdlog::trace("path formulation relaxation has value: {}",
                          master_value);
            for(auto && option : instance.options()) {
                const auto & option_name = instance.option_name(option);
                spdlog::trace("    {:<{}} {}", option_name,
                              option_name_max_length, x_values[option]);
            }
        };

        // a column y_pS, the key of the column manager: the index p of its
        // pair and its options S; its value v_pS is the data
        using column = std::pair<std::size_t, option_set>;
        colgen::column_manager<
            LP, column, colgen::property_list<colgen::reduced_cost>,
            colgen::property_list<colgen::reduced_cost, colgen::age>, double>
            columns;
        auto add_column = [&](LP &, const auto & entry) {
            const auto & [p, S] = entry.seed;
            if(!uniqueness_rows[p])
                uniqueness_rows[p] = model.add_constraint(
                    empty_linear_expression<variable_t, double> <= 1);
            for(const option_t o : S)
                if(!purchase_rows[p].contains(o))
                    purchase_rows[p].emplace(
                        o, model.add_constraint(
                               empty_linear_expression<variable_t, double> <=
                               x(o)));
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_rows[pairs[p].case_id], -entry.data)),
                std::views::single(std::make_pair(*uniqueness_rows[p], 1.0)),
                std::views::transform(S, [&](const option_t o) {
                    return std::make_pair(purchase_rows[p].at(o), 1.0);
                })));
        };
        for(std::size_t p = 0; p < pairs.size(); ++p)
            for(const auto & [S, v] : pairs[p].start_columns)
                columns.emplace_master_column(model, column{p, S}, add_column,
                                              v);

        // in the master for more than evict_age rounds and unattractive, so
        // nonbasic
        const colgen::conjunction<colgen::above<colgen::age>,
                                  colgen::negative<colgen::reduced_cost>>
            unattractive{colgen::above<colgen::age>{evict_age},
                         colgen::negative<colgen::reduced_cost>{1e-6}};
        if constexpr(!has_remove_variable<LP>)
            if(evict_age)
                spdlog::warn(
                    "This solver cannot remove columns: --evict-age ignored");

        // per pair, the duals of its last search and the column it found,
        // which the same duals find again
        struct last_search {
            std::vector<double> duals;  // π_k, σ_p and the λ_po of its rows
            std::optional<std::pair<double, valued_set>> best;  // rc, column
        };
        std::vector<last_search> searches(pairs.size());
        std::atomic<std::size_t> num_searches = 0;

        // Column generation.
        solve_report report{"converged"};
        std::size_t iterations = 1;  // pricing rounds
        std::size_t num_evicted = 0;
        // per pair, max(0, rc_p) or a bound on it
        std::vector<double> pricing_bounds(pairs.size());
        std::vector<double> option_bounds(
            instance.options().size(), std::numeric_limits<double>::infinity());
        for(;; ++iterations) {
            if constexpr(has_time_limit<LP>)
                if(time_limit)
                    model.set_time_limit(std::chrono::duration<double>(std::max(
                        0.0, time_limit.value() - sw.elapsed().count())));
            model.solve();
            if(!is_a<status::optimal>(model.get_status())) {
                report.status = stop_requested ? "interrupted"
                                : time_up()    ? "time limit reached"
                                               : "master not solved";
                spdlog::warn("Column generation stopped: {}", report.status);
                break;
            }
            const auto dual_solution = model.get_dual_solution();
            master_value = model.get_solution_value();
            {
                const auto model_solution = model.get_solution();
                for(auto && o : instance.options())
                    x_values[o] = model_solution[x(o)];
            }
            log_master();

            auto pi = [&](const std::size_t p) {
                return dual_solution[contribution_rows[pairs[p].case_id]];
            };
            auto sigma = [&](const std::size_t p) {
                return uniqueness_rows[p] ? dual_solution[*uniqueness_rows[p]]
                                          : 0.0;
            };
            auto lambda = [&](const std::size_t p, const option_t o) {
                const auto it = purchase_rows[p].find(o);
                return it == purchase_rows[p].end() ? 0.0
                                                    : dual_solution[it->second];
            };
            auto reduced_cost = [&](const std::size_t p, const option_set & S,
                                    const double v) {
                double rc = pi(p) * v - sigma(p);
                for(const option_t o : S) rc -= lambda(p, o);
                return rc;
            };
            // the pricing test, at the master's dual tolerance: rc > margin
            auto margin = [&](const std::size_t p) {
                return 1e-7 * sigma(p) + 1e-6;
            };

            tbb::concurrent_vector<std::pair<column, double>> priced_columns;
            // the pricing of pair p found the column c, of reduced cost rc
            auto found = [&](const std::size_t p, const double rc,
                             const valued_set & c) {
                pricing_bounds[p] = rc;
                priced_columns.push_back({column{p, c.first}, c.second});
            };
            auto enumerate = [&](const std::size_t p) {
                const valued_set * best = nullptr;
                double best_rc = 0;
                for(const valued_set & c : pairs[p].columns)
                    if(const double rc = reduced_cost(p, c.first, c.second);
                       !best || rc > best_rc) {
                        best = &c;
                        best_rc = rc;
                    }
                pricing_bounds[p] = std::max(best_rc, 0.0);
                if(best && best_rc > margin(p)) found(p, best_rc, *best);
            };
            std::atomic<bool> cut_short = false;  // by a stop
            // The search of pair p. The labels only lose value and gain
            // weight, so a label at u is kept while π_k q_s(every option) w
            // R_su - W - σ_p > margin, R_su the probability of the most
            // reliable s-u path with every option, which bounds max(0, rc_p)
            // when none is found; so does the max-value skip. Without a
            // positive purchase dual, the
            // best column is the one with every option. The master can return
            // a dual a little below 0 (-7.6e-7 on Quebec), which counts as 0:
            // the search needs weights that never improve a label.
            auto search = [&](const std::size_t p) {
                const pair_data & data = pairs[p];
                const double pi_p = pi(p), sigma_p = sigma(p),
                             margin_p = margin(p);
                pricing_bounds[p] = margin_p;
                const double max_rc = pi_p * data.max_column.second - sigma_p;
                if(max_rc <= margin_p) return;
                auto lambda_plus = [&](const option_t o) {
                    return std::max(0.0, lambda(p, o));
                };
                if(std::ranges::none_of(
                       std::views::keys(purchase_rows[p]),
                       [&](const option_t o) { return lambda_plus(o) > 0; }))
                    return found(p, max_rc, data.max_column);
                std::vector<double> duals{pi_p, sigma_p};
                for(const auto & [o, row] : purchase_rows[p])
                    duals.push_back(dual_solution[row]);
                if(duals == searches[p].duals) {
                    if(const auto & best = searches[p].best)
                        found(p, best->first, best->second);
                    return;
                }
                if(stop_requested || time_up()) {
                    cut_short = true;
                    return;
                }
                ++num_searches;
                const auto & instance_case = *std::ranges::next(
                    instance.cases().begin(),
                    static_cast<std::ptrdiff_t>(data.case_id));
                const option_reach & reach = reaches[data.case_id];
                const vertex s = data.source;
                auto all = [](auto) { return true; };
                const option_set source_options =
                    _quality_options(instance_case, s, true, all);
                const option_set target_options =
                    _quality_options(instance_case, data.target, false, all);
                const double q_max = _quality(instance_case, s, true, all);
                const auto & source_reach = source_reaches[data.case_id][s];
                std::optional<std::pair<double, valued_set>> best;
                // a label can use again its options of a gain at s, those of
                // a gain at t, which it can have bought before their arcs,
                // and those that reach accepts
                _labels(
                    instance_case, data.target, all, lambda_plus,
                    [&](const option_t o, const vertex u) {
                        return std::ranges::binary_search(source_options, o) ||
                               std::ranges::binary_search(target_options, o) ||
                               reach(o, u);
                    },
                    [&](const label & l, const vertex u) {
                        return pi_p * q_max * l.value * source_reach[u] *
                                       (1 + 1e-12) -
                                   l.weight - sigma_p >
                               margin_p;
                    },
                    // the live options of b that a lacks cost a their λ
                    [&](const label & a, const label & b, vertex) {
                        if(a.value < b.value || a.weight > b.weight)
                            return false;
                        double weight = a.weight;
                        for(const option_t o : b.live)
                            if(!std::ranges::binary_search(a.live, o))
                                weight += lambda_plus(o);
                        return weight <= b.weight;
                    },
                    [&](const vertex u, const label & l, auto && options) {
                        if(u != s) return true;
                        // each quality option of s, with all its gains at s
                        // (an option can have several), if worth its λ
                        valued_set c{options(), 0.0};
                        double rc = -sigma_p - l.weight;
                        for(const option_t o : source_options) {
                            if(std::ranges::binary_search(l.live, o)) continue;
                            double gain = 0;
                            for(const auto & [sqg, tqg, e] :
                                instance_case.vertex_options_map()[s])
                                if(e == o) gain += sqg;
                            if(pi_p * gain * l.value > lambda_plus(o)) {
                                rc -= lambda_plus(o);
                                c.first = _union(c.first, {o});
                            }
                        }
                        c.second =
                            _quality(instance_case, s, true, _in(c.first)) *
                            l.value;
                        rc += pi_p * c.second;
                        if(!best || rc > best->first)
                            best.emplace(rc, std::move(c));
                        return false;  // a path through s again has a cycle
                    });
                searches[p] = {std::move(duals), {}};
                if(best && best->first > margin_p) {
                    searches[p].best = best;
                    found(p, best->first, best->second);
                }
            };
            tbb::parallel_for(std::size_t{0}, pairs.size(),
                              [&](const std::size_t p) {
                                  if(search_pricing)
                                      search(p);
                                  else
                                      enumerate(p);
                              });

            // L = master_value + Σ_p max(0, rc_p), after a complete round
            if(!cut_short) {
                const double bound =
                    master_value + std::accumulate(pricing_bounds.begin(),
                                                   pricing_bounds.end(), 0.0);
                if(!report.best_bound || bound < report.best_bound.value())
                    report.best_bound = bound;
                // the purchase duals as the search counts them, a little
                // below 0 as 0
                std::vector<double> reduced_costs(instance.options().size());
                for(auto && o : instance.options())
                    reduced_costs[o] =
                        -instance.option_cost(o) * dual_solution[budget_row];
                for(const auto & rows : purchase_rows)
                    for(const auto & [o, row] : rows)
                        reduced_costs[o] += std::max(0.0, dual_solution[row]);
                for(auto && o : instance.options())
                    option_bounds[o] =
                        std::min(option_bounds[o],
                                 bound + std::min(0.0, reduced_costs[o]));
                report_progress({.bound = report.best_bound,
                                 .relaxation = master_value,
                                 .round = iterations});
                if(relative_gap > 0 &&
                   report.best_bound.value() - master_value <=
                       relative_gap * master_value) {
                    spdlog::debug(
                        "Column generation stopped: relative gap reached");
                    report.status = "relative gap reached";
                    break;
                }
            }

            // the improving columns enter the master, or the max_activations
            // best of them
            columns.emplace_pool_columns(priced_columns |
                                         std::views::as_rvalue);
            columns.update_columns([&](const auto & entry) {
                return colgen::priced{reduced_cost(
                    entry.seed.first, entry.seed.second, entry.data)};
            });
            columns.purge_pool([](const auto & entry) {
                return entry.state.template get<colgen::reduced_cost>() <= 0;
            });
            auto improving = [&](const auto & entry) {
                return entry.state.template get<colgen::reduced_cost>() >
                       margin(entry.seed.first);
            };
            const std::size_t num_activated =
                max_activations
                    ? columns.activate_columns(
                          model, improving, add_column,
                          colgen::at_most_k_best(
                              max_activations,
                              [](const auto & a, const auto & b) {
                                  return a.state.template get<
                                             colgen::reduced_cost>() >
                                         b.state.template get<
                                             colgen::reduced_cost>();
                              }))
                    : columns.activate_columns(model, improving, add_column);
            if(!num_activated && !cut_short) {
                spdlog::debug("Column generation stopped: converged");
                break;
            }
            if(stop_requested || time_up()) {
                report.status =
                    stop_requested ? "interrupted" : "time limit reached";
                spdlog::warn("Column generation stopped: {}", report.status);
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns.evict_columns(model, unattractive);
        }

        // x cast to bool can exceed the budget: the options by decreasing x
        // that fit in it, or the start if it scores more
        auto options = std::ranges::to<std::vector>(instance.options());
        std::ranges::stable_sort(options,
                                 [&](const option_t a, const option_t b) {
                                     return x_values[a] > x_values[b];
                                 });
        auto solution = instance.create_option_map(false);
        double purchased = 0.0;
        for(const option_t o : options) {
            const double price = instance.option_cost(o);
            if(purchased + price > budget + feasibility_tol) continue;
            purchased += price;
            solution[o] = true;
        }
        auto score = [&](const auto & s) {
            return instance.eval_criterion(
                compute_solution_cases_pc_num(instance, s));
        };
        if(start && score(start.value()) > score(solution))
            solution = start.value();
        log_master();

        std::size_t num_columns = 0;
        for(const pair_data & p : pairs) num_columns += p.columns.size();
        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pairs", pairs.size()},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted}};
        report.score_stats = {{"relaxation_value", master_value}};
        if(search_pricing)
            report.stats.emplace_back("searches", num_searches.load());
        else
            report.stats.emplace_back("columns", num_columns);
        return {std::move(solution), std::move(report),
                std::move(option_bounds)};
    }
};

}  // namespace solvers
}  // namespace gecot
