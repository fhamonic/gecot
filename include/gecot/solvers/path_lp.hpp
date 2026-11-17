#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <flat_map>
#include <format>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
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

#include "gecot/solvers/column_generation.hpp"
#include "gecot/solvers/mip_solver_base.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The path relaxation of docs/formulations/relaxations.md, by column
// generation: its program, its two pricings and their labels, its Lagrangian
// bound and the bound of each option are stated there.
struct path_lp : public mip_solver_base {
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
        return [&S](const auto i) { return std::ranges::binary_search(S, i); };
    }
    // the quality of v as a source or as a target, plus the gains of the
    // options bought
    template <case_c C>
    static double _quality(const C & instance_case, const vertex v,
                           const bool as_source, auto && bought) {
        double q = as_source ? instance_case.source_quality_map()[v]
                             : instance_case.target_quality_map()[v];
        for(const auto & [sqg, tqg, i] : instance_case.vertex_options_map()[v])
            if(bought(i)) q += as_source ? sqg : tqg;
        return q;
    }
    // the allowed options with a gain for v as a source or a target
    template <case_c C>
    static option_set _quality_options(const C & instance_case, const vertex v,
                                       const bool as_source, auto && allowed) {
        option_set S;
        for(const auto & [sqg, tqg, i] : instance_case.vertex_options_map()[v])
            if((as_source ? sqg : tqg) > 0 && allowed(i)) S.push_back(i);
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
                   for(std::size_t j = 0; j < S.size(); ++j)
                       if(mask >> j & 1) A.push_back(S[j]);
                   return A;
               });
    }

    // a path to t that buys the options S, of value ω = q^S_t Π without the
    // source quality, with W = Σ_{i∈S} weight(i), and the options of S that
    // it can still use
    struct label {
        double value, weight;
        option_set live;
    };
    // The labels of the paths to t that buy allowed options, by decreasing
    // value, then increasing weight. At t, a label per set of the quality
    // options of t. Along an arc a, a label takes π_a, or π^i_a > π_a for an
    // option i, buying i unless it is live; it keeps live the options that
    // reusable(i, u) accepts at the vertex u it reaches. A label of u is
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
            for(const option_t i : S)
                if(reusable(i, u)) live.push_back(i);
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
            for(const option_t i : A) l.weight += weight(i);
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
                for(const auto & [improved_probability, i] :
                    instance_case.arc_options_map()[a]) {
                    if(improved_probability <= p || !allowed(i)) continue;
                    if(std::ranges::binary_search(l.live, i))
                        relax(u,
                              {l.value * improved_probability, l.weight,
                               live_at(u, l.live)},
                              n, {});
                    else
                        relax(u,
                              {l.value * improved_probability,
                               l.weight + weight(i),
                               live_at(u, _union(l.live, {i}))},
                              n, {i});
                }
            }
        }
    }

    // Where a label that owns the option i can use it again for free, past
    // the vertex u it is at: anywhere if the arcs of i share no vertex, else
    // at most through an in-arc of u at a vertex they all share, since the
    // label went through those vertices when it bought i.
    struct option_reach {
        // the vertices all the arcs of i share, if i improves arcs
        std::vector<std::optional<std::vector<vertex>>> shared;
        melon::vertex_map_t<melon::static_digraph, option_set> in_options;
        bool operator()(const option_t i, const vertex u) const {
            return shared[i] &&
                   (shared[i]->empty() ||
                    (std::ranges::contains(*shared[i], u) &&
                     std::ranges::binary_search(in_options[u], i)));
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
            for(const auto & [improved_probability, i] :
                instance_case.arc_options_map()[a]) {
                std::vector<vertex> ends{melon::arc_source(graph, a),
                                         melon::arc_target(graph, a)};
                std::ranges::sort(ends);
                auto & shared = reach.shared[i];
                if(shared)
                    std::erase_if(*shared, [&ends](const vertex v) {
                        return !std::ranges::binary_search(ends, v);
                    });
                else
                    shared = std::move(ends);
                reach.in_options[melon::arc_target(graph, a)].push_back(i);
            }
        for(const auto & v : melon::vertices(graph)) {
            auto & options = reach.in_options[v];
            std::ranges::sort(options);
            options.erase(std::ranges::unique(options).begin(), options.end());
        }
        return reach;
    }

    // S within the budgets beyond the dimension 0, which the weights carry,
    // and without two options of a conflict set
    template <instance_c I>
    static bool _fits_beyond_first(const I & instance, const budget_t & budget,
                                   const option_set & S) {
        for(std::size_t r = 1; r < budget.size(); ++r) {
            double cost = 0;
            for(const option_t i : S) cost += instance.option_cost(i, r);
            if(cost > budget[r]) return false;
        }
        return std::ranges::none_of(instance.conflict_sets(), [&](auto && set) {
            return std::ranges::count_if(set, [&](auto && i) {
                       return std::ranges::binary_search(S, i);
                   }) > 1;
        });
    }

    // The non-dominated columns of the pair (s, t) within the budget: each
    // label (S, ω) from s to t with each set of the quality options of s.
    template <instance_c I, case_c C>
    static std::vector<valued_set> _pair_columns(
        const I & instance, const C & instance_case, const vertex s,
        const std::vector<valued_set> & labels, const budget_t & budget) {
        const bool plain = has_plain_budget(instance);
        const option_set source_options =
            _quality_options(instance_case, s, true, [](auto) { return true; });
        std::vector<valued_set> columns;
        for(const auto & [S, w] : labels)
            for(const option_set & A : _subsets(source_options)) {
                valued_set c{_union(S, A), 0.0};
                double cost = 0;
                for(const option_t i : c.first)
                    cost += instance.option_cost(i, 0);
                if(cost > budget.front() ||
                   (!plain && !_fits_beyond_first(instance, budget, c.first)))
                    continue;
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

    // The duals of the rows of a pair at a round: γ_k and σ_p, and the λ_pi
    // read on demand, 0 for an option without a purchase row.
    template <typename D>
    struct pair_duals {
        double gamma, sigma;
        const std::flat_map<option_t, constraint_t> & purchase_rows;
        const D & dual_solution;

        double lambda(const option_t i) const {
            const auto it = purchase_rows.find(i);
            return it == purchase_rows.end() ? 0.0 : dual_solution[it->second];
        }
        double reduced_cost(const option_set & S, const double v) const {
            double rc = gamma * v - sigma;
            for(const option_t i : S) rc -= lambda(i);
            return rc;
        }
        double margin() const { return pricing_margin(sigma); }
    };
    // The pricing of a pair: max(0, r_p) or a bound on it, and its best
    // column if its reduced cost is above the margin, which the pricer or
    // the pair keeps.
    struct pair_pricing {
        double bound;
        const valued_set * column = nullptr;
    };

    // The two pricings. Each gives target_pairs(...), which calls add for
    // the pair of each source with a target; price(p, pair, duals, stop),
    // none if stop() cut it short; and the stat of its work.

    // enumerate: the non-dominated columns of each pair within the budget
    // are built with the pairs, and a round takes the best of them.
    struct enumerate_pricer {
        template <instance_c I, case_c C>
        static void target_pairs(
            const I & instance, const C & instance_case,
            const budget_t & budget,
            const std::optional<instance_solution_t<I>> & start, const vertex t,
            const std::vector<vertex> & sources, const auto & is_source,
            auto && add) {
            const auto & graph = instance_case.graph();
            const bool plain = has_plain_budget(instance);
            auto all = [](auto) { return true; };
            auto in_start = [&start](auto i) { return start && (*start)[i]; };
            // every option bought stays live
            auto labels =
                melon::create_vertex_map<std::vector<valued_set>>(graph);
            _labels(
                instance_case, t, all,
                [&](const option_t i) { return instance.option_cost(i, 0); },
                [](auto, auto) { return true; },
                [&](const label & l, vertex) {
                    return l.weight <= budget.front() &&
                           (plain ||
                            _fits_beyond_first(instance, budget, l.live));
                },
                [](const label & a, const label & b, vertex) {
                    return a.value >= b.value &&
                           std::ranges::includes(b.live, a.live);
                },
                [&](const vertex u, const label & l, auto && options) {
                    if(is_source[u]) labels[u].emplace_back(options(), l.value);
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
                add(s, max_column, std::move(columns), base, from_start);
            }
        }
        static std::optional<pair_pricing> price(std::size_t,
                                                 const pair_data & pair,
                                                 const auto & duals, auto &&) {
            const valued_set * best = nullptr;
            double best_rc = 0;
            for(const valued_set & c : pair.columns)
                if(const double rc = duals.reduced_cost(c.first, c.second);
                   !best || rc > best_rc) {
                    best = &c;
                    best_rc = rc;
                }
            if(best && best_rc > duals.margin())
                return pair_pricing{best_rc, best};
            return pair_pricing{std::max(best_rc, 0.0)};
        }
        static std::pair<std::string, double> stat(
            const std::vector<pair_data> & pairs) {
            std::size_t num_columns = 0;
            for(const pair_data & p : pairs) num_columns += p.columns.size();
            return {"columns", num_columns};
        }
    };

    // search: a search of labels per pair and round, without the budget. It
    // keeps, per pair, the duals of its last search and the column it found,
    // which the same duals find again.
    template <instance_c I>
    struct search_pricer {
        const I & instance;
        std::vector<option_reach> reaches;  // per case
        // per case, source s of a pair and vertex u, the probability of the
        // most reliable s-u path with every option: the share of its value
        // that a label at u keeps at best on its way to s
        std::vector<std::vector<std::vector<double>>> source_reaches;
        struct last_search {
            std::vector<double> duals;  // γ_k, σ_p and the λ_pi of its rows
            std::optional<std::pair<double, valued_set>> best;  // rc, column
        };
        std::vector<last_search> searches;  // per pair
        std::atomic<std::size_t> num_searches = 0;

        search_pricer(const I & t_instance,
                      const std::vector<pair_data> & pairs)
            : instance(t_instance), searches(pairs.size()) {
            for(const auto & instance_case : instance.cases()) {
                reaches.push_back(_option_reach(instance, instance_case));
                const auto & graph = instance_case.graph();
                auto probability_map = instance_case.arc_probability_map();
                for(const auto & a : melon::arcs(graph))
                    for(const auto & [improved_probability, i] :
                        instance_case.arc_options_map()[a])
                        probability_map[a] =
                            std::max(probability_map[a], improved_probability);
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
                    std::size_t{0}, sources.size(), [&](const std::size_t j) {
                        for(const auto & [u, prob] : melon::dijkstra(
                                detail::pc_num_dijkstra_traits<
                                    std::decay_t<decltype(graph)>, double>{},
                                graph, probability_map, sources[j]))
                            case_reaches[sources[j]][u] = prob;
                    });
            }
        }

        template <case_c C>
        static void target_pairs(
            const I &, const C & instance_case, const budget_t &,
            const std::optional<instance_solution_t<I>> & start, const vertex t,
            const std::vector<vertex> & sources, const auto & is_source,
            auto && add) {
            const auto & graph = instance_case.graph();
            auto none = [](auto) { return false; };
            auto all = [](auto) { return true; };
            auto in_start = [&start](auto i) { return start && (*start)[i]; };
            // per vertex, the best column of its pair with t that buys
            // allowed options, all free: the first label to reach it,
            // with the allowed quality options of the source
            auto best_columns = [&](auto && allowed) {
                auto best =
                    melon::create_vertex_map<std::optional<valued_set>>(graph);
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
                            options(),
                            _quality_options(instance_case, u, true, allowed));
                        const double v =
                            _quality(instance_case, u, true, _in(S)) * l.value;
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
        }

        // The search of the pair p. The labels only lose value and gain
        // weight, so a label at u is kept while γ_k w̄_s ω R_su - W - σ_p >
        // margin, w̄_s the source quality with every option and R_su the
        // probability of the most reliable s-u path with every option,
        // which bounds max(0, r_p) when none is found; so does the
        // max-value skip. Without a positive purchase dual, the best
        // column is the one with every option. The master can return
        // a dual a little below 0 (-7.6e-7 on Quebec), which counts as 0:
        // the search needs weights that never improve a label.
        // A stop is only asked before a search: a pair that needs none never
        // cuts a round short.
        std::optional<pair_pricing> price(const std::size_t p,
                                          const pair_data & data,
                                          const auto & duals, auto && stop) {
            const double gamma_p = duals.gamma, sigma_p = duals.sigma,
                         margin_p = duals.margin();
            const double max_rc = gamma_p * data.max_column.second - sigma_p;
            if(max_rc <= margin_p) return pair_pricing{margin_p};
            auto lambda_plus = [&](const option_t i) {
                return std::max(0.0, duals.lambda(i));
            };
            if(std::ranges::none_of(
                   std::views::keys(duals.purchase_rows),
                   [&](const option_t i) { return lambda_plus(i) > 0; }))
                return pair_pricing{max_rc, &data.max_column};
            std::vector<double> search_duals{gamma_p, sigma_p};
            for(const auto & [i, row] : duals.purchase_rows)
                search_duals.push_back(duals.dual_solution[row]);
            if(search_duals == searches[p].duals) {
                if(const auto & best = searches[p].best)
                    return pair_pricing{best->first, &best->second};
                return pair_pricing{margin_p};
            }
            if(stop()) return std::nullopt;
            ++num_searches;
            const auto & instance_case =
                *std::ranges::next(instance.cases().begin(),
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
                [&](const option_t i, const vertex u) {
                    return std::ranges::binary_search(source_options, i) ||
                           std::ranges::binary_search(target_options, i) ||
                           reach(i, u);
                },
                [&](const label & l, const vertex u) {
                    return gamma_p * q_max * l.value * source_reach[u] *
                                   (1 + 1e-12) -
                               l.weight - sigma_p >
                           margin_p;
                },
                // the live options of b that a lacks cost a their λ
                [&](const label & a, const label & b, vertex) {
                    if(a.value < b.value || a.weight > b.weight) return false;
                    double weight = a.weight;
                    for(const option_t i : b.live)
                        if(!std::ranges::binary_search(a.live, i))
                            weight += lambda_plus(i);
                    return weight <= b.weight;
                },
                [&](const vertex u, const label & l, auto && options) {
                    if(u != s) return true;
                    // each quality option of s, with all its gains at s
                    // (an option can have several), if worth its λ
                    valued_set c{options(), 0.0};
                    double rc = -sigma_p - l.weight;
                    for(const option_t i : source_options) {
                        if(std::ranges::binary_search(l.live, i)) continue;
                        double gain = 0;
                        for(const auto & [sqg, tqg, e] :
                            instance_case.vertex_options_map()[s])
                            if(e == i) gain += sqg;
                        if(gamma_p * gain * l.value > lambda_plus(i)) {
                            rc -= lambda_plus(i);
                            c.first = _union(c.first, {i});
                        }
                    }
                    c.second = _quality(instance_case, s, true, _in(c.first)) *
                               l.value;
                    rc += gamma_p * c.second;
                    if(!best || rc > best->first)
                        best.emplace(rc, std::move(c));
                    return false;  // a path through s again has a cycle
                });
            searches[p] = {std::move(search_duals), {}};
            if(best && best->first > margin_p) {
                searches[p].best = std::move(best);
                return pair_pricing{searches[p].best->first,
                                    &searches[p].best->second};
            }
            return pair_pricing{margin_p};
        }
        std::pair<std::string, double> stat(
            const std::vector<pair_data> &) const {
            return {"searches", num_searches.load()};
        }
    };

    // The pairs of each case, by target, between the vertices of positive
    // quality or quality gain as a source and as a target, and the C_k: the
    // sum of the values of the pairs no column improves, whose best column,
    // with every option or within the budget for enumerate, is worth no more
    // than their column without.
    template <typename Pricer, instance_c I>
    static std::pair<std::vector<pair_data>, std::vector<double>> _pairs(
        const I & instance, const budget_t & budget,
        const std::optional<instance_solution_t<I>> & start) {
        auto all = [](auto) { return true; };
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
            tbb::parallel_for(std::size_t{0}, targets.size(), [&](auto j) {
                const vertex t = targets[j];
                auto & [constant, t_pairs] = target_pairs[j];
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
                Pricer::target_pairs(instance, instance_case, budget, start, t,
                                     sources, is_source, add);
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
        // the least L + d_i over the complete pricing rounds
        std::vector<double> option_bounds;
    };

    template <typename LP, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        auto [solution, report, option_bounds] =
            this->template relax<LP>(instance, budget);
        return {std::move(solution), std::move(report)};
    }

    // x cast to bool can exceed the budgets and buy two options of a
    // conflict set: the options by decreasing x that fit, or the start if
    // it scores more
    template <instance_c I>
    instance_solution_t<I> _rounded(
        const I & instance, const budget_t & budget, const auto & x_values,
        const std::optional<instance_solution_t<I>> & start) const {
        auto options = std::ranges::to<std::vector>(instance.options());
        std::ranges::stable_sort(options,
                                 [&](const option_t a, const option_t b) {
                                     return x_values[a] > x_values[b];
                                 });
        auto solution = instance.create_option_map(false);
        double purchased = 0.0;  // in the dimension 0
        for(const option_t i : options) {
            const double price = instance.option_cost(i, 0);
            if(purchased + price > budget.front() + feasibility_tol) continue;
            solution[i] = true;
            if(!has_plain_budget(instance) &&
               !is_feasible(instance, solution, budget, feasibility_tol)) {
                solution[i] = false;
                continue;
            }
            purchased += price;
        }
        auto score = [&](const auto & s) {
            return instance.eval_criterion(
                compute_solution_cases_pc_num(instance, s));
        };
        if(start && score(start.value()) > score(solution))
            solution = start.value();

        return solution;
    }

    template <typename LP, instance_c I>
    relaxation<I> relax(const I & instance, const budget_t & budget) const {
        namespace colgen = mippp::colgen;
        using namespace mippp;
        using namespace mippp::operators;
        _check_criterion(instance);
        spdlog::stopwatch sw;

        std::optional<instance_solution_t<I>> start;
        if(mip_start) {
            start = _mip_start_solution(instance);
            if(const auto defect = _start_defect(instance, *start, budget);
               !defect.empty()) {
                spdlog::warn("MIP start {}: ignored", defect);
                start.reset();
            }
        }
        // enumerate's columns fit in budget + feasibility_tol, as the start
        // and the solution returned do, or L can be below their score
        budget_t capacity;
        for(std::size_t r = 0; r < budget.size(); ++r)
            capacity.values.push_back(budget[r] + feasibility_tol);
        const auto [pairs, constants] =
            search_pricing
                ? _pairs<search_pricer<I>>(instance, capacity, start)
                : _pairs<enumerate_pricer>(instance, capacity, start);
        spdlog::debug("Source-target pairs: {}", pairs.size());
        std::optional<search_pricer<I>> search;
        if(search_pricing) search.emplace(instance, pairs);

        // The master: x, the budget and conflict rows, and for each case k,
        // z_k with its contribution row; the rows of a pair come with the first
        // column that uses them.
        LP model;
        if constexpr(has_feasibility_tolerance<LP>)
            model.set_feasibility_tolerance(feasibility_tol);
        _warn_without_numeric_focus<LP>("LP");
        _set_numeric_focus(model);
        model.set_maximization();
        const auto x = model.add_variables(
            instance.options().size(), [](const option_t & i) { return i; });
        std::vector<constraint_t> budget_rows, conflict_rows;
        for(std::size_t r = 0; r < budget.size(); ++r)
            budget_rows.push_back(model.add_constraint(
                xsum(instance.options(), [&](auto && i) {
                    return instance.option_cost(i, r) * x(i);
                }) <= budget[r]));
        for(auto && set : instance.conflict_sets())
            conflict_rows.push_back(model.add_constraint(xsum(set, x) <= 1));
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
        // NaN, left out of the report, while no master is solved: 0 would
        // read as a relaxation of -100 %
        double master_value = std::numeric_limits<double>::quiet_NaN();
        auto x_values = instance.create_option_map(0.0);
        auto log_master = [&] {
            solvers::log_master(instance, "path formulation", master_value,
                                [&](const option_t i) { return x_values[i]; });
        };

        // a column y_pS, the key of the column manager: the index p of its
        // pair and its options S; its value v_pS is the data
        using column = std::pair<std::size_t, option_set>;
        column_manager<LP, column> columns;
        auto add_column = [&](LP &, const auto & entry) {
            const auto & [p, S] = entry.seed;
            if(!uniqueness_rows[p])
                uniqueness_rows[p] = model.add_constraint(
                    empty_linear_expression<variable_t, double> <= 1);
            for(const option_t i : S)
                if(!purchase_rows[p].contains(i))
                    purchase_rows[p].emplace(
                        i, model.add_constraint(
                               empty_linear_expression<variable_t, double> <=
                               x(i)));
            return model.add_column(std::views::concat(
                std::views::single(std::make_pair(
                    contribution_rows[pairs[p].case_id], -entry.data)),
                std::views::single(std::make_pair(*uniqueness_rows[p], 1.0)),
                std::views::transform(S, [&](const option_t i) {
                    return std::make_pair(purchase_rows[p].at(i), 1.0);
                })));
        };
        for(std::size_t p = 0; p < pairs.size(); ++p)
            for(const auto & [S, v] : pairs[p].start_columns)
                columns.emplace_master_column(model, column{p, S}, add_column,
                                              v);

        const auto unattractive = unattractive_columns<LP>(evict_age);

        // Column generation.
        solve_report report{"converged"};
        std::size_t iterations = 1;  // pricing rounds
        std::size_t num_evicted = 0;
        // per pair, max(0, r_p) or a bound on it
        std::vector<double> pricing_bounds(pairs.size());
        // A complete round of bound L lowers the bound of each option to
        // L + d_i, with the purchase duals as the search counts them, a
        // little below 0 as 0
        const auto options_sets = compute_options_conflict_sets(instance);
        std::vector<double> option_bounds(
            instance.options().size(), std::numeric_limits<double>::infinity());
        auto lower_option_bounds = [&](const auto & dual_solution,
                                       const double bound) {
            std::vector<double> reduced_costs(instance.options().size());
            for(auto && i : instance.options()) {
                reduced_costs[i] = -instance.option_cost(i, 0) *
                                   dual_solution[budget_rows.front()];
                for(std::size_t r = 1; r < budget.size(); ++r)
                    reduced_costs[i] -= instance.option_cost(i, r) *
                                        dual_solution[budget_rows[r]];
                for(auto && l : options_sets[i])
                    reduced_costs[i] -= dual_solution[conflict_rows[l]];
            }
            for(const auto & rows : purchase_rows)
                for(const auto & [i, row] : rows)
                    reduced_costs[i] += std::max(0.0, dual_solution[row]);
            for(auto && i : instance.options())
                option_bounds[i] = std::min(
                    option_bounds[i], bound + std::min(0.0, reduced_costs[i]));
        };
        // The end of the rounds: the end of the progress comes before the
        // line of the log, which would otherwise print inside its table
        auto end_rounds = [&](const std::string_view status,
                              const spdlog::level::level_enum level,
                              const std::string_view where = {}) {
            report.status = status;
            report_progress({.end = true});
            spdlog::log(level, "Column generation stopped: {}{}", status,
                        where);
        };
        auto stop = [&] { return stop_requested || _time_up(sw); };
        for(;; ++iterations) {
            _set_time_left(model, sw);
            model.solve();
            if(!is_a<status::optimal>(model.get_status())) {
                end_rounds(stop_requested ? "interrupted"
                           : _time_up(sw) ? "time limit reached"
                                          : "master not solved",
                           spdlog::level::warn,
                           std::format(" in round {}", iterations));
                --iterations;  // a round counts once its master is solved
                break;
            }
            const auto dual_solution = model.get_dual_solution();
            master_value = model.get_solution_value();
            {
                const auto model_solution = model.get_solution();
                for(auto && i : instance.options())
                    x_values[i] = model_solution[x(i)];
            }
            log_master();

            auto duals = [&](const std::size_t p) {
                return pair_duals{
                    dual_solution[contribution_rows[pairs[p].case_id]],
                    uniqueness_rows[p] ? dual_solution[*uniqueness_rows[p]]
                                       : 0.0,
                    purchase_rows[p], dual_solution};
            };

            // the pricing of every pair: its bound and, if it has one, its
            // column
            std::atomic<bool> cut_short = false;  // by a stop
            tbb::concurrent_vector<std::pair<column, double>> priced_columns;
            auto price_pairs = [&](auto && pricer) {
                tbb::parallel_for(
                    std::size_t{0}, pairs.size(), [&](const std::size_t p) {
                        const auto pricing =
                            pricer.price(p, pairs[p], duals(p), stop);
                        if(!pricing) {
                            cut_short = true;
                            return;
                        }
                        pricing_bounds[p] = pricing->bound;
                        if(const valued_set * c = pricing->column)
                            priced_columns.push_back(
                                {column{p, c->first}, c->second});
                    });
            };
            if(search)
                price_pairs(*search);
            else
                price_pairs(enumerate_pricer{});

            // L = master_value + Σ_p max(0, r_p), after a complete round
            if(!cut_short) {
                const double bound =
                    master_value + std::accumulate(pricing_bounds.begin(),
                                                   pricing_bounds.end(), 0.0);
                if(!report.best_bound || bound < report.best_bound.value())
                    report.best_bound = bound;
                lower_option_bounds(dual_solution, bound);
            }
            // after a pricing cut short, with the bound of the rounds before
            report_progress({.bound = report.best_bound,
                             .relaxation = master_value,
                             .round = iterations});
            if(!cut_short && relative_gap > 0 &&
               report.best_bound.value() - master_value <=
                   relative_gap * master_value) {
                end_rounds("relative gap reached", spdlog::level::debug);
                break;
            }

            // the improving columns enter the master, or the max_activations
            // best of them
            columns.emplace_pool_columns(priced_columns |
                                         std::views::as_rvalue);
            columns.update_columns([&](const auto & entry) {
                return colgen::priced{
                    duals(entry.seed.first)
                        .reduced_cost(entry.seed.second, entry.data)};
            });
            columns.purge_pool(
                [](const auto & entry) { return reduced_cost_of(entry) <= 0; });
            auto improving = [&](const auto & entry) {
                return reduced_cost_of(entry) >
                       duals(entry.seed.first).margin();
            };
            const std::size_t num_activated =
                max_activations
                    ? columns.activate_columns(
                          model, improving, add_column,
                          colgen::at_most_k_best(
                              max_activations,
                              [](const auto & a, const auto & b) {
                                  return reduced_cost_of(a) >
                                         reduced_cost_of(b);
                              }))
                    : columns.activate_columns(model, improving, add_column);
            if(!num_activated && !cut_short) {
                end_rounds("converged", spdlog::level::debug);
                break;
            }
            if(stop()) {
                end_rounds(
                    stop_requested ? "interrupted" : "time limit reached",
                    spdlog::level::warn);
                break;
            }
            // only once the master will be solved again, for the same reason
            if constexpr(has_remove_variable<LP>)
                if(evict_age)
                    num_evicted += columns.evict_columns(model, unattractive);
        }

        auto solution = _rounded(instance, budget, x_values, start);
        log_master();

        report.stats = {{"iterations", iterations},
                        {"variables", model.num_variables()},
                        {"constraints", model.num_constraints()},
                        {"pairs", pairs.size()},
                        {"pool_columns", columns.num_pool_columns()},
                        {"evicted_columns", num_evicted}};
        report.score_stats = {{"relaxation_value", master_value}};
        report.stats.push_back(search ? search->stat(pairs)
                                      : enumerate_pricer::stat(pairs));
        return {std::move(solution), std::move(report),
                std::move(option_bounds)};
    }
};

}  // namespace solvers
}  // namespace gecot
