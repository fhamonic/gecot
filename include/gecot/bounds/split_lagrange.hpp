#ifndef GECOT_BOUNDS_SPLIT_LAGRANGE_HPP
#define GECOT_BOUNDS_SPLIT_LAGRANGE_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fmt/core.h>

#include <spdlog/spdlog.h>

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "melon/graph.hpp"

#include "gecot/bounds/bound_report.hpp"
#include "gecot/bounds/criterion_analysis.hpp"
#include "gecot/bounds/lp_knapsack.hpp"
#include "gecot/bounds/qkp_planes.hpp"
#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/utils/log_table.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {

// Lagrangian bound on the vertex/arc split of the weighted case scores:
//   Σ_k w_k pc_k(X) <= V(X) + A(X),
// V the quality gains with every probability at P̄, bounded by
// qkp_planes' u: V(X) <= Σ_{o∈X} u_o; A(X) = Σ_p w_p P_p(X) over the pairs
// p = (k, s, t) of base qualities, w_p = w_k q^s_s q^t_t. For any ν >= 0,
// weak duality on "p uses o" <= x_o gives
//   A(X) <= Σ_p R_p(ν) + Σ_{o∈X} Σ_p ν_po,
//   R_p(ν) = max over the s-t paths π and the options S used on π of
//            w_p P_π(S) - Σ_{o∈S} ν_po,
// so the bound is Σ_p R_p(ν) + max{Σ_o (u_o + Σ_p ν_po) x_o : c·x <= B},
// the knapsack by its LP relaxation, ν improved by projected subgradient
// steps. ν_po is kept at 0 unless o lies on a pair's corridor (a valid
// choice). R_p is exact by a label search that charges ν_po once per run of
// consecutive arcs of o, which check_structure makes once per path.
struct split_lagrange {
    std::size_t iterations = 30;
    std::optional<double> time_limit;  // seconds, for each weighted bound
    // the score the steps aim at, the best known solution's for instance
    std::optional<double> target;
    qkp_planes vertex_side;

    // On each case, the improved arcs (by an option of cost at most budget)
    // must satisfy:
    //  R1: an arc is improved by at most one option;
    //  R2: the improved arcs of an option share a vertex, so a simple path
    //      uses them consecutively and pays the option once;
    //  R3: the improved arcs entering a vertex are of one option, so the
    //      labels that reach it by an improved arc share their last option.
    // Throws std::invalid_argument naming the first offender. With weights,
    // only the cases of positive weight are checked: the others are not
    // priced.
    template <instance_c I>
    static void check_structure(const I & instance, const double budget,
                                std::span<const double> case_weights = {}) {
        constexpr option_t no_option = std::numeric_limits<option_t>::max();
        for(auto && instance_case : instance.cases()) {
            if(!case_weights.empty() && case_weights[instance_case.id()] <= 0)
                continue;
            const auto & graph = instance_case.graph();
            const auto & probability_map = instance_case.arc_probability_map();
            const auto & arc_options_map = instance_case.arc_options_map();
            auto entering_option =
                melon::create_vertex_map<option_t>(graph, no_option);
            std::vector<std::vector<melon::arc_t<instance_graph_t<I>>>>
                option_arcs(std::ranges::size(instance.options()));
            for(auto && a : melon::arcs(graph)) {
                option_t arc_option = no_option;
                for(auto && [improved_probability, option] :
                    arc_options_map[a]) {
                    if(!(improved_probability > probability_map[a]) ||
                       instance.option_cost(option) > budget)
                        continue;
                    if(arc_option != no_option && arc_option != option)
                        throw std::invalid_argument(fmt::format(
                            "split_lagrange: arc '{}' of '{}' is improved by "
                            "options '{}' and '{}'",
                            instance_case.arc_name(a), instance_case.name(),
                            instance.option_name(arc_option),
                            instance.option_name(option)));
                    arc_option = option;
                }
                if(arc_option == no_option) continue;
                option_arcs[arc_option].push_back(a);
                const auto t = melon::arc_target(graph, a);
                if(entering_option[t] != no_option &&
                   entering_option[t] != arc_option)
                    throw std::invalid_argument(fmt::format(
                        "split_lagrange: vertex '{}' of '{}' is entered by "
                        "arcs improved by options '{}' and '{}'",
                        instance_case.vertex_name(t), instance_case.name(),
                        instance.option_name(entering_option[t]),
                        instance.option_name(arc_option)));
                entering_option[t] = arc_option;
            }
            for(auto && option : instance.options())
                if(!option_arcs[option].empty() &&
                   !common_vertex(graph, option_arcs[option]))
                    throw std::invalid_argument(fmt::format(
                        "split_lagrange: the arcs of '{}' improved by option "
                        "'{}' share no vertex",
                        instance_case.name(), instance.option_name(option)));
        }
    }

    template <typename GR, typename ARCS>
    static std::optional<melon::vertex_t<GR>> common_vertex(const GR & graph,
                                                            const ARCS & arcs) {
        const auto first = arcs.front();
        for(auto v : {melon::arc_source(graph, first),
                      melon::arc_target(graph, first)}) {
            if(std::ranges::all_of(arcs, [&](auto && a) {
                   return melon::arc_source(graph, a) == v ||
                          melon::arc_target(graph, a) == v;
               }))
                return v;
        }
        return std::nullopt;
    }

private:
    static constexpr double infinity = std::numeric_limits<double>::infinity();
    static constexpr std::uint32_t none =
        std::numeric_limits<std::uint32_t>::max();

    struct case_arc {
        std::uint32_t to;
        std::uint32_t option;            // none if not improvable
        double length, improved_length;  // -ln of the probabilities
    };
    struct case_data {
        std::vector<case_arc> arcs;
        std::vector<std::size_t> out_begin, in_begin;  // CSR, by vertex
        std::vector<std::uint32_t> out_arcs, in_arcs;  // arc indices
        std::vector<std::uint32_t> in_sources;         // of in_arcs
        std::vector<std::uint32_t> option_vertex;      // none without arcs
        std::vector<std::uint32_t> sources, targets;
        std::vector<double> source_qualities, target_qualities;
        // per source index or target index, over the vertices
        std::vector<std::vector<double>> base, improved, improved_to_target;
        double weight;
    };
    struct pair_t {
        std::uint32_t case_index, source, target;  // indices in case_data
    };
    struct label {
        double log_probability, charge;
        std::uint32_t vertex, last, parent, option;
    };
    struct scratch {
        std::vector<double> nu, min_charge_free, min_charge_paid;
        std::vector<std::uint32_t> touched, used;
        std::vector<label> labels;
        scratch(std::size_t num_vertices, std::size_t num_options)
            : nu(num_options, 0.0)
            , min_charge_free(num_vertices, infinity)
            , min_charge_paid(num_vertices, infinity) {}
    };
    struct sweep_totals {
        double sum_R = 0.0, norm = 0.0;
        std::size_t labels = 0, used = 0, outside = 0;
        sweep_totals & operator+=(const sweep_totals & o) {
            sum_R += o.sum_R;
            norm += o.norm;
            labels += o.labels;
            used += o.used;
            outside += o.outside;
            return *this;
        }
    };

    // -ln probabilities from v, forward or along reversed arcs
    static std::vector<double> distances(const case_data & c,
                                         std::uint32_t source, bool improved,
                                         bool reversed) {
        const std::size_t n = c.out_begin.size() - 1;
        std::vector<double> d(n, infinity);
        using entry = std::pair<double, std::uint32_t>;
        std::priority_queue<entry, std::vector<entry>, std::greater<entry>>
            heap;
        d[source] = 0.0;
        heap.push({0.0, source});
        while(!heap.empty()) {
            const auto [dv, v] = heap.top();
            heap.pop();
            if(dv > d[v]) continue;
            const auto & begin = reversed ? c.in_begin : c.out_begin;
            for(std::size_t i = begin[v]; i < begin[v + 1]; ++i) {
                const std::uint32_t a = reversed ? c.in_arcs[i] : c.out_arcs[i];
                const case_arc & arc = c.arcs[a];
                const double length =
                    improved ? arc.improved_length : arc.length;
                if(!std::isfinite(length)) continue;
                const std::uint32_t w = reversed ? c.in_sources[i] : arc.to;
                if(dv + length < d[w]) {
                    d[w] = dv + length;
                    heap.push({d[w], w});
                }
            }
        }
        return d;
    }

    // R_p, the options of a maximiser in sc.used and the labels created
    static double price(const case_data & c, const pair_t & p,
                        std::span<const std::uint32_t> corridor,
                        std::span<const double> corridor_nu, scratch & sc,
                        std::size_t & num_labels) {
        const std::uint32_t s = c.sources[p.source], t = c.targets[p.target];
        const double w = c.weight * c.source_qualities[p.source] *
                         c.target_qualities[p.target];
        const auto & to_target = c.improved_to_target[p.target];
        for(std::size_t k = 0; k < corridor.size(); ++k)
            sc.nu[corridor[k]] = corridor_nu[k];
        double best = w * std::exp(-c.base[p.source][t]);
        std::uint32_t best_label = none;
        auto & L = sc.labels;
        L.clear();
        // the highest probability first, then the lowest charge
        using entry = std::tuple<double, double, std::uint32_t>;
        std::priority_queue<entry> heap;
        L.push_back({0.0, 0.0, s, none, none, none});
        heap.push({0.0, -0.0, 0});
        while(!heap.empty()) {
            const std::uint32_t li = std::get<2>(heap.top());
            heap.pop();
            const label lab = L[li];
            const std::uint32_t v = lab.vertex;
            const bool paid = lab.last != none;
            double & min_charge =
                paid ? sc.min_charge_paid[v] : sc.min_charge_free[v];
            if(lab.charge >= min_charge ||
               (!paid && lab.charge >= sc.min_charge_paid[v]))
                continue;
            if(sc.min_charge_free[v] == infinity &&
               sc.min_charge_paid[v] == infinity)
                sc.touched.push_back(v);
            min_charge = lab.charge;
            if(v == t) {
                const double value =
                    w * std::exp(lab.log_probability) - lab.charge;
                if(value > best) {
                    best = value;
                    best_label = li;
                }
                continue;
            }
            for(std::size_t i = c.out_begin[v]; i < c.out_begin[v + 1]; ++i) {
                const case_arc & arc = c.arcs[c.out_arcs[i]];
                const double rest = -to_target[arc.to];
                if(!std::isfinite(rest)) continue;
                for(int improved = 0; improved < 2; ++improved) {
                    if(improved && (arc.option == none ||
                                    !(arc.improved_length < arc.length)))
                        continue;
                    const double length =
                        improved ? arc.improved_length : arc.length;
                    if(!std::isfinite(length)) continue;
                    const double charge =
                        lab.charge + (improved && arc.option != lab.last
                                          ? sc.nu[arc.option]
                                          : 0.0);
                    const double log_probability = lab.log_probability - length;
                    if(w * std::exp(log_probability + rest) - charge <= best)
                        continue;
                    const std::uint32_t last = improved ? arc.option : none;
                    const double min_charge_to =
                        last != none ? sc.min_charge_paid[arc.to]
                                     : std::min(sc.min_charge_free[arc.to],
                                                sc.min_charge_paid[arc.to]);
                    if(charge >= min_charge_to) continue;
                    L.push_back({log_probability, charge, arc.to, last, li,
                                 improved ? arc.option : none});
                    heap.push({log_probability, -charge,
                               static_cast<std::uint32_t>(L.size() - 1)});
                }
            }
        }
        num_labels += L.size();
        sc.used.clear();
        for(std::uint32_t li = best_label; li != none; li = L[li].parent)
            if(L[li].option != none &&
               (sc.used.empty() || sc.used.back() != L[li].option))
                sc.used.push_back(L[li].option);
        for(const std::uint32_t v : sc.touched)
            sc.min_charge_free[v] = sc.min_charge_paid[v] = infinity;
        sc.touched.clear();
        for(const std::uint32_t o : corridor) sc.nu[o] = 0.0;
        return best;
    }

    template <instance_c I>
    static double weighted_score(const I & instance, const auto & solution,
                                 std::span<const double> case_weights) {
        const auto cases_pc_num =
            compute_solution_cases_pc_num(instance, solution);
        double score = 0.0;
        for(auto && instance_case : instance.cases())
            score += case_weights[instance_case.id()] *
                     cases_pc_num[instance_case.id()];
        return score;
    }

public:
    struct weighted_result {
        double bound;
        std::size_t iterations = 0;
        bool timed_out = false;
        std::vector<double> sweep_bounds =
            {};  // of the complete steps, in order
    };

    template <instance_c I>
    weighted_result weighted(
        const I & instance, const double budget,
        std::span<const double> case_weights,
        std::optional<double> weighted_target = std::nullopt) const {
        check_structure(instance, budget, case_weights);
        using clock = std::chrono::steady_clock;
        const auto start_time = clock::now();
        auto elapsed = [&] {
            return std::chrono::duration<double>(clock::now() - start_time)
                .count();
        };
        const std::size_t num_options = std::ranges::size(instance.options());
        std::vector<double> costs(num_options);
        for(auto && o : instance.options()) costs[o] = instance.option_cost(o);

        const auto vertex_bounds =
            vertex_side.weighted(instance, budget, case_weights);
        weighted_result result{vertex_bounds.bound};

        std::vector<case_data> cases;
        for(auto && instance_case : instance.cases()) {
            const double w = case_weights[instance_case.id()];
            if(w <= 0) continue;
            cases.push_back(make_case_data(instance, instance_case, budget, w));
        }
        std::size_t max_vertices = 0;
        std::vector<pair_t> pairs;
        double constant_R = 0.0;  // of the pairs s = t
        for(std::uint32_t k = 0; k < cases.size(); ++k) {
            const case_data & c = cases[k];
            max_vertices = std::max(max_vertices, c.out_begin.size() - 1);
            for(std::uint32_t i = 0; i < c.sources.size(); ++i)
                for(std::uint32_t j = 0; j < c.targets.size(); ++j) {
                    if(c.sources[i] == c.targets[j]) {
                        constant_R += c.weight * c.source_qualities[i] *
                                      c.target_qualities[j];
                        continue;
                    }
                    if(!std::isfinite(c.improved[i][c.targets[j]])) continue;
                    pairs.push_back({k, i, j});
                }
        }
        const std::size_t num_pairs = pairs.size();

        // corridors: o with P̄_{s v_o} P̄_{v_o t} >= P_st, rounded outwards
        auto in_corridor = [&](const pair_t & p, std::uint32_t o) {
            const case_data & c = cases[p.case_index];
            const std::uint32_t v = c.option_vertex[o];
            if(v == none) return false;
            return c.improved[p.source][v] +
                       c.improved_to_target[p.target][v] <=
                   c.base[p.source][c.targets[p.target]] + 1e-9;
        };
        std::vector<std::size_t> offsets(num_pairs + 1, 0);
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, num_pairs),
                          [&](const tbb::blocked_range<std::size_t> & r) {
                              for(std::size_t p = r.begin(); p != r.end(); ++p)
                                  for(std::uint32_t o = 0; o < num_options; ++o)
                                      offsets[p + 1] +=
                                          in_corridor(pairs[p], o);
                          });
        for(std::size_t p = 0; p < num_pairs; ++p) offsets[p + 1] += offsets[p];
        const std::size_t num_entries = offsets[num_pairs];
        spdlog::debug(
            "split_lagrange: {} pairs, {} corridor entries ({:.1f} per pair), "
            "{:.0f} MB of prices",
            num_pairs, num_entries,
            num_pairs ? static_cast<double>(num_entries) /
                            static_cast<double>(num_pairs)
                      : 0.0,
            static_cast<double>(num_entries) * (sizeof(double) + 5) / 1e6);
        std::vector<std::uint32_t> corridors(num_entries);
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, num_pairs),
                          [&](const tbb::blocked_range<std::size_t> & r) {
                              for(std::size_t p = r.begin(); p != r.end();
                                  ++p) {
                                  std::size_t k = offsets[p];
                                  for(std::uint32_t o = 0; o < num_options; ++o)
                                      if(in_corridor(pairs[p], o))
                                          corridors[k++] = o;
                              }
                          });
        std::vector<double> nu(num_entries, 0.0);
        std::vector<unsigned char> z(num_entries, 0);

        tbb::enumerable_thread_specific<scratch> scratches(
            [&] { return scratch(max_vertices, num_options); });
        std::optional<double> aim = weighted_target;
        double best = vertex_bounds.bound, step = 1.0;
        std::size_t stall = 0;
        std::vector<double> values(num_options), x(num_options);
        // bound: of the iteration, the sum of the pairs' R and of the
        // knapsack; labels: of the pairs' searches
        log_table<spdlog::level::debug, 20, 9, 10, 10, 10, 8, 7> sweeps(
            {"iteration", log_format::fixed(0)},
            {"bound", log_format::general(9)}, {"best", log_format::general(9)},
            {"knapsack", log_format::general(9)},
            {"labels", log_format::scientific(2)},
            {"options per pair", log_format::fixed(1)});
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            values = vertex_bounds.option_bounds;
            for(std::size_t e = 0; e < num_entries; ++e)
                values[corridors[e]] += nu[e];
            const double outer = lp_knapsack(values, costs, budget, x);
            if(!aim) {
                // a solution within the budget: below the bound's minimum
                auto solution = instance.template create_option_map<bool>();
                for(auto && o : instance.options()) solution[o] = (x[o] == 1.0);
                aim = weighted_score(instance, solution, case_weights);
                spdlog::trace("split_lagrange: steps aim at {}", aim.value());
            }
            std::atomic<bool> timed_out = false;
            const sweep_totals totals = tbb::parallel_deterministic_reduce(
                tbb::blocked_range<std::size_t>(0, num_pairs, 64),
                sweep_totals{},
                [&](const tbb::blocked_range<std::size_t> & r,
                    sweep_totals acc) {
                    scratch & sc = scratches.local();
                    for(std::size_t p = r.begin(); p != r.end(); ++p) {
                        if(timed_out.load(std::memory_order_relaxed)) break;
                        if(stop_requested ||
                           (time_limit && elapsed() > time_limit.value())) {
                            timed_out = true;
                            break;
                        }
                        const std::span<const std::uint32_t> corridor(
                            corridors.data() + offsets[p],
                            offsets[p + 1] - offsets[p]);
                        acc.sum_R += price(
                            cases[pairs[p].case_index], pairs[p], corridor,
                            std::span<const double>(nu.data() + offsets[p],
                                                    corridor.size()),
                            sc, acc.labels);
                        acc.used += sc.used.size();
                        std::fill(
                            z.begin() + static_cast<std::ptrdiff_t>(offsets[p]),
                            z.begin() +
                                static_cast<std::ptrdiff_t>(offsets[p + 1]),
                            0);
                        for(const std::uint32_t o : sc.used) {
                            const auto it =
                                std::ranges::lower_bound(corridor, o);
                            if(it != corridor.end() && *it == o)
                                z[offsets[p] + static_cast<std::size_t>(
                                                   it - corridor.begin())] = 1;
                            else
                                ++acc.outside;
                        }
                        for(std::size_t e = offsets[p]; e < offsets[p + 1];
                            ++e) {
                            const double g = x[corridors[e]] - z[e];
                            acc.norm += g * g;
                        }
                    }
                    return acc;
                },
                [](sweep_totals a, const sweep_totals & b) {
                    a += b;
                    return a;
                });
            if(timed_out) {
                sweeps.end();
                spdlog::debug(
                    "split_lagrange: stopped in iteration {}, its sweep "
                    "discarded",
                    iteration);
                result.timed_out = true;
                break;
            }
            const double bound = constant_R + totals.sum_R + outer;
            ++result.iterations;
            result.sweep_bounds.push_back(bound);
            if(bound < best - 1e-9 * std::abs(best)) {
                best = bound;
                stall = 0;
            } else if(++stall >= 3) {
                step /= 2;
                stall = 0;
            }
            sweeps.row(iteration, bound, best, outer, totals.labels,
                       num_pairs ? static_cast<double>(totals.used) /
                                       static_cast<double>(num_pairs)
                                 : 0.0);
            const double t = step * (bound - aim.value()) / totals.norm;
            if(totals.norm == 0 || !std::isfinite(t)) break;
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, num_entries),
                [&](const tbb::blocked_range<std::size_t> & r) {
                    for(std::size_t e = r.begin(); e != r.end(); ++e)
                        nu[e] =
                            std::max(0.0, nu[e] - t * (x[corridors[e]] - z[e]));
                });
        }
        result.bound = best;
        return result;
    }

    // R_p of the pair (s, t) of a case, of weight q^s_s q^t_t, under the
    // prices nu of the options, by the label search: for the tests
    template <instance_c I, case_c C>
    static double pair_value(const I & instance, const C & instance_case,
                             const double budget, const std::uint32_t s,
                             const std::uint32_t t,
                             std::span<const double> nu) {
        const case_data c =
            make_case_data(instance, instance_case, budget, 1.0);
        const auto i = std::ranges::find(c.sources, s) - c.sources.begin();
        const auto j = std::ranges::find(c.targets, t) - c.targets.begin();
        if(static_cast<std::size_t>(i) == c.sources.size() ||
           static_cast<std::size_t>(j) == c.targets.size())
            throw std::invalid_argument("pair_value: s or t is no habitat");
        std::vector<std::uint32_t> corridor(nu.size());
        for(std::uint32_t o = 0; o < corridor.size(); ++o) corridor[o] = o;
        scratch sc(c.out_begin.size() - 1, nu.size());
        std::size_t num_labels = 0;
        return price(
            c,
            {0, static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)},
            corridor, nu, sc, num_labels);
    }

    template <instance_c I>
    bound_report bound(const I & instance, const double budget) const {
        std::optional<double> weighted_target;
        bool linear = false;
        if(const auto form = linear_form(instance.criterion(),
                                         std::ranges::size(instance.cases()));
           form && std::ranges::all_of(form->weights,
                                       [](double w) { return w >= 0.0; })) {
            linear = true;
            if(target) weighted_target = target.value() - form->constant;
        } else {
            spdlog::info(
                "split_lagrange: the criterion is not linear, one bound per "
                "case it reads{}",
                target ? ", --target ignored" : "");
        }
        std::size_t total_iterations = 0;
        bool timed_out = false;
        const double b = bound_criterion(
            instance, "split_lagrange", [&](std::span<const double> w) {
                const auto r =
                    weighted(instance, budget, w,
                             linear ? weighted_target : std::nullopt);
                total_iterations += r.iterations;
                timed_out |= r.timed_out;
                return r.bound;
            });
        return {b,
                timed_out
                    ? (stop_requested ? "interrupted" : "time limit reached")
                    : "completed",
                {{"iterations", static_cast<double>(total_iterations)}}};
    }

private:
    template <instance_c I, case_c C>
    static case_data make_case_data(const I & instance, const C & instance_case,
                                    const double budget, const double weight) {
        const auto & graph = instance_case.graph();
        const auto & probability_map = instance_case.arc_probability_map();
        const auto & arc_options_map = instance_case.arc_options_map();
        const std::size_t n = graph.num_vertices();
        const std::size_t num_options = std::ranges::size(instance.options());
        case_data c;
        c.weight = weight;
        c.arcs.resize(graph.num_arcs());
        std::vector<std::vector<melon::arc_t<instance_graph_t<I>>>> option_arcs(
            num_options);
        auto ln = [](double p) { return p > 0 ? -std::log(p) : infinity; };
        for(auto && a : melon::arcs(graph)) {
            std::uint32_t option = none;
            double improved = probability_map[a];
            for(auto && [improved_probability, o] : arc_options_map[a])
                if(improved_probability > probability_map[a] &&
                   instance.option_cost(o) <= budget) {
                    option = o;
                    improved = std::max(improved, improved_probability);
                }
            if(option != none) option_arcs[option].push_back(a);
            c.arcs[a] = {
                static_cast<std::uint32_t>(melon::arc_target(graph, a)), option,
                ln(probability_map[a]), ln(improved)};
        }
        c.option_vertex.assign(num_options, none);
        for(std::uint32_t o = 0; o < num_options; ++o)
            if(!option_arcs[o].empty())
                c.option_vertex[o] = static_cast<std::uint32_t>(
                    common_vertex(graph, option_arcs[o]).value());
        c.out_begin.assign(n + 1, 0);
        c.in_begin.assign(n + 1, 0);
        for(auto && a : melon::arcs(graph)) {
            ++c.out_begin[melon::arc_source(graph, a) + 1];
            ++c.in_begin[melon::arc_target(graph, a) + 1];
        }
        for(std::size_t v = 0; v < n; ++v) {
            c.out_begin[v + 1] += c.out_begin[v];
            c.in_begin[v + 1] += c.in_begin[v];
        }
        c.out_arcs.resize(graph.num_arcs());
        c.in_arcs.resize(graph.num_arcs());
        c.in_sources.resize(graph.num_arcs());
        {
            std::vector<std::size_t> out_next(c.out_begin.begin(),
                                              c.out_begin.end() - 1),
                in_next(c.in_begin.begin(), c.in_begin.end() - 1);
            for(auto && a : melon::arcs(graph)) {
                const auto u = melon::arc_source(graph, a);
                const auto v = melon::arc_target(graph, a);
                c.out_arcs[out_next[u]++] = static_cast<std::uint32_t>(a);
                c.in_sources[in_next[v]] = static_cast<std::uint32_t>(u);
                c.in_arcs[in_next[v]++] = static_cast<std::uint32_t>(a);
            }
        }
        const auto & source_quality_map = instance_case.source_quality_map();
        const auto & target_quality_map = instance_case.target_quality_map();
        for(auto && v : melon::vertices(graph)) {
            if(source_quality_map[v] > 0) {
                c.sources.push_back(static_cast<std::uint32_t>(v));
                c.source_qualities.push_back(source_quality_map[v]);
            }
            if(target_quality_map[v] > 0) {
                c.targets.push_back(static_cast<std::uint32_t>(v));
                c.target_qualities.push_back(target_quality_map[v]);
            }
        }
        c.base.resize(c.sources.size());
        c.improved.resize(c.sources.size());
        c.improved_to_target.resize(c.targets.size());
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, c.sources.size()),
            [&](const tbb::blocked_range<std::size_t> & r) {
                for(std::size_t i = r.begin(); i != r.end(); ++i) {
                    c.base[i] = distances(c, c.sources[i], false, false);
                    c.improved[i] = distances(c, c.sources[i], true, false);
                }
            });
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, c.targets.size()),
                          [&](const tbb::blocked_range<std::size_t> & r) {
                              for(std::size_t j = r.begin(); j != r.end(); ++j)
                                  c.improved_to_target[j] =
                                      distances(c, c.targets[j], true, true);
                          });
        return c;
    }
};

}  // namespace gecot

#endif  // GECOT_BOUNDS_SPLIT_LAGRANGE_HPP
