#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <tbb/parallel_for.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/solvers/multiflow_benders.hpp"
#include "gecot/utils/progress.hpp"

namespace gecot {
namespace solvers {

// The relaxation of multiflow_benders, with its rounds of cuts, then a
// randomized rounding of its optimum x*, in docs/formulations/benders.md.
struct multiflow_rounding : public multiflow_benders {
    // the scenarios drawn; 0: until --time-limit, which is then required
    std::size_t roundings = 1000;
    std::size_t random_seed = 0;
    // The scenarios are scored by batches of this many draws, in parallel,
    // and the stops are tested between batches: a constant, so that the
    // draws, and the solution, do not depend on the number of threads.
    static constexpr std::size_t batch_size = 64;

    // Without the lift of multiflow_benders: its cuts pay in a search, which
    // this run has not, and it made the rounds 31 % longer on the benchmark
    // for solutions as good, in docs/formulations/benders.md
    multiflow_rounding() { in_out_lift = 0; }

    // The randomized rounding, after the rounds on the relaxation: the
    // scenarios drawn from x* and the MIP start, the best by the flows of
    // the blocks.
    template <typename M, instance_c I, typename B>
    std::pair<std::optional<instance_solution_t<I>>, solve_report> _round(
        M & model, const spdlog::stopwatch & sw, const I & instance,
        const budget_t & budget, const std::vector<B> & blocks,
        const root_rounds & root) const {
        solve_report report = _model_report(model, {}, sw.elapsed());
        // no round ended: x* is 0
        auto x_star = [&root](const option_t i) {
            return root.optimum.empty() ? 0.0 : root.optimum[i];
        };
        auto by_x_star = std::ranges::to<std::vector>(instance.options());
        std::ranges::stable_sort(by_x_star, std::ranges::greater{}, x_star);
        // the options of first, then the others by decreasing x*, each if it
        // fits, as path_lp rounds
        auto scenario = [&](const std::vector<option_t> & first) {
            auto y = instance.create_option_map(false);
            double purchased = 0.0;  // in the dimension 0
            auto add = [&](const option_t i) {
                const double price = instance.option_cost(i, 0);
                if(y[i] || purchased + price > budget.front() + feasibility_tol)
                    return;
                y[i] = true;
                if(!has_plain_budget(instance) &&
                   !is_feasible(instance, y, budget, feasibility_tol)) {
                    y[i] = false;
                    return;
                }
                purchased += price;
            };
            std::ranges::for_each(first, add);
            std::ranges::for_each(by_x_star, add);
            return y;
        };
        // its score, by the flow of each block, as a candidate of the search
        auto score_of = [&](const auto & y) {
            auto case_scores = instance.create_case_map(0.0);
            for(const B & block : blocks) {
                const double flow = block.flow_at(y);
                double quality = block.target_quality;
                for(const auto & [target_gain, i, big_M, f_ti] :
                    block.target_quality_gains)
                    quality += y[i] * target_gain;
                case_scores[block.case_id] += quality * flow;
            }
            return instance.eval_criterion(case_scores);
        };
        auto key = [&](const auto & y) {
            return std::ranges::to<std::vector<bool>>(std::views::transform(
                instance.options(), [&y](const option_t i) { return y[i]; }));
        };

        std::mt19937_64 rng{random_seed};
        std::uniform_real_distribution<double> uniform;
        std::unordered_set<std::vector<bool>> seen;
        std::vector<instance_solution_t<I>> batch;
        if(mip_start)
            if(auto start = _mip_start_solution(instance);
               _start_defect(instance, start, budget).empty()) {
                seen.insert(key(start));
                batch.push_back(std::move(start));
            }
        std::optional<instance_solution_t<I>> best;
        double best_score = 0;
        std::size_t draws = 0;
        std::string_view status = "completed";
        for(;;) {
            // The draw 0 buys the options by decreasing x*; each other one
            // draws each option i with probability x*_i and tries those drawn
            // in a random order first.
            for(std::size_t k = 0;
                k < batch_size && (roundings == 0 || draws < roundings);
                ++k, ++draws) {
                std::vector<option_t> drawn;
                if(draws > 0) {
                    for(auto && i : instance.options())
                        if(uniform(rng) < x_star(i)) drawn.push_back(i);
                    std::ranges::shuffle(drawn, rng);
                }
                if(auto y = scenario(drawn); seen.insert(key(y)).second)
                    batch.push_back(std::move(y));
            }
            std::vector<double> scores(batch.size());
            tbb::parallel_for(
                std::size_t{0}, batch.size(),
                [&](const std::size_t k) { scores[k] = score_of(batch[k]); });
            for(std::size_t k = 0; k < batch.size(); ++k)
                if(!best || scores[k] > best_score) {
                    best_score = scores[k];
                    best = std::move(batch[k]);
                }
            report_progress({.score = best_score, .bound = root.value});
            const bool new_scenarios = !batch.empty();
            batch.clear();
            if(root.value &&
               *root.value <= best_score + mip_gap * std::abs(best_score)) {
                status = "optimal";
                break;
            }
            // the draws are done, or a batch brought no new scenario
            if((roundings != 0 && draws >= roundings) || !new_scenarios) break;
            if(stop_requested) {
                status = "interrupted";
                break;
            }
            if(_time_up(sw)) {
                status = "time limit reached";
                break;
            }
        }
        report_progress(
            {.score = best_score, .bound = root.value, .end = true});
        report.status = std::string(status);
        report.stats.emplace_back("roundings", draws);
        report.stats.emplace_back("scenarios", seen.size());
        spdlog::debug("Roundings: {}, scenarios: {}", draws, seen.size());
        return {std::move(best), report};
    }

    template <mippp::has_column_generation M, instance_c I>
    solve_result<I> solve(const I & instance, const budget_t & budget) const {
        if(roundings == 0 && !time_limit)
            throw std::invalid_argument("--roundings 0 needs a --time-limit");
        return _with_relaxation<M>(
            instance, budget,
            [&](auto & model, const auto & sw, const auto &, const auto &,
                const auto &, const auto & blocks, const auto &,
                const root_rounds & root) {
                return _round(model, sw, instance, budget, blocks, root);
            });
    }
};

}  // namespace solvers
}  // namespace gecot
