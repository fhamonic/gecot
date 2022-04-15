#ifndef LANDSCAPE_OPT_SOLVERS_MIP_HPP
#define LANDSCAPE_OPT_SOLVERS_MIP_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "concepts/instance.hpp"
#include "helper.hpp"
#include "indices/eca.hpp"
#include "utils/chrono.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct MIP {
    bool verbose = false;
    bool parallel = false;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using Graph = typename I::Landscape::Graph;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        // int time_ms = 0;
        Chrono chrono;
        Solution solution = instance.create_solution();

        const QualityMap & quality_map = instance.landscape().quality_map();
        const ProbabilityMap & probability_map =
            instance.landscape().probability_map();

        std::vector<Graph::vertex> target_vertices;
        std::vector<int> ids(graph.nb_vertices(), -1);
        for(auto && t : graph.vertices()) {
            if(quality_map[t] == 0 && !instance.node_options_map()[t]) continue;
            ids[t] = target_vertices.size();
            target_vertices.emplace_back(t);
        }

        ////////////////////////////////////////////////////////////////////////
        // Columns : Objective
        ////////////////////
        for(auto && t : target_vertices) {
            if(quality_map[t] == 0 && !plan.contains(t)) continue;
            // sum w(t) * f_t
            const int f_t = vars.f.id(t);
            solver_builder.setObjective(f_t, landscape.getQuality(t));
            for(auto const & e : instance.node_options_map()[t]) {
                const int restored_f_t = vars.restored_f.id(e);
                solver_builder.setObjective(restored_f_t, e.quality_gain);
            }
        }
        ////////////////////////////////////////////////////////////////////////
        // Rows : Constraints
        ////////////////////
        for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t) {
            if(landscape.getQuality(t) == 0 && !plan.contains(t)) continue;
            const int f_t = vars.f.id(t);
            // out_flow(u) - in_flow(u) <= w(u)
            for(MutableLandscape::NodeIt u(graph); u != lemon::INVALID; ++u) {
                // out flow
                for(MutableLandscape::Graph::OutArcIt b(graph, u);
                    b != lemon::INVALID; ++b) {
                    const int x_tb = vars.x.id(t, b);
                    solver_builder.buffEntry(x_tb, 1);
                    for(auto const & e : plan[b]) {
                        const int restored_x_t_b = vars.restored_x.id(t, e);
                        solver_builder.buffEntry(restored_x_t_b, 1);
                    }
                }
                // in flow
                for(MutableLandscape::Graph::InArcIt a(graph, u);
                    a != lemon::INVALID; ++a) {
                    const int x_ta = vars.x.id(t, a);
                    solver_builder.buffEntry(x_ta,
                                             -landscape.getProbability(a));
                    for(auto const & e : plan[a]) {
                        const int restored_x_t_a = vars.restored_x.id(t, e);
                        solver_builder.buffEntry(restored_x_t_a,
                                                 -e.restored_probability);
                    }
                }
                // optional injected flow
                for(auto const & e : plan[u]) {
                    const int y_u = vars.y.id(e.option);
                    solver_builder.buffEntry(y_u, -e.quality_gain);
                }
                // optimisation variable
                if(u == t) solver_builder.buffEntry(f_t, 1);
                // injected flow
                solver_builder.pushRow(-OSI_Builder::INFTY,
                                       landscape.getQuality(u));
            }

            // x_a < y_i * M
            for(MutableLandscape::ArcIt a(graph); a != lemon::INVALID; ++a) {
                for(auto const & e : plan[a]) {
                    const int y_i = vars.y.id(e.option);
                    const int x_ta = vars.restored_x.id(t, e);
                    solver_builder.buffEntry(y_i, M[graph.source(a)]);
                    solver_builder.buffEntry(x_ta, -1);
                    solver_builder.pushRow(0, OSI_Builder::INFTY);
                }
            }

            // restored_f_t <= f_t
            // restored_f_t <= y_i * M
            for(const auto & e : plan[t]) {
                const int y_i = vars.y.id(e.option);
                const int f_t = vars.f.id(t);
                const int restored_f_t = vars.restored_f.id(e);
                solver_builder.buffEntry(f_t, 1);
                solver_builder.buffEntry(restored_f_t, -1);
                solver_builder.pushRow(0, OSI_Builder::INFTY);

                solver_builder.buffEntry(y_i, M[t]);
                solver_builder.buffEntry(restored_f_t, -1);
                solver_builder.pushRow(0, OSI_Builder::INFTY);
            }
        }
        ////////////////////
        // sum y_i < B
        for(const RestorationPlan<MutableLandscape>::Option i :
            plan.options()) {
            const int y_i = vars.y.id(i);
            solver_builder.buffEntry(y_i, plan.getCost(i));
        }
        solver_builder.pushRow(-OSI_Builder::INFTY, B);
        ////////////////////
        // integer constraints
        if(!relaxed) {
            for(RestorationPlan<MutableLandscape>::Option i : plan.options()) {
                const int y_i = vars.y.id(i);
                solver_builder.setInteger(y_i);
            }
        }

        // time_ms = chrono.timeMs();
        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_MIP_HPP