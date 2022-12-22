#ifndef LANDSCAPE_OPT_SOLVERS_MIP_HPP
#define LANDSCAPE_OPT_SOLVERS_MIP_HPP

#include <stdexcept>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "melon/concepts/graph.hpp"

#include "concepts/instance.hpp"
#include "helper.hpp"
#include "indices/eca.hpp"
#include "indices/parallel_eca.hpp"
#include "preprocessing/compute_big_M_map.hpp"
#include "preprocessing/compute_generalized_flow_graph.hpp"
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
        using Graph = typename Landscape::Graph;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        // int time_ms = 0;
        Chrono chrono;
        Solution solution = instance.create_solution();

        const auto [graph, quality_map, vertex_options_map, probability_map,
                    arc_option_map] = compute_generalized_flow_graph(instance);

        const auto big_M_map = compute_big_M_map(
            graph, quality_map, vertex_options_map, probability_map, parallel);

        using namespace mippp;
        using MIP = mip_model<default_solver_traits>;
        MIP model;

        const auto F_vars =
            model.add_vars(graph.nb_vertices(),
                           [](const melon::vertex_t<Graph> v) { return v; });
        const auto Phi_vars = model.add_vars(
            graph.nb_vertices() * graph.nb_arcs(),
            [n = graph.nb_vertices()](const melon::vertex_t<Graph> v,
                                      const melon::arc_t<Graph> a) {
                return v * n + a;
            });
        const auto X_vars = model.add_vars(
            instance.options().size(), [](Option i) { return i; },
            {.upper_bound = 1, .type = MIP::var_category::binary});

        model.add_obj(xsum(graph.vertices(), F_vars, quality_map));
        for(const auto & t : graph.vertices()) {
            for(const auto & [quality_gain, option] : vertex_options_map[t]) {
                const auto F_prime_t_var = model.add_var();
                model.add_constraint(F_prime_t_var <= F_vars(t));
                model.add_constraint(F_prime_t_var <=
                                     big_M_map[t] * X_vars(option));
                model.add_obj(quality_gain * F_prime_t_var);
            }
            const auto Phi_t_var = [&Phi_vars, t](const auto & a) {
                return Phi_vars(t, a);
            };
            for(const auto & u : graph.vertices()) {
                if(u == t) continue;
                model.add_constraint(
                    xsum(graph.out_arcs(u), Phi_t_var) <=
                    xsum(graph.in_arcs(u), Phi_t_var, probability_map) +
                        quality_map[u] +
                        xsum(
                            vertex_options_map[u],
                            [&X_vars](const auto & p) {
                                return X_vars(p.second);
                            },
                            [](const auto & p) { return p.first; }));
            }
            model.add_constraint(
                F_vars(t) + xsum(graph.out_arcs(t), Phi_t_var) <=
                xsum(graph.in_arcs(t), Phi_t_var, probability_map) +
                    quality_map[t] +
                    xsum(
                        vertex_options_map[t],
                        [&X_vars](const auto & p) { return X_vars(p.second); },
                        [](const auto & p) { return p.first; }));

            for(const auto & a : graph.arcs()) {
                if(!arc_option_map[a].has_value()) continue;
                model.add_constraint(Phi_t_var(a) <=
                                     X_vars(arc_option_map[a].value()) *
                                         big_M_map[graph.source(a)]);
            }
        }

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](const auto & o) {
                return instance.option_cost(o);
            }) <= budget);

        // std::cout << model << std::endl;

        auto solver = model.build();
        solver.set_loglevel(verbose ? 1 : 0);
        solver.set_timeout(3600);
        auto ret_code = solver.optimize();
        if(ret_code != 0)
            throw std::runtime_error(
                "The thirdparty MIP solver failed with code " +
                std::to_string(ret_code));
        const auto solver_solution = solver.get_solution();

        for(const auto & i : instance.options()) {
            solution[i] = solver_solution[X_vars(i).id()];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_MIP_HPP