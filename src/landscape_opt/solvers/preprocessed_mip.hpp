#ifndef LANDSCAPE_OPT_SOLVERS_PREPROCESSED_MIP_HPP
#define LANDSCAPE_OPT_SOLVERS_PREPROCESSED_MIP_HPP

#include <type_traits>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "melon/graph.hpp"

#include "concepts/instance.hpp"
#include "helper.hpp"
#include "indices/eca.hpp"
#include "indices/parallel_eca.hpp"
#include "preprocessing/compute_contracted_generalized_flow_graph.hpp"
#include "preprocessing/compute_strong_and_useless_arcs.hpp"
#include "utils/chrono.hpp"

namespace fhamonic {
namespace landscape_opt {
namespace solvers {

struct preprocessed_MIP {
    bool verbose = false;
    bool parallel = false;

    template <concepts::Instance I>
    typename I::Solution solve(const I & instance, const double budget) const {
        using Landscape = typename I::Landscape;
        using OriginalGraph = typename Landscape::Graph;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        Chrono chrono;
        Solution solution = instance.create_solution();
        using namespace mippp;
        using MIP = mip_model<default_solver_traits>;
        MIP model;

        const auto X_vars = model.add_vars(
            instance.options().size(), [](Option i) { return i; },
            {.upper_bound = 1, .type = MIP::var_category::binary});

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](Option i) {
                return instance.option_cost(i);
            }) <= budget);

        const OriginalGraph & original_graph = instance.landscape().graph();
        const auto & original_quality_map = instance.landscape().quality_map();
        const auto & original_vertex_options_map =
            instance.vertex_options_map();

        const auto F_vars = model.add_vars(
            original_graph.nb_vertices(),
            [](const melon::vertex_t<OriginalGraph> v) { return v; });

        model.add_obj(xsum(melon::vertices(original_graph), F_vars,
                           original_quality_map));

        const auto [strong_arcs_map, useless_arcs_map] =
            compute_strong_and_useless_arcs(instance, parallel);

        for(const auto & original_t : melon::vertices(original_graph)) {
            const auto [graph, quality_map, vertex_options_map, arc_no_map,
                        probability_map, arc_option_map, t] =
                compute_contracted_generalized_flow_graph(
                    instance, strong_arcs_map, useless_arcs_map, original_t);
            using Graph = std::decay_t<decltype(graph)>;
            const auto big_M_map =
                compute_big_M_map(graph, quality_map, vertex_options_map,
                                  probability_map, parallel);

            for(const auto & [quality_gain, option] :
                original_vertex_options_map[original_t]) {
                const auto F_prime_t_var = model.add_var();
                model.add_constraint(F_prime_t_var <= F_vars(original_t));
                model.add_constraint(F_prime_t_var <=
                                     big_M_map[t] * X_vars(option));
                model.add_obj(quality_gain * F_prime_t_var);
            }
            const auto Phi_t_vars = model.add_vars(
                graph.nb_arcs(), [&arc_no_map](const melon::arc_t<Graph> & a) {
                    return arc_no_map[a];
                });
            for(const auto & u : melon::vertices(graph)) {
                if(u == t) continue;
                model.add_constraint(
                    xsum(graph.out_arcs(u), Phi_t_vars) <=
                    xsum(graph.in_arcs(u), Phi_t_vars, probability_map) +
                        quality_map[u] +
                        xsum(
                            vertex_options_map[u],
                            [&X_vars](const auto & p) {
                                return X_vars(p.second);
                            },
                            [](const auto & p) { return p.first; }));
            }

            std::cout << "Il y a des arcs sortant de t ? : " << std::ranges::distance(graph.out_arcs(t)) << std::endl;

            model.add_constraint(
                F_vars(original_t) + xsum(graph.out_arcs(t), Phi_t_vars) <=
                xsum(graph.in_arcs(t), Phi_t_vars, probability_map) +
                    quality_map[t] +
                    xsum(
                        vertex_options_map[t],
                        [&X_vars](const auto & p) { return X_vars(p.second); },
                        [](const auto & p) { return p.first; }));

            for(const auto & a : melon::arcs(graph)) {
                if(!arc_option_map[a].has_value()) continue;
                model.add_constraint(
                    Phi_t_vars(a) <=
                    X_vars(arc_option_map[a].value()) *
                        big_M_map[melon::arc_source(graph, a)]);
            }
        }

        if(verbose) std::cout << model << std::endl;

        auto solver = model.build();
        solver.set_loglevel(verbose ? 1 : 0);
        solver.set_timeout(3600);
        solver.optimize();
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

#endif  // LANDSCAPE_OPT_SOLVERS_PREPROCESSED_MIP_HPP