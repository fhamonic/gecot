#ifndef LANDSCAPE_OPT_SOLVERS_MIP_HPP
#define LANDSCAPE_OPT_SOLVERS_MIP_HPP

#include <stdexcept>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "melon/graph.hpp"

#include "landscape_opt/concepts/instance.hpp"
#include "landscape_opt/helper.hpp"
#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/indices/parallel_eca.hpp"
#include "landscape_opt/preprocessing/compute_big_M_map.hpp"
#include "landscape_opt/preprocessing/compute_generalized_flow_graph.hpp"
#include "landscape_opt/utils/chronometer.hpp"

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
        chronometer chrono;
        Solution solution = instance.create_solution();

        const auto [graph, quality_map, vertex_options_map, probability_map,
                    arc_option_map] = compute_generalized_flow_graph(instance);

        const auto big_M_map = compute_big_M_map(
            graph, quality_map, vertex_options_map, probability_map, parallel);

        using namespace mippp;
        using mip = mip_model<default_solver_traits>;
        mip model;

        const auto F_vars =
            model.add_vars(graph.nb_vertices(),
                           [](const melon::vertex_t<Graph> & v) { return v; });
        const auto X_vars = model.add_vars(
            instance.options().size(), [](const Option i) { return i; },
            {.upper_bound = 1, .type = mip::var_category::binary});

        model.add_obj(xsum(melon::vertices(graph), F_vars, quality_map));
        for(const auto & t : melon::vertices(graph)) {
            for(const auto & [quality_gain, option] : vertex_options_map[t]) {
                const auto F_prime_t_var = model.add_var();
                model.add_constraint(F_prime_t_var <= F_vars(t));
                model.add_constraint(F_prime_t_var <=
                                     big_M_map[t] * X_vars(option));
                model.add_obj(quality_gain * F_prime_t_var);
            }
            const auto Phi_t_vars =
                model.add_vars(graph.nb_arcs(),
                               [](const melon::arc_t<Graph> & a) { return a; });
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
            model.add_constraint(
                F_vars(t) + xsum(graph.out_arcs(t), Phi_t_vars) <=
                xsum(graph.in_arcs(t), Phi_t_vars, probability_map) +
                    quality_map[t] +
                    xsum(
                        vertex_options_map[t],
                        [&X_vars](const auto & p) { return X_vars(p.second); },
                        [](const auto & p) { return p.first; }));

            for(const auto & a : melon::arcs(graph)) {
                if(!arc_option_map[a].has_value()) continue;
                model.add_constraint(Phi_t_vars(a) <=
                                     big_M_map[melon::arc_source(graph, a)] *
                                         X_vars(arc_option_map[a].value()));
            }
        }

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](const auto & o) {
                return instance.option_cost(o);
            }) <= budget);

        if(verbose) std::cout << model << std::endl;

        auto solver = model.build();
        solver.set_loglevel(verbose ? 1 : 0);
        solver.set_timeout(3600);
        solver.set_mip_gap(1e-8);
        auto ret_code = solver.optimize();
        if(ret_code != 0)
            throw std::runtime_error(
                "The thirdparty MIP solver failed with code " +
                std::to_string(ret_code));
        const auto solver_solution = solver.get_solution();

        for(const auto & i : instance.options()) {
            solution[i] = solver_solution[static_cast<std::size_t>(X_vars(i).id())];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_MIP_HPP