#ifndef LANDSCAPE_OPT_SOLVERS_MIP_HPP
#define LANDSCAPE_OPT_SOLVERS_MIP_HPP

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

        auto [graph, quality_map, vertex_options_map, probability_map,
              arc_options_map] = generalized_flow_graph(instance);

        auto big_M_map = compute_big_M_map(graph, quality_map,
                                           vertex_options_map, probability_map);

        using namespace mippp;
        using MIP = mip_model<default_solver_traits>;
        MIP model;

        auto F_vars =
            model.add_vars(graph.nb_vertices(),
                           [](const melon::vertex_t<Graph> v) { return v; });
        auto Phi_vars = model.add_vars(
            graph.nb_vertices() * graph.nb_arcs(),
            [n = graph.nb_vertices()](const melon::vertex_t<Graph> v,
                                      const melon::arc_t<Graph> a) {
                return v * n + a;
            });
        auto X_vars = model.add_vars(
            instance.options().size(), [](Option i) { return i; },
            {.upper_bound = 1, .type = MIP::var_category::binary});

        model.add_obj(xsum(graph.vertices(), F_vars, quality_map));
        for(auto && t : graph.vertices()) {
            auto Phi_t_var = [&Phi_vars, t](const melon::arc_t<Graph> a) {
                return Phi_vars(t, a);
            };
            for(auto && u : graph.vertices()) {
                if(u == t) continue;
                model.add_constraint(
                    xsum(graph.out_arcs(u), Phi_t_var) <=
                    xsum(graph.in_arcs(u), Phi_t_var, probability_map) +
                        quality_map[u] +
                        xsum(
                            vertex_options_map[u],
                            [&X_vars](auto && p) { return X_vars(p.second); },
                            [](auto && p) { return p.first; }));
            }
            model.add_constraint(
                F_vars(t) + xsum(graph.out_arcs(t), Phi_t_var) <=
                xsum(graph.in_arcs(t), Phi_t_var, probability_map) +
                    quality_map[t] +
                    xsum(
                        vertex_options_map[t],
                        [&X_vars](auto && p) { return X_vars(p.second); },
                        [](auto && p) { return p.first; }));

            for(auto && a : graph.arcs()) {
                if(!arc_options_map[a].has_value()) continue;
                model.add_constraint(Phi_t_var(a) <=
                                     X_vars(arc_options_map[a].value()) *
                                         big_M_map[graph.source(a)]);
            }
        }

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](auto && o) {
                return instance.option_cost(o);
            }) <= budget);

        // std::cout << model << std::endl;

        auto solver = model.build();
        solver.set_loglevel(verbose ? 1 : 0);
        solver.set_timeout(3600);
        solver.optimize();
        auto solver_solution = solver.get_solution();

        for(auto && i : instance.options()) {
            solution[i] = solver_solution[X_vars(i).id()];
        }

        return solution;
    }
};

}  // namespace solvers
}  // namespace landscape_opt
}  // namespace fhamonic

#endif  // LANDSCAPE_OPT_SOLVERS_MIP_HPP

// Solution Solvers::PL_ECA_2::solve(
//     const MutableLandscape & landscape,
//     const RestorationPlan<MutableLandscape> & plan, const double B) const {
//     Solution solution(landscape, plan);
//     const int log_level = params.at("log")->getInt();
//     const int timeout = params.at("timeout")->getInt();
//     (void)timeout;  // pas bien
//     const bool relaxed = params.at("relaxed")->getBool();
//     Chrono chrono;
//     OSI_Builder solver_builder;
//     Variables vars(landscape, plan);
//     insert_variables(solver_builder, vars);
//     if(log_level > 0)
//         std::cout << name()
//                   << ": Start filling solver : " <<
//                   solver_builder.getNbVars()
//                   << " variables" << std::endl;
//     fill_solver(solver_builder, landscape, plan, B, vars, relaxed);
//     // OsiGrbSolverInterface * solver =
//     // solver_builder.buildSolver<OsiGrbSolverInterface>(OSI_Builder::MAX);
//     OsiSolverInterface * solver =
//         solver_builder.buildSolver<OsiClpSolverInterface>(OSI_Builder::MAX);
//     if(log_level <= 1) solver->setHintParam(OsiDoReducePrint);
//     if(log_level >= 1) {
//         if(log_level >= 2) {
//             name_variables(solver_builder, landscape, plan, vars);
//             OsiClpSolverInterface * solver_clp =
//                 solver_builder.buildSolver<OsiClpSolverInterface>(
//                     OSI_Builder::MAX);
//             solver_clp->writeLp("pl_eca_2");
//             delete solver_clp;
//             std::cout << name() << ": LP printed to 'pl_eca_2.lp'" <<
//             std::endl;
//         }
//         std::cout << name() << ": Complete filling solver : "
//                   << solver_builder.getNbConstraints() << " constraints in "
//                   << chrono.lapTimeMs() << " ms" << std::endl;
//         std::cout << name() << ": Start solving" << std::endl;
//     }
//     ////////////////////
//     // GRBsetdblparam(GRBgetenv(solver->getLpPtr()), GRB_DBL_PAR_MIPGAP,
//     1e-8);
//     // GRBsetintparam(GRBgetenv(solver->getLpPtr()),
//     GRB_INT_PAR_LOGTOCONSOLE,
//     // (log_level >= 2 ? 1 : 0));
//     GRBsetintparam(GRBgetenv(solver->getLpPtr()),
//     // GRB_DBL_PAR_TIMELIMIT, timeout);
//     solver->branchAndBound();
//     ////////////////////
//     const double * var_solution = solver->getColSolution();
//     if(var_solution == nullptr) {
//         std::cerr << name() << ": Fail" << std::endl;
//         delete solver;
//         assert(false);
//     }
//     for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
//         const int y_i = vars.y.id(i);
//         const double value = var_solution[y_i];
//         solution.set(i, value);
//     }
//     solution.setComputeTimeMs(chrono.timeMs());
//     solution.obj = solver->getObjValue();
//     solution.nb_vars = solver_builder.getNbNonZeroVars();
//     solution.nb_constraints = solver_builder.getNbConstraints();
//     solution.nb_elems = solver->getNumElements();
//     if(log_level >= 1) {
//         std::cout << name()
//                   << ": Complete solving : " << solution.getComputeTimeMs()
//                   << " ms" << std::endl;
//         std::cout << name() << ": ECA from obj : " << std::sqrt(solution.obj)
//                   << std::endl;
//     }
//     delete solver;

//     return solution;
// }
