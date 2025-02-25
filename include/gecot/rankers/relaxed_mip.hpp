#ifndef GECOT_SOLVERS_MIP_HPP
#define GECOT_SOLVERS_MIP_HPP

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/mip_model.hpp"
#include "mippp/operators.hpp"
#include "mippp/xsum.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/helper.hpp"
#include "gecot/indices/pc_num.hpp"
#include "gecot/utils/chronometer.hpp"

namespace fhamonic {
namespace gecot {
namespace solvers {

struct MIP {
    bool parallel = false;

    template <instance_c I>
    instance_options_rank_t<I> rank_options(const I & instance) const {
        using Landscape = typename I::Landscape;
        using Graph = typename I::Landscape::Graph;
        using QualityMap = typename Landscape::QualityMap;
        using ProbabilityMap = typename Landscape::ProbabilityMap;
        using Option = typename I::Option;
        using Solution = typename I::Solution;

        // int time_ms = 0;
        chronometer chrono;
        Solution solution = instance.create_solution();
        const Graph & graph = instance.landscape().graph();
        const QualityMap & quality = instance.landscape().quality_map();
        const ProbabilityMap & probality =
            instance.landscape().probability_map();

        using namespace mippp;
        using Model = Model<GrbTraits>;
        Model model;

        auto F_vars = model.add_variables(graph.num_vertices(),
                                     [](const melon::vertex_t<Graph> v) { return v; });
        auto Phi_vars = model.add_variables(
            graph.num_vertices() * graph.num_arcs(),
            [n = graph.num_vertices()](const melon::vertex_t<Graph> v, const melon::arc_t<Graph> a) {
                return v * n + a;
            });
        auto X_vars = model.add_variables(instance.options().size(),
                                     [](const Option i) { return i; },
                                     {.type = Model::ColType::BINARY});

        model.add_to_objective(xsum(melon::vertices(graph), F_vars, quality));
        for(auto && t : melon::vertices(graph)) {
            auto Phi_t_var = [&Phi_vars, t](melon::arc_t<Graph> a) {
                return Phi_vars(t, a);
            };
            for(auto && u : melon::vertices(graph)) {
                if(u == t) continue;
                model.add_constraint(
                    xsum(graph.out_arcs(u), Phi_t_var) -
                        xsum(graph.in_arcs(u), Phi_t_var, probality) <=
                    quality[u] +
                        xsum(
                            instance.vertex_options_map()[u],
                            [&X_vars](auto && p) { return X_vars(p.second); },
                            [](auto && p) { return p.first; }));
            }
            model.add_constraint(
                xsum(graph.in_arcs(t), Phi_t_var, probality) <=
                quality[t] +
                    xsum(
                        instance.vertex_options_map()[t],
                        [&X_vars](auto && p) { return X_vars(p.second); },
                        [](auto && p) { return p.first; }) -
                    F_vars(t));
        }

        model.add_constraint(
            xsum(instance.options(), X_vars, [&instance](auto && o) {
                return instance.option_cost(o);
            }) <= budget);

        return solution;
    }
};

}  // namespace solvers
}  // namespace gecot
}  // namespace fhamonic

#endif  // GECOT_SOLVERS_MIP_HPP

// Solution Solvers::PL_PC_NUM_2::solve(
//     const MutableLandscape & landscape,
//     const RestorationPlan<MutableLandscape> & plan, const double B) const {
//     Solution solution(landscape, plan);
//     const int log_level = params.at("log")->getInt();
//     const int timeout = params.at("timeout")->getInt();
//     (void)timeout;  // pas bien
//     const bool relaxed = params.at("relaxed")->getBool();
//     chronometer chrono;
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
//             solver_clp->writeLp("pl_pc_num_2");
//             delete solver_clp;
//             std::cout << name() << ": LP printed to 'pl_pc_num_2.lp'" <<
//             std::endl;
//         }
//         std::cout << name() << ": Complete filling solver : "
//                   << solver_builder.getNbConstraints() << " constraints in "
//                   << chrono.lap_time_ms() << " ms" << std::endl;
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
//     solution.setComputeTimeMs(chrono.time_ms());
//     solution.obj = solver->getObjValue();
//     solution.num_vars = solver_builder.getNbNonZeroVars();
//     solution.num_constraints = solver_builder.getNbConstraints();
//     solution.num_elems = solver->getNumElements();
//     if(log_level >= 1) {
//         std::cout << name()
//                   << ": Complete solving : " << solution.getComputeTimeMs()
//                   << " ms" << std::endl;
//     }
//     delete solver;

//     return solution;
// }
