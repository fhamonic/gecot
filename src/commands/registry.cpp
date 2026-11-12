#include "commands/common.hpp"

#include <algorithm>
#include <utility>

#include "bound_interfaces/algorithm_bound_interface.hpp"
#include "bound_interfaces/all_options_interface.hpp"
#include "bound_interfaces/qkp_planes_interface.hpp"
#include "bound_interfaces/split_lagrange_interface.hpp"

#include "solver_interfaces/flow_benders_interface.hpp"
#include "solver_interfaces/greedy_decremental_interface.hpp"
#include "solver_interfaces/greedy_incremental_interface.hpp"
#include "solver_interfaces/mip_interface.hpp"
#include "solver_interfaces/multicut_benders_interface.hpp"
#include "solver_interfaces/multiflow_benders_interface.hpp"
#include "solver_interfaces/path_formulation_lp_interface.hpp"
#include "solver_interfaces/preprocessed_mip_interface.hpp"
#include "solver_interfaces/static_decremental_interface.hpp"
#include "solver_interfaces/static_incremental_interface.hpp"
#include "solver_interfaces/tree_formulation_lp_interface.hpp"
#include "solver_interfaces/worst_greedy_decremental_interface.hpp"
#include "solver_interfaces/worst_greedy_incremental_interface.hpp"

std::vector<std::shared_ptr<AbstractSolverInterface>> make_solver_interfaces() {
    return {std::make_unique<StaticIncrementalInterface>(),
            std::make_unique<StaticDecrementalInterface>(),
            std::make_unique<GreedyIncrementalInterface>(),
            std::make_unique<GreedyDecrementalInterface>(),
            std::make_unique<mipInterface>(),
            std::make_unique<PreprocessedmipInterface>(),
            std::make_unique<MulticutBendersInterface>(),
            std::make_unique<FlowBendersInterface>(),
            std::make_unique<MultiflowBendersInterface>(),
            std::make_unique<TreeFormulationLPInterface>(),
            std::make_unique<PathFormulationLPInterface>(),
            std::make_unique<WorstGreedyIncrementalInterface>(),
            std::make_unique<WorstGreedyDecrementalInterface>()};
}

std::vector<std::shared_ptr<AbstractBoundInterface>> make_bound_interfaces() {
    std::vector<std::shared_ptr<AbstractBoundInterface>> methods{
        std::make_unique<AllOptionsInterface>(),
        std::make_unique<QkpPlanesInterface>(),
        std::make_unique<SplitLagrangeInterface>()};
    // the algorithms whose relaxation proves a bound; tree_lp's pricing can
    // miss improving trees, so its relaxation value bounds nothing
    const std::vector<std::pair<std::string, std::string>> proving_algorithms{
        {"mip",
         "Dual bound of the MIP solver, the optimum unless --time-limit stops "
         "it; needs an incumbent."},
        {"prep_mip",
         "Dual bound of the preprocessed MIP, exact up to --resolution; needs "
         "an incumbent."},
        {"multicut_benders",
         "Dual bound of the multicut Benders master; needs an incumbent."},
        {"flow_benders",
         "Dual bound of the flow Benders master; needs an incumbent."},
        {"multiflow_benders",
         "Dual bound of the multiflow Benders master; needs an incumbent."},
        {"path_lp",
         "Lagrangian bound of the path formulation relaxation, the lowest "
         "over its complete pricing rounds."}};
    for(auto & solver : make_solver_interfaces()) {
        const auto it =
            std::ranges::find(proving_algorithms, solver->name(),
                              &std::pair<std::string, std::string>::first);
        if(it == proving_algorithms.end()) continue;
        methods.push_back(std::make_shared<AlgorithmBoundInterface>(
            std::move(solver), it->second));
    }
    return methods;
}
