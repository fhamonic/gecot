#include "commands/common.hpp"

#include <algorithm>
#include <utility>

#include "bound_interfaces/algorithm_bound_interface.hpp"
#include "bound_interfaces/all_options_interface.hpp"
#include "bound_interfaces/qkp_planes_interface.hpp"
#include "bound_interfaces/split_lagrange_interface.hpp"

#include "solver_interfaces/flow_benders_interface.hpp"
#include "solver_interfaces/flow_mip_interface.hpp"
#include "solver_interfaces/greedy_interface.hpp"
#include "solver_interfaces/multicut_benders_interface.hpp"
#include "solver_interfaces/multiflow_benders_interface.hpp"
#include "solver_interfaces/path_lp_interface.hpp"
#include "solver_interfaces/static_interface.hpp"
#include "solver_interfaces/tree_lp_interface.hpp"

std::vector<std::shared_ptr<AbstractSolverInterface>> make_solver_interfaces() {
    return {std::make_unique<StaticInterface>(),
            std::make_unique<GreedyInterface>(),
            std::make_unique<FlowMipInterface>(),
            std::make_unique<MulticutBendersInterface>(),
            std::make_unique<FlowBendersInterface>(),
            std::make_unique<MultiflowBendersInterface>(),
            std::make_unique<TreeLpInterface>(),
            std::make_unique<PathLpInterface>()};
}

std::vector<std::shared_ptr<AbstractBoundInterface>> make_bound_interfaces() {
    std::vector<std::shared_ptr<AbstractBoundInterface>> methods{
        std::make_unique<AllOptionsInterface>(),
        std::make_unique<QkpPlanesInterface>(),
        std::make_unique<SplitLagrangeInterface>()};
    for(auto & solver : make_solver_interfaces())
        if(const auto bound = solver->bound_description())
            methods.push_back(std::make_shared<AlgorithmBoundInterface>(
                std::move(solver), bound.value()));
    return methods;
}
