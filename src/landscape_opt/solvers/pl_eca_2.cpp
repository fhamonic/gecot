#include "solvers/pl_eca_2.hpp"

// #include "gurobi_c.h"

namespace Solvers::PL_ECA_2_Vars {
class XVar : public OSI_Builder::VarType {
private:
    const MutableLandscape::Graph & _graph;
    const int _m;

public:
    XVar(const MutableLandscape::Graph & graph, int n, int m)
        : VarType(n * m, 0, OSI_Builder::INFTY, false), _graph(graph), _m(m) {}
    int id(MutableLandscape::Node t, MutableLandscape::Arc a) {
        const int id = _graph.id(t) * _m + _graph.id(a);
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};
class RestoredXVar : public OSI_Builder::VarType {
private:
    const MutableLandscape::Graph & _graph;
    int _nb_vars_by_node;

public:
    RestoredXVar(const RestorationPlan<MutableLandscape> & plan, int n)
        : VarType(0, 0, OSI_Builder::INFTY, false)
        , _graph(plan.getLandscape().getNetwork()) {
        _nb_vars_by_node = plan.getNbArcRestorationElements();
        _number = n * _nb_vars_by_node;
    }
    int id(MutableLandscape::Node t,
           RestorationPlan<MutableLandscape>::ArcRestorationElement e) {
        const int id = _graph.id(t) * _nb_vars_by_node + e.id;
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};
class FVar : public OSI_Builder::VarType {
private:
    const MutableLandscape::Graph & _graph;

public:
    FVar(const MutableLandscape::Graph & graph, int n)
        : VarType(n), _graph(graph) {}
    int id(MutableLandscape::Node u) {
        const int id = _graph.id(u);
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};
class RestoredFVar : public OSI_Builder::VarType {
public:
    RestoredFVar(const RestorationPlan<MutableLandscape> & plan)
        : VarType(plan.getNbNodeRestorationElements(), 0, OSI_Builder::INFTY,
                  false) {}
    int id(RestorationPlan<MutableLandscape>::NodeRestorationElement e) {
        assert(e.id >= 0 && e.id < _number);
        return _offset + e.id;
    }
};
class YVar : public OSI_Builder::VarType {
public:
    YVar(const RestorationPlan<MutableLandscape> & plan)
        : VarType(plan.getNbOptions(), 0, 1, true) {}
    int id(RestorationPlan<MutableLandscape>::Option option) {
        const int id = option;
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};

class Variables {
private:
    const MutableLandscape::Graph & graph;
    int n, m;

public:
    XVar x;
    RestoredXVar restored_x;
    FVar f;
    RestoredFVar restored_f;
    YVar y;

    Variables(const MutableLandscape & landscape,
              const RestorationPlan<MutableLandscape> & plan)
        : graph(landscape.getNetwork())
        , n(lemon::countNodes(graph))
        , m(lemon::countArcs(graph))
        , x(graph, n, m)
        , restored_x(plan, n)
        , f(graph, n)
        , restored_f(plan)
        , y(plan) {}
};

void name_variables(OSI_Builder & solver, const MutableLandscape & landscape,
                    const RestorationPlan<MutableLandscape> & plan,
                    Variables & vars) {
    const MutableLandscape::Graph & graph = landscape.getNetwork();

    auto node_str = [&graph](MutableLandscape::Node v) {
        return std::to_string(graph.id(v));
    };
    auto arc_str = [&graph, &node_str](MutableLandscape::Arc a) {
        return std::to_string(graph.id(a)) + "(" + node_str(graph.source(a)) +
               "," + node_str(graph.target(a)) + ")";
    };

    // XVar
    for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t)
        for(MutableLandscape::ArcIt a(graph); a != lemon::INVALID; ++a)
            solver.setColName(vars.x.id(t, a),
                              "x_t_" + node_str(t) + "_a_" + arc_str(a));
    // RestoredXVar
    for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t)
        for(MutableLandscape::ArcIt a(graph); a != lemon::INVALID; ++a)
            for(auto const & e : plan[a])
                solver.setColName(vars.restored_x.id(t, e),
                                  "restored_x_t_" + node_str(t) + "_a_" +
                                      arc_str(a) + "_" +
                                      std::to_string(e.option));
    // FVar
    for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t)
        solver.setColName(vars.f.id(t), "f_t_" + node_str(t));
    // RestoredFVar
    for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t)
        for(auto const & e : plan[t])
            solver.setColName(
                vars.restored_f.id(e),
                "restored_f_t_" + node_str(t) + "_" + std::to_string(e.option));
    // YVar
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options())
        solver.setColName(vars.y.id(i), "y_" + std::to_string(i));
}
}  // namespace Solvers::PL_ECA_2_Vars

using namespace Solvers::PL_ECA_2_Vars;

void insert_variables(OSI_Builder & solver_builder, Variables & vars) {
    solver_builder.addVarType(&vars.x);
    solver_builder.addVarType(&vars.restored_x);
    solver_builder.addVarType(&vars.f);
    solver_builder.addVarType(&vars.restored_f);
    solver_builder.addVarType(&vars.y);
    solver_builder.init();
}

void fill_solver(OSI_Builder & solver_builder,
                 const MutableLandscape & landscape,
                 const RestorationPlan<MutableLandscape> & plan, const double B,
                 Variables & vars, bool relaxed) {
    const MutableLandscape::Graph & graph = landscape.getNetwork();

    std::vector<MutableLandscape::Node> nodes;
    for(MutableLandscape::NodeIt u(graph); u != lemon::INVALID; ++u) {
        nodes.push_back(u);
    }
    // M_Map
    MutableLandscape::Graph::NodeMap<double> M(graph);
    std::for_each(std::execution::par, nodes.begin(), nodes.end(),
                  [&](MutableLandscape::Node t) {
                      M[t] = max_flow_in(landscape, plan, t);
                  });

    ////////////////////////////////////////////////////////////////////////
    // Columns : Objective
    ////////////////////
    for(MutableLandscape::NodeIt t(graph); t != lemon::INVALID; ++t) {
        if(landscape.getQuality(t) == 0 && !plan.contains(t)) continue;
        // sum w(t) * f_t
        const int f_t = vars.f.id(t);
        solver_builder.setObjective(f_t, landscape.getQuality(t));
        for(auto const & e : plan[t]) {
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
                solver_builder.buffEntry(x_ta, -landscape.getProbability(a));
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
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
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
}

Solution Solvers::PL_ECA_2::solve(
    const MutableLandscape & landscape,
    const RestorationPlan<MutableLandscape> & plan, const double B) const {
    Solution solution(landscape, plan);
    const int log_level = params.at("log")->getInt();
    const int timeout = params.at("timeout")->getInt();
    (void)timeout;  // pas bien
    const bool relaxed = params.at("relaxed")->getBool();
    Chrono chrono;
    OSI_Builder solver_builder;
    Variables vars(landscape, plan);
    insert_variables(solver_builder, vars);
    if(log_level > 0)
        std::cout << name()
                  << ": Start filling solver : " << solver_builder.getNbVars()
                  << " variables" << std::endl;
    fill_solver(solver_builder, landscape, plan, B, vars, relaxed);
    // OsiGrbSolverInterface * solver =
    // solver_builder.buildSolver<OsiGrbSolverInterface>(OSI_Builder::MAX);
    OsiSolverInterface * solver =
        solver_builder.buildSolver<OsiClpSolverInterface>(OSI_Builder::MAX);
    if(log_level <= 1) solver->setHintParam(OsiDoReducePrint);
    if(log_level >= 1) {
        if(log_level >= 2) {
            name_variables(solver_builder, landscape, plan, vars);
            OsiClpSolverInterface * solver_clp =
                solver_builder.buildSolver<OsiClpSolverInterface>(
                    OSI_Builder::MAX);
            solver_clp->writeLp("pl_eca_2");
            delete solver_clp;
            std::cout << name() << ": LP printed to 'pl_eca_2.lp'" << std::endl;
        }
        std::cout << name() << ": Complete filling solver : "
                  << solver_builder.getNbConstraints() << " constraints in "
                  << chrono.lapTimeMs() << " ms" << std::endl;
        std::cout << name() << ": Start solving" << std::endl;
    }
    ////////////////////
    // GRBsetdblparam(GRBgetenv(solver->getLpPtr()), GRB_DBL_PAR_MIPGAP, 1e-8);
    // GRBsetintparam(GRBgetenv(solver->getLpPtr()), GRB_INT_PAR_LOGTOCONSOLE,
    // (log_level >= 2 ? 1 : 0)); GRBsetintparam(GRBgetenv(solver->getLpPtr()),
    // GRB_DBL_PAR_TIMELIMIT, timeout);
    solver->branchAndBound();
    ////////////////////
    const double * var_solution = solver->getColSolution();
    if(var_solution == nullptr) {
        std::cerr << name() << ": Fail" << std::endl;
        delete solver;
        assert(false);
    }
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const int y_i = vars.y.id(i);
        const double value = var_solution[y_i];
        solution.set(i, value);
    }
    solution.setComputeTimeMs(chrono.timeMs());
    solution.obj = solver->getObjValue();
    solution.nb_vars = solver_builder.getNbNonZeroVars();
    solution.nb_constraints = solver_builder.getNbConstraints();
    solution.nb_elems = solver->getNumElements();
    if(log_level >= 1) {
        std::cout << name()
                  << ": Complete solving : " << solution.getComputeTimeMs()
                  << " ms" << std::endl;
        std::cout << name() << ": ECA from obj : " << std::sqrt(solution.obj)
                  << std::endl;
    }
    delete solver;

    return solution;
}

double Solvers::PL_ECA_2::eval(const MutableLandscape & landscape,
                               const RestorationPlan<MutableLandscape> & plan,
                               const double B,
                               const Solution & solution) const {
    const int log_level = params.at("log")->getInt();
    OSI_Builder solver_builder = OSI_Builder();
    Variables vars(landscape, plan);
    insert_variables(solver_builder, vars);
    if(log_level > 0) std::cout << name() << ": Start eval" << std::endl;
    fill_solver(solver_builder, landscape, plan, B, vars, false);
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const int y_i = vars.y.id(i);
        double y_i_value = solution[i];
        solver_builder.setBounds(y_i, y_i_value, y_i_value);
    }
    OsiSolverInterface * solver =
        solver_builder.buildSolver<OsiClpSolverInterface>(OSI_Builder::MAX);
    if(log_level <= 1) solver->setHintParam(OsiDoReducePrint);
    ////////////////////
    solver->initialSolve();
    ////////////////////
    const double * var_solution = solver->getColSolution();
    if(var_solution == nullptr) {
        std::cerr << name() << ": eval failed" << std::endl;
        return 0.0;
    }
    double obj = solver->getObjValue();
    if(log_level >= 1) std::cout << name() << ": eval : " << obj << std::endl;
    delete solver;
    return obj;
}