#include "solvers/pl_eca_3.hpp"

#include "CglFlowCover.hpp"
#include "CglMixedIntegerRounding2.hpp"
#include "gurobi_c.h"

namespace Solvers::PL_ECA_3_Vars {
class PreprocessedDatas {
public:
    std::vector<MutableLandscape::Node> target_nodes;
    std::unique_ptr<
        MutableLandscape::Graph::NodeMap<std::shared_ptr<ContractionResult>>>
        contracted_instances;
    MutableLandscape::Graph::NodeMap<StaticLandscape::Graph::NodeMap<double> *>
        M_Maps_Map;

    PreprocessedDatas(const MutableLandscape & landscape,
                      const RestorationPlan<MutableLandscape> & plan)
        : M_Maps_Map(landscape.getNetwork()) {
        const MutableLandscape::Graph & graph = landscape.getNetwork();
        // target_nodes
        for(MutableLandscape::NodeIt u(graph); u != lemon::INVALID; ++u) {
            if(landscape.getQuality(u) == 0 && !plan.contains(u)) continue;
            target_nodes.push_back(u);
        }
        // contracted_instances
        MyContractionAlgorithm alg2;
        contracted_instances = alg2.precompute(landscape, plan, target_nodes);
        // M_Maps_Map
        std::for_each(
            std::execution::par, target_nodes.begin(), target_nodes.end(),
            [&](MutableLandscape::Node t) {
                const ContractionResult & cr = *(*contracted_instances)[t];
                const StaticLandscape & contracted_landscape = cr.landscape;
                const StaticLandscape::Graph & contracted_graph =
                    contracted_landscape.getNetwork();
                const RestorationPlan<StaticLandscape> & contracted_plan =
                    cr.plan;

                M_Maps_Map[t] = new StaticLandscape::Graph::NodeMap<double>(
                    contracted_graph);
                StaticLandscape::Graph::NodeMap<double> & M_Map =
                    *M_Maps_Map[t];
                for(StaticLandscape::NodeIt v(contracted_graph);
                    v != lemon::INVALID; ++v)
                    M_Map[v] =
                        max_flow_in(contracted_landscape, contracted_plan, v);
            });
    }
    ~PreprocessedDatas() {
        for(MutableLandscape::Node v : target_nodes) delete M_Maps_Map[v];
    }
};

class XVar : public OSI_Builder::VarType {
private:
    const ContractionResult & _cr;

public:
    XVar(const ContractionResult & cr)
        : VarType(lemon::countArcs(cr.landscape.getNetwork())), _cr(cr) {}
    int id(StaticLandscape::Arc a) const {
        const int id = _cr.landscape.getNetwork().id(a);
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};
class RestoredXVar : public OSI_Builder::VarType {
public:
    RestoredXVar(const ContractionResult & cr)
        : VarType(cr.plan.getNbArcRestorationElements()) {}
    int id(RestorationPlan<StaticLandscape>::ArcRestorationElement e) const {
        assert(e.id >= 0 && e.id < _number);
        return _offset + e.id;
    }
};
class FVar : public OSI_Builder::VarType {
public:
    FVar() : VarType(1) {}
    int id() const { return _offset; }
};
class RestoredFVar : public OSI_Builder::VarType {
public:
    RestoredFVar(const ContractionResult & cr)
        : VarType(cr.plan.getNbNodeRestorationElements()) {}
    int id(RestorationPlan<MutableLandscape>::NodeRestorationElement e) const {
        assert(e.id >= 0 && e.id < _number);
        return _offset + e.id;
    }
};
class YVar : public OSI_Builder::VarType {
public:
    YVar(const RestorationPlan<MutableLandscape> & plan)
        : VarType(plan.getNbOptions(), 0, 1, true) {}
    int id(RestorationPlan<MutableLandscape>::Option option) const {
        const int id = option;
        assert(id >= 0 && id < _number);
        return _offset + id;
    }
};

class ContractedVars {
public:
    XVar x;
    RestoredXVar restored_x;
    FVar f;
    RestoredFVar restored_f;
    ContractedVars(const ContractionResult & cr)
        : x(cr), restored_x(cr), f(), restored_f(cr){};
};

class Variables {
public:
    MutableLandscape::Graph::NodeMap<ContractedVars *> contracted;
    YVar y;
    const PreprocessedDatas & _pdatas;
    Variables(const MutableLandscape & landscape,
              const RestorationPlan<MutableLandscape> & plan,
              PreprocessedDatas & pdatas)
        : contracted(landscape.getNetwork(), nullptr)
        , y(plan)
        , _pdatas(pdatas) {
        for(MutableLandscape::Node v : pdatas.target_nodes)
            contracted[v] =
                new ContractedVars(*(*pdatas.contracted_instances)[v]);
    }
    ~Variables() {
        for(MutableLandscape::Node v : _pdatas.target_nodes)
            delete contracted[v];
    }
    ContractedVars & operator[](MutableLandscape::Node t) const {
        return *contracted[t];
    }
};

void name_variables(OSI_Builder & solver, const MutableLandscape & landscape,
                    const RestorationPlan<MutableLandscape> & plan,
                    PreprocessedDatas & pdatas, const Variables & vars) {
    const MutableLandscape::Graph & graph = landscape.getNetwork();
    auto node_str = [&graph](MutableLandscape::Node v) {
        return std::to_string(graph.id(v));
    };
    // XVar
    for(MutableLandscape::Node t : pdatas.target_nodes) {
        const ContractedVars & cvars = vars[t];
        const ContractionResult & cr = *(*pdatas.contracted_instances)[t];
        const StaticLandscape & contracted_landscape = cr.landscape;
        const StaticLandscape::Graph & contracted_graph =
            contracted_landscape.getNetwork();
        const RestorationPlan<StaticLandscape> & contracted_plan = cr.plan;
        for(StaticLandscape::ArcIt a(contracted_graph); a != lemon::INVALID;
            ++a) {
            // solver->setColName(x_var->id(a), "x_t_" + node_str(t) + "_a_" +
            // std::to_string(contracted_graph.id(a)));
            solver.setColName(
                cvars.x.id(a),
                "x_t_" + node_str(t) + "(" +
                    std::to_string(contracted_graph.id(cr.t)) + ")_a_" +
                    std::to_string(contracted_graph.id(a)) + "(" +
                    std::to_string(
                        contracted_graph.id(contracted_graph.source(a))) +
                    "," +
                    std::to_string(
                        contracted_graph.id(contracted_graph.target(a))) +
                    ")");
            // RestoredXVar
            for(auto const & e : contracted_plan[a])
                solver.setColName(cvars.restored_x.id(e),
                                  "restored_x_t_" + node_str(t) + "_a_" +
                                      std::to_string(contracted_graph.id(a)) +
                                      "_" + std::to_string(e.option));
        }
    }
    // FVar
    for(MutableLandscape::Node t : pdatas.target_nodes)
        solver.setColName(vars[t].f.id(), "f_t_" + node_str(t));
    // RestoredFVar
    for(MutableLandscape::Node t : pdatas.target_nodes)
        for(auto const & e : plan[t])
            solver.setColName(
                vars[t].restored_f.id(e),
                "restored_f_t_" + node_str(t) + "_" + std::to_string(e.option));
    // YVar
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options())
        solver.setColName(vars.y.id(i), "y_" + std::to_string(i));
}
}  // namespace Solvers::PL_ECA_3_Vars

using namespace Solvers::PL_ECA_3_Vars;

void insert_variables(OSI_Builder & solver_builder, Variables & vars,
                      PreprocessedDatas & pdatas) {
    for(MutableLandscape::Node t : pdatas.target_nodes) {
        solver_builder.addVarType(&vars[t].x);
        solver_builder.addVarType(&vars[t].restored_x);
        solver_builder.addVarType(&vars[t].f);
        solver_builder.addVarType(&vars[t].restored_f);
    }
    solver_builder.addVarType(&vars.y);
    solver_builder.init();
}

void fill_solver(OSI_Builder & solver_builder,
                 const MutableLandscape & landscape,
                 const RestorationPlan<MutableLandscape> & plan, const double B,
                 Variables & vars, PreprocessedDatas & pdatas) {
    for(MutableLandscape::Node t : pdatas.target_nodes) {
        ContractionResult & cr = *(*pdatas.contracted_instances)[t];
        cr.plan.initElementIDs();
    }

    auto M_x_const = [&](MutableLandscape::Node t, StaticLandscape::Arc a) {
        const ContractionResult & cr = *(*pdatas.contracted_instances)[t];
        const StaticLandscape::Graph & contracted_graph =
            cr.landscape.getNetwork();
        return (*pdatas.M_Maps_Map[t])[contracted_graph.source(a)];
    };
    auto M_f_const = [&](MutableLandscape::Node t) {
        const ContractionResult & cr = *(*pdatas.contracted_instances)[t];
        return (*pdatas.M_Maps_Map[t])[cr.t];
    };
    ////////////////////////////////////////////////////////////////////////
    // Columns : Objective
    ////////////////////
    for(MutableLandscape::Node t : pdatas.target_nodes) {
        // sum w(t) * f_t
        const int f_t = vars[t].f.id();
        solver_builder.setObjective(f_t, landscape.getQuality(t));
        for(auto const & e : plan[t]) {
            const int restored_f_t = vars[t].restored_f.id(e);
            solver_builder.setObjective(restored_f_t, e.quality_gain);
        }
    }
    ////////////////////////////////////////////////////////////////////////
    // Rows : Constraints
    ////////////////////
    for(MutableLandscape::Node t : pdatas.target_nodes) {
        const ContractedVars & cvars = vars[t];
        const int f_t = cvars.f.id();
        const ContractionResult & cr = *(*pdatas.contracted_instances)[t];
        const StaticLandscape & contracted_landscape = cr.landscape;
        const StaticLandscape::Graph & contracted_graph =
            contracted_landscape.getNetwork();
        const RestorationPlan<StaticLandscape> & contracted_plan = cr.plan;
        // out_flow(u) - in_flow(u) <= w(u)
        for(StaticLandscape::NodeIt u(contracted_graph); u != lemon::INVALID;
            ++u) {
            // out flow
            for(StaticLandscape::Graph::OutArcIt b(contracted_graph, u);
                b != lemon::INVALID; ++b) {
                const int x_tb = cvars.x.id(b);
                solver_builder.buffEntry(x_tb, 1);
                for(auto const & e : contracted_plan[b]) {
                    const int restored_x_t_b = cvars.restored_x.id(e);
                    solver_builder.buffEntry(restored_x_t_b, 1);
                }
            }
            // in flow
            for(StaticLandscape::Graph::InArcIt a(contracted_graph, u);
                a != lemon::INVALID; ++a) {
                const int x_ta = cvars.x.id(a);
                solver_builder.buffEntry(
                    x_ta, -contracted_landscape.getProbability(a));
                for(auto const & e : contracted_plan[a]) {
                    const int degraded_x_t_a = cvars.restored_x.id(e);
                    solver_builder.buffEntry(degraded_x_t_a,
                                             -e.restored_probability);
                }
            }
            // optional injected flow
            for(auto const & e : contracted_plan[u]) {
                const int y_u = vars.y.id(e.option);
                solver_builder.buffEntry(y_u, -e.quality_gain);
            }
            // optimisation variable
            if(u == cr.t) solver_builder.buffEntry(f_t, 1);
            // injected flow
            solver_builder.pushRow(-OSI_Builder::INFTY,
                                   contracted_landscape.getQuality(u));
        }
        // restored_x_a < y_i * M
        for(StaticLandscape::ArcIt a(contracted_graph); a != lemon::INVALID;
            ++a) {
            for(auto const & e : contracted_plan[a]) {
                const int y_i = vars.y.id(e.option);
                const int x_ta = cvars.restored_x.id(e);
                solver_builder.buffEntry(y_i, M_x_const(t, a));
                solver_builder.buffEntry(x_ta, -1);
                solver_builder.pushRow(0, OSI_Builder::INFTY);
            }
        }
        // restored_f_t <= f_t
        // restored_f_t <= y_i * M
        for(const auto & e : plan[t]) {
            const int y_i = vars.y.id(e.option);
            const int restored_f_t = cvars.restored_f.id(e);
            solver_builder.buffEntry(f_t, 1);
            solver_builder.buffEntry(restored_f_t, -1);
            solver_builder.pushRow(0, OSI_Builder::INFTY);

            solver_builder.buffEntry(y_i, M_f_const(t));
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
    solver_builder.pushRow(0, B);
}

Solution Solvers::PL_ECA_3::solve(
    const MutableLandscape & landscape,
    const RestorationPlan<MutableLandscape> & plan, const double B) const {
    Solution solution(landscape, plan);
    const int log_level = params.at("log")->getInt();
    const int timeout = params.at("timeout")->getInt();
    (void)timeout;  // pas bien
    const bool relaxed = params.at("relaxed")->getBool();
    Chrono chrono;
    if(log_level > 0)
        std::cout << name() << ": Start preprocessing" << std::endl;
    PreprocessedDatas preprocessed_datas(landscape, plan);
    OSI_Builder solver_builder = OSI_Builder();
    Variables vars(landscape, plan, preprocessed_datas);
    insert_variables(solver_builder, vars, preprocessed_datas);
    if(log_level > 0) {
        std::cout << name()
                  << ": Complete preprocessing : " << chrono.lapTimeMs()
                  << " ms" << std::endl;
        std::cout << name()
                  << ": Start filling solver : " << solver_builder.getNbVars()
                  << " variables" << std::endl;
    }
    fill_solver(solver_builder, landscape, plan, B, vars, preprocessed_datas);
#define WITH_GUROBI
#ifndef WITH_GUROBI
    OsiSolverInterface * solver =
        solver_builder.buildSolver<OsiClpSolverInterface>(OSI_Builder::MAX);
    if(log_level <= 1) solver->setHintParam(OsiDoReducePrint);
    if(log_level >= 1) {
        if(log_level >= 3) {
            name_variables(solver_builder, landscape, plan, preprocessed_datas,
                           vars);
            OsiClpSolverInterface * solver_clp =
                solver_builder.buildSolver<OsiClpSolverInterface>(
                    OSI_Builder::MAX);
            solver_clp->writeLp("pl_eca_3");
            delete solver_clp;
            std::cout << name() << ": LP printed to 'pl_eca_3.lp'" << std::endl;
        }
        std::cout << name() << ": Complete filling solver : "
                  << solver_builder.getNbConstraints() << " constraints and "
                  << solver_builder.getNbElems() << " entries in "
                  << chrono.lapTimeMs() << " ms" << std::endl;
        std::cout << name() << ": Start solving" << std::endl;
    }

    solver->initialSolve();
    CbcModel model(*solver);
    model.setLogLevel(log_level - 1);
    model.setNumberThreads(8);
    CglFlowCover cut_flow;
    model.addCutGenerator(&cut_flow, 1, "FlowCover");
    CglMixedIntegerRounding2 cut_mir;
    model.addCutGenerator(&cut_mir, 1, "MIR");
    model.setAllowableGap(1e-10);
    CbcMain0(model);
    model.branchAndBound(1);
    ////////////////////
    const double * var_solution = model.bestSolution();
    if(var_solution == nullptr) {
        std::cerr << name() << ": Fail" << std::endl;
        delete solver;
        throw "caca";
    }
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const int y_i = vars.y.id(i);
        double value = var_solution[y_i];
        solution.set(i, value);
    }
    solution.setComputeTimeMs(chrono.timeMs());
    solution.obj = model.getObjValue();
    solution.nb_vars = solver_builder.getNbNonZeroVars();
    solution.nb_constraints = solver_builder.getNbConstraints();
    solution.nb_elems = model.getNumElements();
    if(log_level >= 1) {
        std::cout << name()
                  << ": Complete solving : " << solution.getComputeTimeMs()
                  << " ms" << std::endl;
        std::cout << name() << ": ECA from obj : " << std::sqrt(solution.obj)
                  << std::endl;
    }
    delete solver;
    return solution;
#else
    const int nb_vars = solver_builder.getNbVars();
    double * objective = solver_builder.getObjective();
    double * col_lb = solver_builder.getColLB();
    double * col_ub = solver_builder.getColUB();
    char * vtype = new char[nb_vars];
    for(OSI_Builder::VarType * varType : solver_builder.getVarTypes()) {
        const int offset = varType->getOffset();
        const int last_id = varType->getOffset() + varType->getNumber() - 1;
        for(int i = offset; i <= last_id; i++) {
            vtype[i] = (!relaxed && varType->isInteger() ? GRB_BINARY
                                                         : GRB_CONTINUOUS);
        }
    }

    CoinPackedMatrix * matrix = solver_builder.getMatrix();
    const int nb_rows = matrix->getNumRows();
    const int nb_elems = matrix->getNumElements();
    int * begins = new int[nb_rows];
    std::copy(matrix->getVectorStarts(), matrix->getVectorStarts() + nb_rows,
              begins);
    int * indices = new int[nb_elems];
    std::copy(matrix->getIndices(), matrix->getIndices() + nb_elems, indices);
    double * elements = new double[nb_elems];
    std::copy(matrix->getElements(), matrix->getElements() + nb_elems,
              elements);
    double * row_lb = solver_builder.getRowLB();
    double * row_ub = solver_builder.getRowUB();

    GRBenv * env = NULL;
    GRBmodel * model = NULL;
    GRBemptyenv(&env);
    GRBstartenv(env);
    ////////////////////
    GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, 1e-8);
    GRBsetintparam(env, GRB_INT_PAR_LOGTOCONSOLE, (log_level >= 2 ? 1 : 0));
    GRBsetintparam(env, GRB_INT_PAR_THREADS, 8);
    GRBsetdblparam(env, GRB_DBL_PAR_TIMELIMIT, timeout);
    ////////////////////
    GRBnewmodel(env, &model, "pl_eca_3", 0, NULL, NULL, NULL, NULL, NULL);
    GRBaddvars(model, nb_vars, 0, NULL, NULL, NULL, objective, col_lb, col_ub,
               vtype, NULL);
    GRBaddrangeconstrs(model, nb_rows, nb_elems, begins, indices, elements,
                       row_lb, row_ub, NULL);
    ////////////////////
    GRBsetintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MAXIMIZE);

    if(log_level >= 1) {
        if(log_level >= 2) {
            name_variables(solver_builder, landscape, plan, preprocessed_datas,
                           vars);
            OsiClpSolverInterface * solver_clp =
                solver_builder.buildSolver<OsiClpSolverInterface>(
                    OSI_Builder::MAX);
            solver_clp->writeLp("pl_eca_3");
            delete solver_clp;
            std::cout << name() << ": LP printed to 'pl_eca_3.lp'" << std::endl;
        }
        std::cout << name() << ": Complete filling solver : "
                  << solver_builder.getNbConstraints() << " constraints and "
                  << solver_builder.getNbElems() << " entries in "
                  << chrono.lapTimeMs() << " ms" << std::endl;
        std::cout << name() << ": Start solving" << std::endl;
    }

    GRBoptimize(model);
    ////////////////////
    int status;
    GRBgetintattr(model, GRB_INT_ATTR_STATUS, &status);
    if(status == GRB_INF_OR_UNBD) {
        std::cout << "Model is infeasible or unbounded" << std::endl;
    } else if(status != GRB_OPTIMAL) {
        std::cout << "Optimization was stopped early" << std::endl;
    }
    double obj;
    GRBgetdblattr(model, GRB_DBL_ATTR_OBJVAL, &obj);
    double * var_solution = new double[nb_vars];
    GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nb_vars, var_solution);
    if(var_solution == nullptr) {
        std::cerr << name() << ": Fail" << std::endl;
        assert(false);
    }
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const int y_i = vars.y.id(i);
        double value = var_solution[y_i];
        solution.set(i, value);
    }
    solution.setComputeTimeMs(chrono.timeMs());
    solution.obj = obj;
    solution.nb_vars = solver_builder.getNbNonZeroVars();
    solution.nb_constraints = solver_builder.getNbConstraints();
    solution.nb_elems = nb_elems;
    if(log_level >= 1) {
        std::cout << name()
                  << ": Complete solving : " << solution.getComputeTimeMs()
                  << " ms" << std::endl;
        std::cout << name() << ": ECA from obj : " << std::sqrt(solution.obj)
                  << std::endl;
    }

    GRBfreemodel(model);
    GRBfreeenv(env);

    delete[] vtype;
    delete[] begins;
    delete[] indices;
    delete[] elements;
    delete[] var_solution;

    return solution;
#endif
}

double Solvers::PL_ECA_3::eval(const MutableLandscape & landscape,
                               const RestorationPlan<MutableLandscape> & plan,
                               const double B,
                               const Solution & solution) const {
    const int log_level = params.at("log")->getInt();
    Chrono chrono;
    PreprocessedDatas preprocessed_datas(landscape, plan);
    OSI_Builder solver_builder = OSI_Builder();
    Variables vars(landscape, plan, preprocessed_datas);
    insert_variables(solver_builder, vars, preprocessed_datas);
    fill_solver(solver_builder, landscape, plan, B, vars, preprocessed_datas);
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const int y_i = vars.y.id(i);
        double y_i_value = solution[i];
        solver_builder.setBounds(y_i, y_i_value, y_i_value);
    }
    OsiSolverInterface * solver =
        solver_builder.buildSolver<OsiClpSolverInterface>(OSI_Builder::MAX);
    ////////////////////
    solver->initialSolve();
    ////////////////////
    const double * var_solution = solver->getColSolution();
    if(var_solution == nullptr) {
        std::cerr << name() << ": Fail" << std::endl;
        delete solver;
        return 0.0;
    }
    double obj = solver->getObjValue();
    if(log_level >= 1) std::cout << name() << ": eval : " << obj << std::endl;
    delete solver;
    return obj;
}