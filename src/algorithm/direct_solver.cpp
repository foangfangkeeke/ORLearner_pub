#include "direct_solver.hpp"
#include "cutting_stock_problem.hpp"

#include <iostream>
#include <map>
#include <functional>
#include <unordered_map>
#include <stdexcept>

// ── Problem registry ─────────────────────────────────────────────────────────
// To add a new problem: register its IMILPFormulator factory here.
static const std::map<ProblemType,
                      std::function<std::unique_ptr<IMILPFormulator>()>> directFormulatorMap = {
    {
        CUTTINGSTOCK,
        []() { return std::make_unique<CuttingStockMILPFormulator>(); }
    }
};

// ── DirectSolver ─────────────────────────────────────────────────────────────

DirectSolver::DirectSolver(ProblemType problemType) : problemType(problemType)
{
    std::cout << "===== build DirectSolver =====" << std::endl;
}

DirectSolver::~DirectSolver()
{
    std::cout << "===== destroy DirectSolver =====" << std::endl;
}

Status DirectSolver::Init()
{
    std::cout << "===== init DirectSolver =====" << std::endl;
    std::cout << "problemType=" << problemType << std::endl;

    auto it = directFormulatorMap.find(problemType);
    if (it == directFormulatorMap.end()) {
        throw std::invalid_argument("Unsupported problem type for DirectSolver");
    }
    formulator = it->second();
    return OK;
}

// Build a Gurobi model from a generic MILPModel and solve it.
Status DirectSolver::Solve(const MILPModel& model)
{
    std::cout << "===== start DirectSolver =====" << std::endl;

    GRBSolver solver(env);

    // Add variables; store them by name for constraint building.
    std::unordered_map<std::string, GRBVar> varMap;
    varMap.reserve(model.variables.size());
    for (const auto& vd : model.variables) {
        varMap[vd.name] = solver.AddVariable(vd.lb, vd.ub, 0.0, vd.type, vd.name);
    }

    // Build and set objective.
    GRBLinExpr obj = 0;
    for (const auto& vd : model.variables) {
        if (vd.objCoeff != 0.0) {
            obj += vd.objCoeff * varMap[vd.name];
        }
    }
    solver.SetObjective(obj, model.objSense);

    // Add constraints.
    for (const auto& cd : model.constraints) {
        GRBLinExpr lhs = 0;
        for (const auto& term : cd.terms) {
            auto it2 = varMap.find(term.first);
            if (it2 == varMap.end()) {
                throw std::runtime_error("Unknown variable in constraint: " + term.first);
            }
            lhs += term.second * it2->second;
        }
        solver.AddConstraint(lhs, cd.sense, cd.rhs, cd.name);
    }

    solver.Update();

    bool ok = solver.Solve();
    if (!ok) {
        std::cerr << "DirectSolver: no optimal solution found." << std::endl;
        return ERROR;
    }

    std::cout << "===== DirectSolver optimal solution =====" << std::endl;
    std::cout << "Objective value: " << solver.GetObjectiveValue() << std::endl;

    // Collect solution values in variable-declaration order.
    std::vector<double> varValues;
    varValues.reserve(model.variables.size());
    for (const auto& vd : model.variables) {
        varValues.push_back(varMap[vd.name].get(GRB_DoubleAttr_X));
    }

    formulator->PrintSolution(model, varValues);
    return OK;
}

Status DirectSolver::Run()
{
    Status status = Init();
    if (status != OK) {
        return status;
    }
    MILPModel model = formulator->Formulate();
    return Solve(model);
}

