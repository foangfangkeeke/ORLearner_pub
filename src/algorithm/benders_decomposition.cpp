#include "benders_decomposition.hpp"
#include "FCTP.hpp"
#include "BARP_S.hpp"
#include "wireless_charging_strategies.hpp"
#include "tools.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <tuple>

static const std::map<ProblemType,
                      std::tuple<std::function<std::unique_ptr<IDataInitializationStrategy_Benders>(const std::string&)>,
                                 std::function<std::unique_ptr<ISubProblemStrategy_Benders>()>>> strategyMap = {
    {
        FCTP,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<FCTPDataInitializationStrategy_Benders>(dataFolder);
            },
            []() { return std::make_unique<FCTPSubProblemStrategy_Benders>(); })
    },
    {
        BARP_S,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<BRSDataInitializationStrategy_Benders>(dataFolder);
            },
            []() { return std::make_unique<BRSSubProblemStrategy_Benders>(); })
    },
    {
        WIRELESS_CHARGING,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<WirelessChargingDataInitializationStrategy_Benders>(dataFolder);
            },
            []() { return std::make_unique<WirelessChargingSubProblemStrategy_Benders>(); })
    }
};

void InitializeBendersMasterModel(GRBModel& model, const std::vector<ProblemDataVar>& masterVarData,
    const std::vector<ProblemDataConstr>& masterConstrs, std::vector<GRBVar>& masterVars,
    GRBVar& theta, bool requireBinaryMasterVars)
{
    masterVars.clear();
    masterVars.reserve(masterVarData.size());

    GRBLinExpr objective = 0.0;
    for (const auto& var : masterVarData) {
        if (requireBinaryMasterVars && var.type != GRB_BINARY) {
            throw std::invalid_argument("Benders-style master problem requires binary first-stage variables");
        }

        GRBVar masterVar = model.addVar(var.lb, var.ub, var.obj, var.type, var.name);
        masterVars.push_back(masterVar);
        objective += var.obj * masterVar;
    }

    theta = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "theta");
    objective += theta;

    for (const auto& constr : masterConstrs) {
        if (constr.coeffs.size() != masterVars.size()) {
            throw std::invalid_argument("Master constraint coefficient size mismatch in Benders initialization");
        }

        GRBLinExpr lhs = 0.0;
        for (size_t idx = 0; idx < masterVars.size(); ++idx) {
            lhs += constr.coeffs[idx] * masterVars[idx];
        }
        model.addConstr(lhs, constr.sense, constr.rhs, constr.name);
    }

    model.setObjective(objective, GRB_MINIMIZE);
    model.update();
}

BendersMasterSolution GetBendersMasterSolution(GRBModel& model, const std::vector<GRBVar>& masterVars,
    const GRBVar& theta)
{
    BendersMasterSolution solution;
    solution.values.assign(masterVars.size(), 0.0);

    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        solution.values[idx] = masterVars[idx].get(GRB_DoubleAttr_X);
        solution.firstStageValue += masterVars[idx].get(GRB_DoubleAttr_Obj) * solution.values[idx];
    }

    solution.thetaValue = theta.get(GRB_DoubleAttr_X);
    solution.objectiveValue = model.get(GRB_DoubleAttr_ObjVal);
    return solution;
}

BendersCutEvaluation BuildBendersCutEvaluation(const BendersCutInfo& cutInfo, const std::vector<GRBVar>& masterVars,
    const std::vector<double>& masterValues)
{
    if (cutInfo.yCoeffs.size() != masterVars.size() || masterValues.size() != masterVars.size()) {
        throw std::invalid_argument("Benders cut size mismatch with master variables");
    }

    BendersCutEvaluation evaluation;
    evaluation.expr = cutInfo.constant;
    evaluation.lhsAtCurrent = cutInfo.constant;

    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        evaluation.expr += cutInfo.yCoeffs[idx] * masterVars[idx];
        evaluation.lhsAtCurrent += cutInfo.yCoeffs[idx] * masterValues[idx];
    }

    return evaluation;
}

BendersSubSolver::BendersSubSolver(std::unique_ptr<ISubProblemStrategy_Benders> strategy)
    : strategy(std::move(strategy))
{
    if (!this->strategy) {
        throw std::invalid_argument("Sub-problem strategy cannot be null");
    }
}

void BendersSubSolver::Init(const ProblemData& problemData)
{
    env = std::make_unique<GRBEnv>(true);
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    model = std::make_unique<GRBModel>(*env);
    strategy->InitSubProblem(problemData, *model, context);
}

Status BendersSubSolver::Solve(const ProblemData& problemData, const std::vector<double>& yValues,
    BendersCutInfo& cutInfo, double& subObj)
{
    strategy->UpdateSubProblem(problemData, *model, context, yValues);
    return strategy->SolveSubProblem(problemData, *model, context, yValues, cutInfo, subObj); // TODO: warmup
}

BendersSubSolver::~BendersSubSolver() {}

Status BendersDecomposition::Initialize()
{
    env = std::make_unique<GRBEnv>(true);
    env->set("LogFile", "gurobi_log.txt");
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    model = std::make_unique<GRBModel>(*env);

    problemData = std::make_unique<ProblemData>();

    auto it = strategyMap.find(problemType);
    if (it == strategyMap.end()) {
        throw std::invalid_argument("Unsupported problem type for Benders decomposition");
    }

    const auto& strategies = it->second;
    dataIniter = std::get<0>(strategies)(dataFolder);
    sub = std::make_unique<BendersSubSolver>(std::get<1>(strategies)());

    dataIniter->DataInit(*problemData);
    std::vector<ProblemDataConstr> masterConstrs = dataIniter->ConstrInit(*problemData);
    const auto& masterVars = problemData->getData<std::vector<ProblemDataVar>>("masterVars");
    InitializeBendersMasterModel(*model, masterVars, masterConstrs, yVars, theta);

    sub->Init(*problemData);

    bestUpperBound = std::numeric_limits<double>::infinity();
    bestLowerBound = -std::numeric_limits<double>::infinity();

    initialized = true;
    return OK;
}

Status BendersDecomposition::Solve()
{
    int iter = 0;
    while (iter < maxIters) {
        model->optimize();
        int masterStatus = model->get(GRB_IntAttr_Status);
        if (!(masterStatus == GRB_OPTIMAL || (masterStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount) > 0))) {
            std::cerr << "Master problem not solved, status: " << masterStatus << std::endl;
            Debug::OutputModel(model);
            return ERROR;
        }

        const BendersMasterSolution masterSolution = GetBendersMasterSolution(*model, yVars, theta);
        bestLowerBound = std::max(bestLowerBound, masterSolution.objectiveValue);

        BendersCutInfo cutInfo;
        double subObj = 0.0;
        Status subStatus = sub->Solve(*problemData, masterSolution.values, cutInfo, subObj);
        if (subStatus != OK) {
            std::cerr << "Sub-problem solve failed in Benders iteration " << iter << std::endl;
            return ERROR;
        }

        bool cutAdded = false;
        if (cutInfo.isOptimalityCut) {
            const BendersCutEvaluation cutEvaluation =
                BuildBendersCutEvaluation(cutInfo, yVars, masterSolution.values);

            bestUpperBound = std::min(bestUpperBound, masterSolution.firstStageValue + subObj);

            if (masterSolution.thetaValue + tolerance < cutEvaluation.lhsAtCurrent) {
                model->addConstr(cutEvaluation.expr <= theta, "opt_cut_" + std::to_string(iter));
                cutAdded = true;
            }
        }
        else { // feasibility cut
            const BendersCutEvaluation cutEvaluation =
                BuildBendersCutEvaluation(cutInfo, yVars, masterSolution.values);

            if (cutInfo.sense == '>') {
                if (cutEvaluation.lhsAtCurrent + tolerance < cutInfo.rhs) {
                    model->addConstr(cutEvaluation.expr >= cutInfo.rhs, "feas_cut_" + std::to_string(iter));
                    cutAdded = true;
                }
            }
            else if (cutInfo.sense == '<') { // rhs = constant at right side
                if (cutEvaluation.lhsAtCurrent - tolerance > cutInfo.rhs) {
                    model->addConstr(cutEvaluation.expr <= cutInfo.rhs, "feas_cut_" + std::to_string(iter));
                    cutAdded = true;
                }
            }
            else {
                std::cerr << "Unsupported feasibility cut sense: " << cutInfo.sense << std::endl;
                return ERROR;
            }
        }

        std::cout << "Benders iter " << iter;
        if (std::isfinite(bestUpperBound)) {
            std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound;
            double denom = std::max(1.0, std::fabs(bestUpperBound));
            double relGap = (bestUpperBound - bestLowerBound) / denom;
            std::cout << ", gap=" << relGap * 100.0 << "%, " << (cutInfo.isOptimalityCut ? "opt cut" : "feas cut");
        }
        std::cout << std::endl;

        if (!cutAdded) {
            std::cout << "Active master variables:" << std::endl;
            for (const auto& yVar : yVars) {
                const double value = yVar.get(GRB_DoubleAttr_X);
                if (value <= 0.5) {
                    continue;
                }
                std::cout << yVar.get(GRB_StringAttr_VarName) << std::endl;
            }
            std::cout << "theta: " << theta.get(GRB_DoubleAttr_X) << std::endl;
            std::cout << "objective: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
            std::cout << "Benders converged with no violated cuts." << std::endl;
            return OK;
        }

        model->update();

        ++iter;
    }

    std::cout << "Benders reached max iterations: " << maxIters << std::endl;
    return ERROR;
}

BendersDecomposition::BendersDecomposition(ProblemType problemType, std::string dataFolder, int maxIters, double tol)
    : problemType(problemType),
      dataFolder(dataFolder),
      maxIters(maxIters),
      tolerance(tol),
      bestUpperBound(std::numeric_limits<double>::infinity()),
      bestLowerBound(-std::numeric_limits<double>::infinity())
{}

BendersDecomposition::~BendersDecomposition() {}
