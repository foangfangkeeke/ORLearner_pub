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

Status BendersSubSolver::Solve(
    const ProblemData& problemData,
    const std::vector<double>& yValues,
    BendersCutInfo& cutInfo,
    double& subObj)
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

    yVars.clear();
    yVars.reserve(masterVars.size());
    GRBLinExpr objective = 0;
    for (const auto& var : masterVars) {
        GRBVar y = model->addVar(var.lb, var.ub, var.obj, var.type, var.name);
        yVars.push_back(y);
        objective += var.obj * y;
    }

    theta = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "theta");
    objective += theta;

    for (size_t i = 0; i < masterConstrs.size(); ++i) {
        const auto& constr = masterConstrs[i];
        if (constr.coeffs.size() != yVars.size()) {
            throw std::invalid_argument("Master constraint coefficient size mismatch in Benders initialization");
        }
        GRBLinExpr lhs = 0;
        for (size_t j = 0; j < yVars.size(); ++j) {
            lhs += constr.coeffs[j] * yVars[j];
        }
        model->addConstr(lhs, constr.sense, constr.rhs, constr.name);
    }

    model->setObjective(objective, GRB_MINIMIZE);
    model->update();

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

        std::vector<double> yValues(yVars.size(), 0.0);
        double fixedCost = 0.0;
        for (size_t i = 0; i < yVars.size(); ++i) {
            yValues[i] = yVars[i].get(GRB_DoubleAttr_X);
            fixedCost += yVars[i].get(GRB_DoubleAttr_Obj) * yValues[i];
        }

        double thetaValue = theta.get(GRB_DoubleAttr_X);
        double masterObj = model->get(GRB_DoubleAttr_ObjVal);
        bestLowerBound = std::max(bestLowerBound, masterObj);

        BendersCutInfo cutInfo;
        double subObj = 0.0;
        Status subStatus = sub->Solve(*problemData, yValues, cutInfo, subObj);
        if (subStatus != OK) {
            std::cerr << "Sub-problem solve failed in Benders iteration " << iter << std::endl;
            return ERROR;
        }

        bool cutAdded = false;
        if (cutInfo.isOptimalityCut) {
            GRBLinExpr lhsExpr = cutInfo.constant; // The parts of the cut that are not related to y.
            double lhsCurrent = cutInfo.constant;
            for (size_t i = 0; i < yVars.size(); ++i) {
                lhsExpr += cutInfo.yCoeffs[i] * yVars[i];
                lhsCurrent += cutInfo.yCoeffs[i] * yValues[i];
            }

            bestUpperBound = std::min(bestUpperBound, fixedCost + subObj);

            if (thetaValue + tolerance < lhsCurrent) {
                model->addConstr(lhsExpr <= theta, "opt_cut_" + std::to_string(iter));
                cutAdded = true;
            }
        }
        else { // feasibility cut
            GRBLinExpr lhsExpr = 0;
            double lhsCurrent = 0.0;
            for (size_t i = 0; i < yVars.size(); ++i) {
                lhsExpr += cutInfo.yCoeffs[i] * yVars[i];
                lhsCurrent += cutInfo.yCoeffs[i] * yValues[i];
            }

            if (cutInfo.sense == '>') {
                if (lhsCurrent + tolerance < cutInfo.rhs) {
                    model->addConstr(lhsExpr >= cutInfo.rhs, "feas_cut_" + std::to_string(iter));
                    cutAdded = true;
                }
            }
            else if (cutInfo.sense == '<') { // rhs = constant at right side
                if (lhsCurrent - tolerance > cutInfo.rhs) {
                    model->addConstr(lhsExpr <= cutInfo.rhs, "feas_cut_" + std::to_string(iter));
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
            Debug::OutputResult(model);
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
