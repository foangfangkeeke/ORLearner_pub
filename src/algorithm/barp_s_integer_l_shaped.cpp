#include "barp_s_integer_l_shaped.hpp"

#include "BARP_S.hpp"
#include "tools.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {
using DataInitFactory = std::function<std::unique_ptr<IDataInitializationStrategy_Benders>(const std::string&)>;
using SubFactory = std::function<std::unique_ptr<ISubProblemStrategy_Benders>()>;

static const std::map<ProblemType, std::tuple<DataInitFactory, SubFactory>> kStrategyMap = {
    {
        BARP_S,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<BRSDataInitializationStrategy_Benders>(dataFolder);
            },
            []() { return std::make_unique<BRSSubProblemStrategy_Benders>(); })
    }
};

Status BuildContinuousCutFromRelaxation(const ProblemData& problemData, GRBModel& subModel,
    const std::vector<double>& zValues, BendersCutInfo& cutInfo, double& relaxedObjective)
{
    try {
        const int storageCount = problemData.getData<int>("brsStorageCount");
        const int scenarioCount = static_cast<int>(
            problemData.getData<std::vector<BRSScenarioData>>("brsScenarios").size());

        auto relaxed = std::make_unique<GRBModel>(subModel.relax());
        relaxed->set(GRB_IntParam_OutputFlag, 0);
        relaxed->optimize();

        const int lpStatus = relaxed->get(GRB_IntAttr_Status);
        if (lpStatus != GRB_OPTIMAL) {
            std::cerr << "Relaxed subproblem ended with status: " << lpStatus << std::endl;
            return ERROR;
        }

        relaxedObjective = relaxed->get(GRB_DoubleAttr_ObjVal);

        cutInfo.yCoeffs.assign(static_cast<size_t>(storageCount), 0.0);
        for (int w = 0; w < scenarioCount; ++w) {
            for (int s = 0; s < storageCount; ++s) {
                const std::string constrName = "assign_w" + std::to_string(w) + "_s" + std::to_string(s);
                const GRBConstr assignConstr = relaxed->getConstrByName(constrName);
                cutInfo.yCoeffs[static_cast<size_t>(s)] += assignConstr.get(GRB_DoubleAttr_Pi);
            }
        }

        cutInfo.constant = relaxedObjective;
        for (int s = 0; s < storageCount; ++s) {
            cutInfo.constant -= cutInfo.yCoeffs[static_cast<size_t>(s)] * zValues[static_cast<size_t>(s)];
        }

        cutInfo.rhs = 0.0;
        cutInfo.isOptimalityCut = true;
        cutInfo.sense = '>';
        return OK;
    }
    catch (const GRBException& e) {
        std::cerr << "Continuous cut build failed: " << e.getMessage() << std::endl;
        return ERROR;
    }
    catch (const std::exception& e) {
        std::cerr << "Continuous cut build failed: " << e.what() << std::endl;
        return ERROR;
    }
}

Status EvaluateContinuousSubproblem(const ProblemData& problemData, ISubProblemStrategy_Benders& strategy,
    GRBModel& subModel, BendersSubProblemContext& context, const std::vector<double>& zValues, BendersCutInfo& cutInfo,
    double& relaxedObjective)
{
    strategy.UpdateSubProblem(problemData, subModel, context, zValues);
    return BuildContinuousCutFromRelaxation(problemData, subModel, zValues, cutInfo, relaxedObjective);
}

Status EvaluateIntegerSubproblem(const ProblemData& problemData, ISubProblemStrategy_Benders& strategy,
    GRBModel& subModel, BendersSubProblemContext& context, const std::vector<double>& zValues, double& subObj)
{
    strategy.UpdateSubProblem(problemData, subModel, context, zValues);

    BendersCutInfo strategyCutInfo;
    Status status = strategy.SolveSubProblem(problemData, subModel, context, zValues, strategyCutInfo, subObj);
    if (status != OK) {
        return status;
    }

    if (!strategyCutInfo.isOptimalityCut) {
        std::cerr << "Integer L-shaped framework received an unexpected feasibility response from subproblem."
                  << std::endl;
        return ERROR;
    }

    return OK;
}
}

BARPSIntegerLShaped::BARPSIntegerLShaped(ProblemType problemType, std::string dataFolder, int maxIters, double tol)
    : problemType(problemType), dataFolder(dataFolder), maxIters(maxIters), tolerance(tol), globalLowerBound(0.0),
        bestUpperBound(std::numeric_limits<double>::infinity()),
        bestLowerBound(-std::numeric_limits<double>::infinity()),
        incumbentSecondStageValue(std::numeric_limits<double>::infinity())
{}

Status BARPSIntegerLShaped::Initialize()
{
    env = std::make_unique<GRBEnv>(true);
    env->set("LogFile", "gurobi_log.txt");
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    model = std::make_shared<GRBModel>(*env);

    auto it = kStrategyMap.find(problemType);
    if (it == kStrategyMap.end()) {
        throw std::invalid_argument("Unsupported problem type for integer L-shaped framework");
    }

    dataIniter = std::get<0>(it->second)(dataFolder);
    subProblemStrategy = std::get<1>(it->second)();
    if (!subProblemStrategy) {
        throw std::invalid_argument("Sub-problem strategy cannot be null");
    }

    problemData = std::make_unique<ProblemData>();
    dataIniter->DataInit(*problemData);
    const std::vector<ProblemDataConstr> masterConstrs = dataIniter->ConstrInit(*problemData);
    const auto& masterVars = problemData->getData<std::vector<ProblemDataVar>>("masterVars");

    zVars.clear();
    zVars.reserve(masterVars.size());
    GRBLinExpr masterObjective = 0.0;
    for (const auto& var : masterVars) {
        if (var.type != GRB_BINARY) {
            throw std::invalid_argument("Integer L-shaped framework requires binary master variables");
        }
        GRBVar zVar = model->addVar(var.lb, var.ub, var.obj, var.type, var.name);
        zVars.push_back(zVar);
        masterObjective += var.obj * zVar;
    }

    theta = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "theta");
    masterObjective += theta;

    for (const auto& constr : masterConstrs) {
        GRBLinExpr lhs = 0.0;
        for (size_t idx = 0; idx < zVars.size(); ++idx) {
            lhs += constr.coeffs[idx] * zVars[idx];
        }
        model->addConstr(lhs, constr.sense, constr.rhs, constr.name);
    }

    model->setObjective(masterObjective, GRB_MINIMIZE);
    model->update();

    subEnv = std::make_unique<GRBEnv>(true);
    subEnv->set(GRB_IntParam_OutputFlag, 0);
    subEnv->start();
    subModel = std::make_unique<GRBModel>(*subEnv);
    subContext.Clear();
    subProblemStrategy->InitSubProblem(*problemData, *subModel, subContext);

    std::vector<double> allOpened(zVars.size(), 1.0);
    BendersCutInfo warmContinuousCutInfo;
    double warmRelaxedValue = 0.0;
    Status relaxedStatus = EvaluateContinuousSubproblem(
        *problemData,
        *subProblemStrategy,
        *subModel,
        subContext,
        allOpened,
        warmContinuousCutInfo,
        warmRelaxedValue);
    if (relaxedStatus != OK) {
        return relaxedStatus;
    }

    double warmIntegerValue = 0.0;
    Status integerStatus = EvaluateIntegerSubproblem(
        *problemData,
        *subProblemStrategy,
        *subModel,
        subContext,
        allOpened,
        warmIntegerValue);
    if (integerStatus != OK) {
        return integerStatus;
    }

    globalLowerBound = warmIntegerValue;
    bestLowerBound = globalLowerBound;
    UpdateIncumbent(allOpened, warmIntegerValue);

    model->addConstr(theta >= globalLowerBound, "theta_global_lb");

    CutEval warmContinuousCut = BuildContinuousCut(warmContinuousCutInfo, allOpened, -1);
    warmContinuousCut.name = "barps_cut_continuous_warm";
    model->addConstr(warmContinuousCut.expr <= theta, warmContinuousCut.name);

    model->update();

    initialized = true;
    return OK;
}

BARPSIntegerLShaped::CutEval BARPSIntegerLShaped::BuildImprovedCut(const std::vector<double>& zValues,
    const std::vector<double>& zCurrent, double delta, double lowerBound, int iter) const
{
    // Improved integer cut.
    CutEval cut;
    cut.name = "barps_cut_improved_" + std::to_string(iter);
    cut.expr = lowerBound + delta;
    cut.lhsAtCurrent = lowerBound + delta;

    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        if (zValues[idx] > 0.5) {
            continue;
        }
        cut.expr += -delta * zVars[idx];
        cut.lhsAtCurrent += -delta * zCurrent[idx];
    }
    return cut;
}

BARPSIntegerLShaped::CutEval BARPSIntegerLShaped::BuildPriorityCut(const std::vector<double>& zValues,
    const std::vector<double>& zCurrent, double delta, double lowerBound, int iter) const
{
    // Priority-weighted integer cut.
    CutEval cut;
    cut.name = "barps_cut_priority_" + std::to_string(iter);

    double maxObj = 0.0;
    for (const auto& zVar : zVars) {
        maxObj = std::max(maxObj, std::max(0.0, zVar.get(GRB_DoubleAttr_Obj)));
    }
    if (maxObj < 1e-9) {
        maxObj = 1.0;
    }

    std::vector<double> xi(zVars.size(), 1.0); // no direction preference
    double sumXiStar = 0.0;
    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        xi[idx] = 1.0 + std::max(0.0, zVars[idx].get(GRB_DoubleAttr_Obj)) / maxObj;
        if (zValues[idx] > 0.5) {
            sumXiStar += xi[idx];
        }
    }

    cut.expr = lowerBound + delta * (1.0 - sumXiStar);
    cut.lhsAtCurrent = lowerBound + delta * (1.0 - sumXiStar);

    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        const double coeff = (zValues[idx] > 0.5) ? delta * xi[idx] : -delta * xi[idx];
        cut.expr += coeff * zVars[idx];
        cut.lhsAtCurrent += coeff * zCurrent[idx];
    }

    return cut;
}

BARPSIntegerLShaped::CutEval BARPSIntegerLShaped::BuildContinuousCut(const BendersCutInfo& cutInfo,
    const std::vector<double>& zCurrent, int iter) const
{
    // Continuous cut from the relaxed subproblem.
    if (cutInfo.yCoeffs.size() != zVars.size()) {
        throw std::invalid_argument("Continuous cut coefficient size mismatch with master variables");
    }

    CutEval cut;
    cut.name = "barps_cut_continuous_" + std::to_string(iter);
    cut.expr = cutInfo.constant;
    cut.lhsAtCurrent = cutInfo.constant;

    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        cut.expr += cutInfo.yCoeffs[idx] * zVars[idx];
        cut.lhsAtCurrent += cutInfo.yCoeffs[idx] * zCurrent[idx];
    }

    return cut;
}

Status BARPSIntegerLShaped::Solve()
{
    for (int iter = 0; iter < maxIters; ++iter) {
        model->optimize();
        int masterStatus = model->get(GRB_IntAttr_Status);
        if (!(masterStatus == GRB_OPTIMAL ||
              (masterStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount) > 0))) {
            std::cerr << "BARP-S master problem not solved, status: " << masterStatus << std::endl;
            Debug::OutputModel(model);
            return ERROR;
        }

        std::vector<double> zValues(zVars.size(), 0.0);
        double fixedCost = 0.0;
        for (size_t idx = 0; idx < zVars.size(); ++idx) {
            zValues[idx] = zVars[idx].get(GRB_DoubleAttr_X);
            fixedCost += zVars[idx].get(GRB_DoubleAttr_Obj) * zValues[idx];
        }

        const double thetaValue = theta.get(GRB_DoubleAttr_X);
        const double masterObj = model->get(GRB_DoubleAttr_ObjVal);
        bestLowerBound = std::max(bestLowerBound, masterObj); // find a tighter lower bound

        double relaxedQValue = 0.0;
        BendersCutInfo continuousCutInfo;
        Status relaxedStatus = EvaluateContinuousSubproblem(*problemData, *subProblemStrategy, *subModel,
            subContext, zValues, continuousCutInfo, relaxedQValue);
        if (relaxedStatus != OK) {
            std::cerr << "BARP-S relaxed second-stage evaluation failed in iteration " << iter << std::endl;
            return ERROR;
        }

        const CutEval cutContinuous = BuildContinuousCut(continuousCutInfo, zValues, iter);

        std::cout << "BARP-S iter " << iter << ", theta=" << thetaValue << ", Q_lp(z)=" << relaxedQValue;
        if (std::isfinite(bestUpperBound)) {
            const double denom = std::max(1.0, std::fabs(bestUpperBound));
            std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound
                      << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
        }

        if (thetaValue + tolerance < relaxedQValue) {
            model->addConstr(cutContinuous.expr <= theta, cutContinuous.name);
            model->update();
            std::cout << ", cut from relaxation" << std::endl;
            continue;
        }
        std::cout << std::endl;

        double aggregatedQValue = 0.0;
        Status secondStageStatus = EvaluateIntegerSubproblem(*problemData, *subProblemStrategy, *subModel,
            subContext, zValues, aggregatedQValue);
        if (secondStageStatus != OK) {
            std::cerr << "BARP-S second-stage evaluation failed in iteration " << iter << std::endl;
            return ERROR;
        }

        const double candidateObjective = fixedCost + aggregatedQValue;
        if (candidateObjective + tolerance < bestUpperBound) {
            UpdateIncumbent(zValues, aggregatedQValue);
        }

        double delta = std::max(0.0, aggregatedQValue - globalLowerBound);

        const CutEval cutImproved = BuildImprovedCut(zValues, zValues, delta, globalLowerBound, iter);
        const CutEval cutPriority = BuildPriorityCut(zValues, zValues, delta, globalLowerBound, iter);

        std::cout << "BARP-S iter " << iter << ", theta=" << thetaValue << ", Q(z)=" << aggregatedQValue;
        if (std::isfinite(bestUpperBound)) {
            std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound;
            double denom = std::max(1.0, std::fabs(bestUpperBound));
            std::cout << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
        }
        std::cout << std::endl;

        if (thetaValue + tolerance >= aggregatedQValue) {
            PrintBestSolution();
            std::cout << "BARP-S integer L-shaped converged." << std::endl;
            return OK;
        }

        model->addConstr(cutImproved.expr <= theta, cutImproved.name);
        model->addConstr(cutPriority.expr <= theta, cutPriority.name);
        model->update();
    }

    std::cout << "BARP-S reached max iterations: " << maxIters << std::endl;
    return ERROR;
}

void BARPSIntegerLShaped::UpdateIncumbent(const std::vector<double>& zValues, double secondStageValue)
{
    incumbentZValues = zValues;
    incumbentSecondStageValue = secondStageValue;

    double fixedCost = 0.0;
    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        fixedCost += zVars[idx].get(GRB_DoubleAttr_Obj) * zValues[idx];
    }
    bestUpperBound = fixedCost + secondStageValue;
}

void BARPSIntegerLShaped::PrintBestSolution() const
{
    if (incumbentZValues.size() != zVars.size()) {
        return;
    }

    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        std::cout << zVars[idx].get(GRB_StringAttr_VarName) << ": "
                  << incumbentZValues[idx] << std::endl;
    }
    std::cout << "theta: " << incumbentSecondStageValue << std::endl;
}

BARPSIntegerLShaped::~BARPSIntegerLShaped() = default;