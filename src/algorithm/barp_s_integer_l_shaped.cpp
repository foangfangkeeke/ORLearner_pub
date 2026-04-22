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
using DataInitFactory = std::function<std::unique_ptr<IDataInitializationStrategy_IntegerLShaped>(const std::string&)>;
using SubFactory = std::function<std::unique_ptr<ISubProblemStrategy_IntegerLShaped>()>;

static const std::map<ProblemType, std::tuple<DataInitFactory, SubFactory>> kStrategyMap = {
    {
        BARP_S,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<BRSDataInitializationStrategy_LShaped>(dataFolder);
            },
            []() { return std::make_unique<BRSSubProblemStrategy_LShaped>(); })
    }
};

Status EvaluateIntegerSubproblem(const ProblemData& problemData, ISubProblemStrategy_IntegerLShaped& strategy,
    GRBModel& subModel, IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo, double& subObj)
{
    strategy.UpdateSubProblem(problemData, subModel, context, zValues);
    return strategy.SolveSubProblem(problemData, subModel, context, zValues, cutInfo, subObj);
}

void AddFeasibilityCut(GRBModel& model, const IntegerLShapedCutInfo& cutInfo,
    const std::vector<GRBVar>& masterVars, const std::string& cutName)
{
    GRBLinExpr cutExpr = cutInfo.constant;
    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        cutExpr += cutInfo.yCoeffs[idx] * masterVars[idx];
    }

    if (cutInfo.sense == '>') {
        model.addConstr(cutExpr >= cutInfo.rhs, cutName);
        return;
    }

    if (cutInfo.sense == '<') {
        model.addConstr(cutExpr <= cutInfo.rhs, cutName);
        return;
    }

    throw std::invalid_argument("Unsupported feasibility cut sense in integer L-shaped framework");
}
} // namespace

IntegerLShaped::IntegerLShaped(ProblemType problemType, std::string dataFolder, int maxIters, double tol)
    : problemType(problemType), dataFolder(dataFolder), maxIters(maxIters), tolerance(tol), globalLowerBound(0.0),
        bestUpperBound(std::numeric_limits<double>::infinity()),
        bestLowerBound(-std::numeric_limits<double>::infinity()),
        incumbentSecondStageValue(std::numeric_limits<double>::infinity())
{}

Status IntegerLShaped::Initialize()
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
    InitializeBendersMasterModel(*model, masterVars, masterConstrs, zVars, theta, true);

    subEnv = std::make_unique<GRBEnv>(true);
    subEnv->set(GRB_IntParam_OutputFlag, 0);
    subEnv->start();
    subModel = std::make_unique<GRBModel>(*subEnv);
    subContext.Clear();
    subProblemStrategy->InitSubProblem(*problemData, *subModel, subContext);

    const std::vector<double> warmStartZ = dataIniter->BuildWarmStartMasterValues(*problemData);
    IntegerLShapedCutInfo warmIntegerCutInfo;

    double warmIntegerValue = -1e9;
    Status integerStatus = EvaluateIntegerSubproblem(
        *problemData,
        *subProblemStrategy,
        *subModel,
        subContext,
        warmStartZ,
        warmIntegerCutInfo,
        warmIntegerValue);
    if (integerStatus != OK) {
        return integerStatus;
    }

    globalLowerBound = warmIntegerValue;
    const bool warmMasterFeasible = dataIniter->IsWarmStartMasterFeasible(*problemData, warmStartZ, tolerance);
    if (warmMasterFeasible) {
        UpdateIncumbent(warmStartZ, warmIntegerValue);
    }
    model->addConstr(theta >= globalLowerBound, "theta_global_lb");

    model->update();

    initialized = true;
    return OK;
}

IntegerLShaped::CutEval IntegerLShaped::BuildImprovedCut(const std::vector<double>& zValues,
    const std::vector<double>& zCurrent, double delta, double lowerBound, int iter) const
{
    // Improved integer cut.
    CutEval cut;
    cut.name = "integer_l_shaped_cut_improved_" + std::to_string(iter);
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

IntegerLShaped::CutEval IntegerLShaped::BuildPriorityCut(const std::vector<double>& zValues,
    const std::vector<double>& zCurrent, double delta, double lowerBound, int iter) const
{
    // Priority-weighted integer cut.
    CutEval cut;
    cut.name = "integer_l_shaped_cut_priority_" + std::to_string(iter);

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

IntegerLShaped::CutEval IntegerLShaped::BuildContinuousCut(const IntegerLShapedCutInfo& cutInfo,
    const std::vector<double>& zCurrent, int iter) const
{
    // Continuous cut from the relaxed subproblem.
    const BendersCutEvaluation cutEvaluation = BuildBendersCutEvaluation(cutInfo, zVars, zCurrent);

    CutEval cut;
    cut.name = "integer_l_shaped_cut_continuous_" + std::to_string(iter);
    cut.expr = cutEvaluation.expr;
    cut.lhsAtCurrent = cutEvaluation.lhsAtCurrent;
    return cut;
}

Status IntegerLShaped::Solve()
{
    for (int iter = 0; iter < maxIters; ++iter) {
        model->optimize();
        int masterStatus = model->get(GRB_IntAttr_Status);
        if (!(masterStatus == GRB_OPTIMAL ||
              (masterStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount) > 0))) {
            std::cerr << "Integer L-shaped master problem not solved, status: " << masterStatus << std::endl;
            Debug::OutputModel(model);
            return ERROR;
        }

        const BendersMasterSolution masterSolution = GetBendersMasterSolution(*model, zVars, theta);
        const std::vector<double>& zValues = masterSolution.values;
        const double fixedCost = masterSolution.firstStageValue;
        const double thetaValue = masterSolution.thetaValue;
        bestLowerBound = std::max(bestLowerBound, masterSolution.objectiveValue); // find a tighter lower bound

        double relaxedQValue = 0.0;
        IntegerLShapedCutInfo continuousCutInfo;
        Status relaxedStatus = subProblemStrategy->SolveRelaxedSubProblem(*problemData, *subModel,
            subContext, zValues, continuousCutInfo, relaxedQValue);
        if (relaxedStatus != OK) { // TODO: feasible cut from relaxation
            std::cout << "Integer L-shaped iter " << iter
                      << ", relaxed subproblem unavailable; fallback to integer evaluation";
            if (std::isfinite(bestUpperBound)) {
                const double denom = std::max(1.0, std::fabs(bestUpperBound));
                std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound
                          << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
            }
            std::cout << std::endl;
        } else {
            if (thetaValue + tolerance < relaxedQValue) {
                std::cout << "Integer L-shaped iter " << iter << ", theta=" << thetaValue << ", Q_lp(z)=" << relaxedQValue;
                if (std::isfinite(bestUpperBound)) {
                    const double denom = std::max(1.0, std::fabs(bestUpperBound));
                    std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound
                            << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
                }

                const CutEval cutContinuous = BuildContinuousCut(continuousCutInfo, zValues, iter);
                model->addConstr(cutContinuous.expr <= theta, cutContinuous.name);
                model->update();
                std::cout << ", cut from relaxation" << std::endl;
                continue;
            }
        }

        double aggregatedQValue = 0.0;
        IntegerLShapedCutInfo integerCutInfo;
        Status secondStageStatus = EvaluateIntegerSubproblem(*problemData, *subProblemStrategy, *subModel,
            subContext, zValues, integerCutInfo, aggregatedQValue);
        if (secondStageStatus != OK) {
            std::cerr << "Integer L-shaped second-stage evaluation failed in iteration " << iter << std::endl;
            return ERROR;
        }

        if (!integerCutInfo.isOptimalityCut) {
            AddFeasibilityCut(*model, integerCutInfo, zVars, "integer_l_shaped_feasibility_" + std::to_string(iter));

            std::cout << "Integer L-shaped iter " << iter << ", feasibility cut";
            if (std::isfinite(bestUpperBound)) {
                std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound;
                const double denom = std::max(1.0, std::fabs(bestUpperBound));
                std::cout << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
            }
            std::cout << std::endl;

            model->update();
            continue;
        }

        const double candidateObjective = fixedCost + aggregatedQValue;
        if (candidateObjective + tolerance < bestUpperBound) {
            UpdateIncumbent(zValues, aggregatedQValue);
        }

        double delta = std::max(0.0, aggregatedQValue - globalLowerBound);

        const CutEval cutImproved = BuildImprovedCut(zValues, zValues, delta, globalLowerBound, iter);
        const CutEval cutPriority = BuildPriorityCut(zValues, zValues, delta, globalLowerBound, iter);

        std::cout << "Integer L-shaped iter " << iter << ", theta=" << thetaValue << ", Q(z)=" << aggregatedQValue;
        if (std::isfinite(bestUpperBound)) {
            std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound;
            double denom = std::max(1.0, std::fabs(bestUpperBound));
            std::cout << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
        }
        std::cout << std::endl;

        if (thetaValue + tolerance >= aggregatedQValue) {
            PrintBestSolution();
            std::cout << "Integer L-shaped converged." << std::endl;
            return OK;
        }

        model->addConstr(cutImproved.expr <= theta, cutImproved.name);
        model->addConstr(cutPriority.expr <= theta, cutPriority.name);
        model->update();
    }

    std::cout << "Integer L-shaped reached max iterations: " << maxIters << std::endl;
    return ERROR;
}

void IntegerLShaped::UpdateIncumbent(const std::vector<double>& zValues, double secondStageValue)
{
    incumbentZValues = zValues;
    incumbentSecondStageValue = secondStageValue;

    double fixedCost = 0.0;
    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        fixedCost += zVars[idx].get(GRB_DoubleAttr_Obj) * zValues[idx];
    }
    bestUpperBound = fixedCost + secondStageValue;
}

void IntegerLShaped::PrintBestSolution() const
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

IntegerLShaped::~IntegerLShaped() = default;