#include "barp_s_integer_l_shaped.hpp"

#include "BARP_S.hpp"
#include "wireless_charging_strategies.hpp"
#include "tools.hpp"

#include <algorithm>
#include <cmath>
#include <future>
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

enum class WarmStartStrategy {
    ZeroLowerBound,
    IntegerSubProblem,
    RelaxedSubProblem
};

enum class RelaxedCutStrategy {
    Aggregate,
    MultiCut
};

constexpr WarmStartStrategy kWarmStartStrategy = WarmStartStrategy::RelaxedSubProblem;
constexpr bool kPruneByUb = false;
constexpr RelaxedCutStrategy kRelaxedCutStrategy = RelaxedCutStrategy::Aggregate;

const char* WarmStartStrategyName()
{
    if constexpr (kWarmStartStrategy == WarmStartStrategy::ZeroLowerBound) {
        return "zero lower bound";
    }
    if constexpr (kWarmStartStrategy == WarmStartStrategy::IntegerSubProblem) {
        return "integer subproblem";
    }
    return "relaxed subproblem";
}

struct SubProblemEvalResult {
    Status status = ERROR;
    IntegerLShapedCutInfo cutInfo;
    double subObj = 0.0;
};

static const std::map<ProblemType, std::tuple<DataInitFactory, SubFactory>> kStrategyMap = {
    {
        BARP_S,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<BRSDataInitializationStrategy_LShaped>(dataFolder);
            },
            []() { return std::make_unique<BRSSubProblemStrategy_LShaped>(); })
    },
    {
        WIRELESS_CHARGING,
        std::make_tuple(
            [](const std::string& dataFolder) {
                return std::make_unique<WirelessChargingDataInitializationStrategy_LShaped>(dataFolder);
            },
            []() { return std::make_unique<WirelessChargingSubProblemStrategy_LShaped>(); })
    }
};

SubProblemEvalResult EvaluateScenarioSubproblem(const ProblemData& problemData,
    ISubProblemStrategy_IntegerLShaped& strategy, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues, bool relaxed)
{
    SubProblemEvalResult result;
    strategy.UpdateSubProblem(problemData, subModel, context, zValues);
    result.status = relaxed
        ? strategy.SolveRelaxedSubProblem(problemData, subModel, context, zValues,
              result.cutInfo, result.subObj)
        : strategy.SolveSubProblem(problemData, subModel, context, zValues,
              result.cutInfo, result.subObj);
    return result;
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

struct IntegerLShaped::ScenarioSubProblem {
    int scenarioIndex = 0;
    std::unique_ptr<ISubProblemStrategy_IntegerLShaped> strategy;
    std::unique_ptr<GRBEnv> env;
    std::unique_ptr<GRBModel> model;
    IntegerLShapedSubProblemContext context;
};

IntegerLShaped::IntegerLShaped(ProblemType problemType, std::string dataFolder, int maxIters, double tol)
    : problemType(problemType), dataFolder(dataFolder), maxIters(maxIters), tolerance(tol), globalLowerBound(0.0),
        bestUpperBound(std::numeric_limits<double>::infinity()),
        bestLowerBound(-std::numeric_limits<double>::infinity()),
        incumbentSecondStageValue(std::numeric_limits<double>::infinity())
{}

Status IntegerLShaped::Initialize()
{
    env = std::make_unique<GRBEnv>(true);
    env->set(GRB_IntParam_OutputFlag, 0);
    env->set("LogFile", "gurobi_log.txt");
    env->start();
    model = std::make_shared<GRBModel>(*env);
    model->set(GRB_IntParam_OutputFlag, 0);

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
    const auto dataInitStart = Tools::Clock::now();
    dataIniter->DataInit(*problemData);
    const std::vector<ProblemDataConstr> masterConstrs = dataIniter->ConstrInit(*problemData);
    const auto& masterVars = problemData->getData<std::vector<ProblemDataVar>>("masterVars");
    InitializeBendersMasterModel(*model, masterVars, masterConstrs, zVars, theta, true);
    model->set(GRB_DoubleParam_MIPGap, 0.0);
    const auto dataInitEnd = Tools::Clock::now();
    std::cout << "Integer L-shaped data/master initialization elapsed_ms="
              << Tools::ElapsedMs(dataInitStart, dataInitEnd) << std::endl;

    subProblems.clear();
    const int scenarioCount = subProblemStrategy->ScenarioCount(*problemData);
    if (scenarioCount <= 0) {
        throw std::runtime_error("Integer L-shaped requires at least one scenario sub-problem.");
    }

    relaxedThetaVars.clear();
    if constexpr (kRelaxedCutStrategy == RelaxedCutStrategy::MultiCut) {
        relaxedThetaVars.reserve(static_cast<size_t>(scenarioCount));
        GRBLinExpr relaxedThetaSum = 0.0;
        for (int scenarioIndex = 0; scenarioIndex < scenarioCount; ++scenarioIndex) {
            GRBVar scenarioTheta = model->addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                "theta_relaxed_scenario_" + std::to_string(scenarioIndex));
            relaxedThetaVars.push_back(scenarioTheta);
            relaxedThetaSum += scenarioTheta;
        }
        model->addConstr(relaxedThetaSum <= theta, "theta_relaxed_sum");
    }

    const auto subProblemsInitStart = Tools::Clock::now();
    subProblems.reserve(static_cast<size_t>(scenarioCount));
    for (int scenarioIndex = 0; scenarioIndex < scenarioCount; ++scenarioIndex) {
        auto sub = std::make_unique<ScenarioSubProblem>();
        sub->scenarioIndex = scenarioIndex;
        sub->strategy = subProblemStrategy->Clone();
        sub->env = std::make_unique<GRBEnv>(true);
        sub->env->set(GRB_IntParam_OutputFlag, 0);
        sub->env->start();
        sub->model = std::make_unique<GRBModel>(*sub->env);
        sub->model->set(GRB_IntParam_OutputFlag, 0);
        sub->model->set(GRB_DoubleParam_MIPGap, 0.0);
        sub->context.Clear();
        sub->strategy->InitSubProblem(*problemData, scenarioIndex, *sub->model, sub->context);
        subProblems.push_back(std::move(sub));
    }
    const auto subProblemsInitEnd = Tools::Clock::now();
    std::cout << "Integer L-shaped sub-problems initialization elapsed_ms="
              << Tools::ElapsedMs(subProblemsInitStart, subProblemsInitEnd) << std::endl;

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

IntegerLShaped::CutEval IntegerLShaped::BuildClassicCut(const std::vector<double>& zValues,
    const std::vector<double>& zCurrent, double delta, double lowerBound, int iter) const
{
    // Classic integer L-shaped cut (uniform coefficients, xi=1 for all variables).
    CutEval cut;
    cut.name = "integer_l_shaped_cut_classic_" + std::to_string(iter);

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

IntegerLShaped::CutEval IntegerLShaped::BuildContinuousCut(const IntegerLShapedCutInfo& cutInfo,
    const std::vector<double>& zCurrent, int iter, int scenarioIndex) const
{
    // Continuous cut from the relaxed subproblem.
    const BendersCutEvaluation cutEvaluation = BuildBendersCutEvaluation(cutInfo, zVars, zCurrent);

    CutEval cut;
    cut.name = "integer_l_shaped_cut_continuous_" + std::to_string(iter);
    if (scenarioIndex >= 0) {
        cut.name += "_scenario_" + std::to_string(scenarioIndex);
    }
    cut.expr = cutEvaluation.expr;
    cut.lhsAtCurrent = cutEvaluation.lhsAtCurrent;
    return cut;
}

Status IntegerLShaped::EvaluateSubProblems(const std::vector<double>& zValues, bool relaxed,
    IntegerLShapedCutInfo& cutInfo, double& subObj, std::vector<double>* scenarioSubObjs,
    std::vector<IntegerLShapedCutInfo>* scenarioCutInfos)
{
    if (subProblems.empty()) {
        std::cerr << "Integer L-shaped sub-problems are not initialized" << std::endl;
        return ERROR;
    }

    std::vector<std::future<SubProblemEvalResult>> futures;
    futures.reserve(subProblems.size());
    const ProblemData* problemDataPtr = problemData.get();
    for (auto& sub : subProblems) { // TODO: parallel execution is not necessary and should be optional.
        futures.push_back(std::async(std::launch::async, [problemDataPtr, &zValues, relaxed, sub = sub.get()] {
            return EvaluateScenarioSubproblem(*problemDataPtr, *sub->strategy, *sub->model,
                sub->context, zValues, relaxed);
        }));
    }

    cutInfo = IntegerLShapedCutInfo{};
    cutInfo.isOptimalityCut = true;
    cutInfo.sense = '>';
    cutInfo.rhs = 0.0;
    cutInfo.yCoeffs.assign(zValues.size(), 0.0);
    cutInfo.constant = 0.0;
    subObj = 0.0;
    if (scenarioSubObjs != nullptr) {
        scenarioSubObjs->assign(subProblems.size(), 0.0);
    }
    if (scenarioCutInfos != nullptr) {
        scenarioCutInfos->assign(subProblems.size(), IntegerLShapedCutInfo{});
    }

    size_t scenarioResultIdx = 0;
    for (auto& future : futures) {
        SubProblemEvalResult result;
        try {
            result = future.get();
        }
        catch (const std::exception& e) {
            std::cerr << "Integer L-shaped sub-problem failed: " << e.what() << std::endl;
            return ERROR;
        }

        if (result.status != OK) {
            return result.status;
        }

        if (scenarioSubObjs != nullptr) {
            (*scenarioSubObjs)[scenarioResultIdx] = result.subObj;
        }
        if (scenarioCutInfos != nullptr) {
            (*scenarioCutInfos)[scenarioResultIdx] = result.cutInfo;
        }
        ++scenarioResultIdx;

        if (!result.cutInfo.isOptimalityCut) {
            if (scenarioSubObjs != nullptr && scenarioCutInfos == nullptr) {
                throw std::runtime_error("Integer L-shaped scenario lower bounds require optimal sub-problem results");
            }
            cutInfo = result.cutInfo;
            subObj = result.subObj;
            return OK;
        }

        if (result.cutInfo.yCoeffs.size() != zValues.size()) {
            std::cerr << "Integer L-shaped cut size mismatch" << std::endl;
            return ERROR;
        }

        subObj += result.subObj;
        cutInfo.constant += result.cutInfo.constant;
        for (size_t idx = 0; idx < zValues.size(); ++idx) {
            cutInfo.yCoeffs[idx] += result.cutInfo.yCoeffs[idx];
        }
    }

    return OK;
}

Status IntegerLShaped::ApplyWarmStart()
{
    double warmL = 0.0;
    std::vector<double> scenarioLs;
    std::vector<double> warmZ;
    bestUpperBound = 1e9;

    if constexpr (kWarmStartStrategy == WarmStartStrategy::ZeroLowerBound) {
        scenarioLs.assign(subProblems.size(), 0.0);
    } else {
        warmZ = dataIniter->BuildWarmStartMasterValues(*problemData);
        IntegerLShapedCutInfo warmCutInfo;

        const bool relaxedWarmStart = kWarmStartStrategy == WarmStartStrategy::RelaxedSubProblem;
        Status warmStatus = EvaluateSubProblems(warmZ, relaxedWarmStart,
            warmCutInfo, warmL, &scenarioLs);
        if (warmStatus != OK) {
            return warmStatus;
        }

        if constexpr (kWarmStartStrategy == WarmStartStrategy::IntegerSubProblem) {
            const bool warmMasterFeasible = dataIniter->IsWarmStartMasterFeasible(*problemData, warmZ, tolerance);
            if (warmMasterFeasible) {
                UpdateIncumbent(warmZ, warmL);
            }
        }
    }

    globalLowerBound = warmL;
    for (size_t idx = 0; idx < subProblems.size(); ++idx) {
        subProblems[idx]->strategy->SetLowerBound(scenarioLs[idx]);
    }

    return OK;
}

Status IntegerLShaped::Solve()
{
    algorithmStart = Tools::Clock::now();
    const auto solveStart = algorithmStart;

    Status warmStartStatus = ApplyWarmStart();
    if (warmStartStatus != OK) {
        return warmStartStatus;
    }
    std::cout << "Integer L-shaped warm start strategy=" << WarmStartStrategyName()
              << ", elapsed_ms=" << Tools::ElapsedMs(algorithmStart, Tools::Clock::now()) << std::endl;

    model->addConstr(theta >= globalLowerBound, "theta_global_lb");
    model->update();

    for (int iter = 0; iter < maxIters; ++iter) {
        const double elapsedSec = Tools::ElapsedMs(algorithmStart, Tools::Clock::now()) / 1000.0;
        if (elapsedSec >= solverConfig.timeLimit) {
            std::cout << "Integer L-shaped reached time limit: " << solverConfig.timeLimit << "s"
                      << ", elapsed_ms=" << Tools::ElapsedMs(algorithmStart, Tools::Clock::now()) << std::endl;
            return ERROR;
        }

        const auto masterStart = Tools::Clock::now();
        model->optimize();
        const auto masterEnd = Tools::Clock::now();
        const long long masterMs = Tools::ElapsedMs(masterStart, masterEnd);
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
        std::vector<double> scenarioRelaxedQValues;
        std::vector<IntegerLShapedCutInfo> scenarioContinuousCutInfos;
        const auto relaxedStart = Tools::Clock::now();
        Status relaxedStatus = ERROR;
        if constexpr (kRelaxedCutStrategy == RelaxedCutStrategy::MultiCut) {
            relaxedStatus = EvaluateSubProblems(zValues, true, continuousCutInfo, relaxedQValue,
                &scenarioRelaxedQValues, &scenarioContinuousCutInfos);
        } else {
            relaxedStatus = EvaluateSubProblems(zValues, true, continuousCutInfo, relaxedQValue);
        }
        const auto relaxedEnd = Tools::Clock::now();
        const long long relaxedMs = Tools::ElapsedMs(relaxedStart, relaxedEnd);
        if (relaxedStatus != OK) { // Error in solving relaxed sub-problem
            std::cerr << "Relaxed sub-problem failed in iteration " << iter << std::endl;
            return ERROR;
        }

        // Relaxed sub-problem infeasible, add feasibility cut.
        if (!continuousCutInfo.isOptimalityCut) {
            AddFeasibilityCut(*model, continuousCutInfo, zVars,
                "integer_l_shaped_relaxed_feasibility_" + std::to_string(iter));
            std::cout << "Integer L-shaped iter " << iter << ", feasibility cut from relaxation";
            if (std::isfinite(bestUpperBound)) {
                const double denom = std::max(1.0, std::fabs(bestUpperBound));
                std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound
                          << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
            }
            model->update();
            const auto iterEnd = Tools::Clock::now();
            std::cout << ", master_ms=" << masterMs
                      << ", relaxed_ms=" << relaxedMs
                      << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, iterEnd) << std::endl;
            continue;
        }
        // Relaxed sub-problem feasible and can be cut.
        if (std::isfinite(relaxedQValue)) {
            bool relaxedCutViolated = false;
            if constexpr (kRelaxedCutStrategy == RelaxedCutStrategy::MultiCut) {
                if (scenarioRelaxedQValues.size() != relaxedThetaVars.size() ||
                    scenarioContinuousCutInfos.size() != relaxedThetaVars.size()) {
                    throw std::runtime_error("Integer L-shaped relaxed multi-cut size mismatch");
                }
                for (size_t scenarioIdx = 0; scenarioIdx < scenarioContinuousCutInfos.size(); ++scenarioIdx) {
                    const double thetaScenarioValue = relaxedThetaVars[scenarioIdx].get(GRB_DoubleAttr_X);
                    if (thetaScenarioValue + tolerance >= scenarioRelaxedQValues[scenarioIdx]) {
                        continue;
                    }

                    const CutEval cutContinuous = BuildContinuousCut(scenarioContinuousCutInfos[scenarioIdx],
                        zValues, iter, static_cast<int>(scenarioIdx));
                    model->addConstr(cutContinuous.expr <= relaxedThetaVars[scenarioIdx], cutContinuous.name);
                    relaxedCutViolated = true;
                }
            } else if (thetaValue + tolerance < relaxedQValue) {
                const CutEval cutContinuous = BuildContinuousCut(continuousCutInfo, zValues, iter);
                model->addConstr(cutContinuous.expr <= theta, cutContinuous.name);
                relaxedCutViolated = true;
            }

            const double relaxedObj = fixedCost + relaxedQValue;
            const bool canImproveUb = relaxedObj + tolerance < bestUpperBound;
            const bool solveInteger = !relaxedCutViolated || (kPruneByUb && canImproveUb);

            if (relaxedCutViolated && !solveInteger) {
                std::cout << "Integer L-shaped iter " << iter << ", theta=" << thetaValue << ", Q_lp(z)=" << relaxedQValue;
                if (std::isfinite(bestUpperBound)) {
                    const double denom = std::max(1.0, std::fabs(bestUpperBound));
                    std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound
                            << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
                }

                model->update();
                const auto iterEnd = Tools::Clock::now();
                std::cout << ", cut from relaxation"
                          << ", master_ms=" << masterMs
                          << ", relaxed_ms=" << relaxedMs
                          << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, iterEnd) << std::endl;
                continue;
            }
        }

        double aggregatedQValue = 0.0;
        IntegerLShapedCutInfo integerCutInfo;
        const auto integerStart = Tools::Clock::now();
        Status secondStageStatus = EvaluateSubProblems(zValues, false, integerCutInfo, aggregatedQValue);
        const auto integerEnd = Tools::Clock::now();
        const long long integerMs = Tools::ElapsedMs(integerStart, integerEnd);
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

            model->update();
            const auto iterEnd = Tools::Clock::now();
            std::cout << ", master_ms=" << masterMs
                      << ", relaxed_ms=" << relaxedMs
                      << ", integer_ms=" << integerMs
                      << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, iterEnd) << std::endl;
            continue;
        }

        const double candidateObjective = fixedCost + aggregatedQValue;
        if (candidateObjective + tolerance < bestUpperBound) {
            UpdateIncumbent(zValues, aggregatedQValue);
        }

        double delta = std::max(0.0, aggregatedQValue - globalLowerBound);

        std::cout << "Integer L-shaped iter " << iter << ", theta=" << thetaValue << ", Q(z)=" << aggregatedQValue;
        if (std::isfinite(bestUpperBound)) {
            std::cout << ", UB=" << bestUpperBound << ", LB=" << bestLowerBound;
            double denom = std::max(1.0, std::fabs(bestUpperBound));
            std::cout << ", gap=" << (bestUpperBound - bestLowerBound) / denom * 100.0 << "%";
        }

        const double gapDenom = std::max(1.0, std::fabs(bestUpperBound));
        const double relGap = (bestUpperBound - bestLowerBound) / gapDenom;
        if (relGap <= solverConfig.mipGap) {
            const auto iterEnd = Tools::Clock::now();
            std::cout << ", master_ms=" << masterMs
                      << ", relaxed_ms=" << relaxedMs
                      << ", integer_ms=" << integerMs
                      << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, iterEnd) << std::endl;
            PrintBestSolution();
            std::cout << "Integer L-shaped converged." << std::endl;
            const auto solveEnd = Tools::Clock::now();
            std::cout << "Integer L-shaped final elapsed time: " << Tools::ElapsedMs(solveStart, solveEnd) << " ms" << std::endl;
            return OK;
        }

        // const CutEval cutImproved = BuildImprovedCut(zValues, zValues, delta, globalLowerBound, iter);
        // model->addConstr(cutImproved.expr <= theta, cutImproved.name);

        const CutEval cutPriority = BuildPriorityCut(zValues, zValues, delta, globalLowerBound, iter);
        model->addConstr(cutPriority.expr <= theta, cutPriority.name);

        // const CutEval cutClassic = BuildClassicCut(zValues, zValues, delta, globalLowerBound, iter);
        // model->addConstr(cutClassic.expr <= theta, cutClassic.name);

        model->update();
        const auto iterEnd = Tools::Clock::now();
        std::cout << ", master_ms=" << masterMs
                  << ", relaxed_ms=" << relaxedMs
                  << ", integer_ms=" << integerMs
                  << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, iterEnd) << std::endl;
    }

    const auto solveEnd = Tools::Clock::now();
    std::cout << "Integer L-shaped reached max iterations: " << maxIters
              << ", elapsed_ms=" << Tools::ElapsedMs(solveStart, solveEnd) << std::endl;
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

    std::cout << "Active master variables:" << std::endl;
    for (size_t idx = 0; idx < zVars.size(); ++idx) {
        if (incumbentZValues[idx] <= 0.5) {
            continue;
        }
        std::cout << zVars[idx].get(GRB_StringAttr_VarName) << std::endl;
    }
    std::cout << "Q(z): " << incumbentSecondStageValue << std::endl;
    std::cout << "objective: " << bestUpperBound << std::endl;
}

IntegerLShaped::~IntegerLShaped() = default;
