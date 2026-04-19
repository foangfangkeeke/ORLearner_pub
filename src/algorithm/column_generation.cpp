#include "column_generation.hpp"
#include "cutting_stock_problem.hpp"
#include "tools.hpp"

#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <map>

static const std::map<ProblemType,
                      std::tuple<std::function<std::unique_ptr<IDataInitializationStrategy_CG>(const std::string&)>,
                                 std::function<std::unique_ptr<ISubProblemStrategy>()>>> strategyMap = {
    {
        CUTTINGSTOCK,
        std::make_tuple(
            [](const std::string& dataFolder){return std::make_unique<CuttingStockDataInitializationStrategy_CG>(dataFolder);},
            [](){return std::make_unique<CuttingStockSubProblemStrategy>();})
    }
};

void PatternUtils::print(const Pattern& pattern)
{
    for (size_t i = 0; i < pattern.size(); ++i) {
        std::cout << pattern[i] << ",";
    }
    std::cout << std::endl;
}

bool PatternUtils::IsDominated(const Pattern& a, const Pattern& b)
{
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] > b[i]) return false;
    }
    return true;
}

double PatternUtils::CalculateReducedCost(const PatternWithInfo& term, const std::vector<double>& duals)
{
    double sum = 0.0;
    for (size_t i = 0; i < term.pattern.size(); ++i) {
        sum += term.pattern[i] * duals[i];
    }
    return term.coeff - sum;
}

bool SubSolver::IsParetoImprovement(const PatternWithInfo& newPWI) {
    const Pattern& newPattern = newPWI.pattern;

    for (const auto& pwi : paretoSet) {
        if (PatternUtils::IsDominated(newPattern, pwi.pattern)) {
            return false;
        }
    }

    return true;
}

void SubSolver::GetNewPatternWithInfos(std::vector<PatternWithInfo>& newPatterns) {
    for (auto& pwi : paretoSet) {
        if (pwi.isNew) {
            newPatterns.push_back(pwi);
            pwi.isNew = false;
        }
    }
}

bool SubSolver::FindNewPatterns(const ProblemData& problemData, std::vector<double>& duals)
{
    bool needUpdate = false;
    const double EPS = 1e-9;
    int currentQueueSize = bfsQueue.size();
    auto itemLengths = problemData.getData<std::vector<int>>("itemLengths");
    for (int i = 0; i < currentQueueSize && !bfsQueue.empty(); ++i) {
        PatternWithInfo current = bfsQueue.front();
        bfsQueue.pop();

        for (size_t j = 0; j < itemLengths.size(); ++j) {
            PatternWithInfo newPWI = ExpandPattern(problemData, current, j);
            if (!CheckValid(problemData, newPWI)) continue;

            if (PatternUtils::CalculateReducedCost(newPWI, duals) >= -EPS) {
                continue;
            }

            if (!IsParetoImprovement(newPWI)) {
                continue;
            }

            bfsQueue.push(newPWI);
            processedPatterns.insert(newPWI);
            paretoSet.push_back(newPWI);
            needUpdate = true;
        }
    }
    return needUpdate;
}

SubSolver::SubSolver(std::unique_ptr<ISubProblemStrategy> strategy) : strategy(std::move(strategy))
{
    if (!this->strategy) {
        throw std::invalid_argument("Strategy cannot be null");
    }
}

SubSolver::~SubSolver() {};

std::vector<PatternWithInfo> SubSolver::GetPatternWithInfos() const
{
    return paretoSet;
}

void SubSolver::InitPatterns(const ProblemData& problemData)
{
    return strategy->InitPatterns(problemData, bfsQueue, processedPatterns, paretoSet);
}

int SubSolver::GetCurrentStatus(const ProblemData& problemData, Pattern pattern)
{
    return strategy->GetCurrentStatus(problemData, pattern);
}

PatternWithInfo SubSolver::ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx)
{
    return strategy->ExpandPattern(problemData, pwi, idx);
}

bool SubSolver::CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi)
{
    return strategy->CheckValid(problemData, pwi);
}

void ColumnGeneration::AddPattern(const std::vector<int>& pattern, GRBVar var) {
    GRBConstr* constrs = model->getConstrs();
    int numConstrs = model->get(GRB_IntAttr_NumConstrs);

    if (pattern.size() != static_cast<size_t>(numConstrs)) {
        throw GRBException("Pattern size does not match number of constraints", ERROR);
    }

    for (int i = 0; i < numConstrs; ++i) {
        model->chgCoeff(constrs[i], var, pattern[i]);
    }
}

Status ColumnGeneration::Initialize()
{
    std::cout << "problemType=" << problemType << std::endl;
    std::cout << "maxIters=" << maxIters << std::endl;
    std::cout << "tolerance=" << tolerance << std::endl;

    env = std::make_unique<GRBEnv>(true);
    env->set("LogFile", "gurobi_log.txt");
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    model = std::make_unique<GRBModel>(*env);

    problemData = std::make_unique<ProblemData>();

    auto it = strategyMap.find(problemType);
    if (it == strategyMap.end()) {
        throw std::invalid_argument("Unsupported problem type");
    }
    const auto& strategies = it->second;

    dataIniter = std::get<0x0>(strategies)(dataFolder);
    sub = std::make_unique<SubSolver>(std::get<0x1>(strategies)());

    dataIniter->DataInit(*problemData);
    std::vector<Constraint> constrs = dataIniter->ConstrInit(*problemData);
    sub->InitPatterns(*problemData);
    for (size_t i = 0; i < constrs.size(); ++i) {
        model->addConstr(GRBLinExpr(), std::get<0x2>(constrs[i]), std::get<0x1>(constrs[i]), std::get<0x0>(constrs[i]));
    }
    model->update();
    initialized = true;

    return OK;
}

void ColumnGeneration::UpdateMP()
{
    std::vector<PatternWithInfo> newPatterns;
    sub->GetNewPatternWithInfos(newPatterns);

    auto baseOffset = sub->GetPatternWithInfos().size()-newPatterns.size();
    std::cout << "newPatterns.size() in UpdatePatterns: " << newPatterns.size() << std::endl;
    for (size_t i = 0; i < newPatterns.size(); i++) {
        GRBVar var = model->addVar(newPatterns[i].lb, newPatterns[i].ub, newPatterns[i].coeff, GRB_CONTINUOUS, "x" + PatternToString(newPatterns[i].pattern));
        AddPattern(newPatterns[i].pattern, var);
        obj = obj + newPatterns[i].coeff * var;
    }

    model->setObjective(obj, GRB_MINIMIZE);
    model->update();
}

Status ColumnGeneration::Solve()
{
    int iter = 0;

    while (iter <= maxIters) {
        UpdateMP();
        model->optimize(); // RLMP
        int solveStatus = model->get(GRB_IntAttr_Status);
        if (solveStatus == GRB_OPTIMAL || (solveStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount)>0)) {
            std::cout << "Current objective: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
        } else {
            std::cerr << "MP not solved to optimality, status: " << solveStatus << std::endl;
            Debug::OutputModel(model);
            return ERROR;
        }

        std::vector<double> duals;
        GRBConstr* constrs = model->getConstrs();
        for (size_t i = 0; i < static_cast<size_t>(model->get(GRB_IntAttr_NumConstrs)); ++i) {
            duals.push_back(constrs[i].get(GRB_DoubleAttr_Pi));
        }

        bool needUpdate = sub->FindNewPatterns(*problemData, duals);
        if (!needUpdate) {
            Debug::OutputResult(model);
            return OK;
        }

        iter++;
    }
    return OK;
}

ColumnGeneration::ColumnGeneration(ProblemType problemType, std::string dataFolder, int maxIters, double tol)
    : problemType(problemType), dataFolder(dataFolder), maxIters(maxIters), tolerance(tol)
{};

ColumnGeneration::~ColumnGeneration() {};