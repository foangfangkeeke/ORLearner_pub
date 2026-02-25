#include "column_generation.hpp"
#include "cutting_stock_problem.hpp"

#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <map>

static const std::map<ProblemType,
                      std::tuple<std::function<std::unique_ptr<IDataInitializationStrategy>()>,
                                 std::function<std::unique_ptr<ISubProblemStrategy>()>>> strategyMap = {
    {
        CUTTINGSTOCK,
        std::make_tuple(
            [](){return std::make_unique<CuttingStockDataInitializationStrategy>();},
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

MasterSolver::MasterSolver(GRBEnv& env) : GRBSolver(env)
{
    std::cout << "===== build MasterSolver =====" << std::endl;
}

MasterSolver::~MasterSolver() {
    std::cout << "===== destroy MasterSolver =====" << std::endl;
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
    std::cout << "===== build SubSolver =====" << std::endl;
}

SubSolver::~SubSolver() {
    std::cout << "===== destroy SubSolver =====" << std::endl;
}

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

Status ColumnGeneration::Init()
{
    std::cout << "===== init CG =====" << std::endl;
    std::cout << "problemType=" << problemType << std::endl;
    std::cout << "maxIters=" << maxIters << std::endl;
    std::cout << "tolerance=" << tolerance << std::endl;

    problemData = std::make_unique<ProblemData>();

    GRBModel model(env);
    master = std::make_unique<MasterSolver>(env);

    auto it = strategyMap.find(problemType);
    if (it == strategyMap.end()) {
        throw std::invalid_argument("Unsupported problem type");
    }
    const auto& strategies = it->second;

    dataIniter = std::get<0x0>(strategies)();
    sub = std::make_unique<SubSolver>(std::get<0x1>(strategies)());

    dataIniter->DataInit(*problemData);
    std::vector<Constraint> constrs = dataIniter->ConstrInit(*problemData);
    sub->InitPatterns(*problemData);
    for (size_t i = 0; i < constrs.size(); ++i) {
        master->AddConstraint(GRBLinExpr(), std::get<0x2>(constrs[i]), std::get<0x1>(constrs[i]), std::get<0x0>(constrs[i]));
    }
    master->Update();

    return OK;
}

void ColumnGeneration::UpdateMP()
{
    std::cout << "===== start UpdatePatterns =====" << std::endl;

    std::vector<PatternWithInfo> newPatterns;
    sub->GetNewPatternWithInfos(newPatterns);

    auto baseOffset = sub->GetPatternWithInfos().size()-newPatterns.size();
    std::cout << "newPatterns.size() in UpdatePatterns: " << newPatterns.size() << std::endl;
    for (size_t i = 0; i < newPatterns.size(); i++) {
        PatternUtils::print(newPatterns[i].pattern);
        auto& var = master->AddVariable(newPatterns[i].lb, newPatterns[i].ub, newPatterns[i].coeff, GRB_CONTINUOUS, "x" + std::to_string(baseOffset + i));
        std::cout << "x" + std::to_string(baseOffset + i) + ":[" << newPatterns[i].lb << ", " << newPatterns[i].ub << "], coef: " << newPatterns[i].coeff << std::endl;
        master->AddPattern(newPatterns[i].pattern, var);
        obj = obj + newPatterns[i].coeff * var;
    }

    master->SetObjective(obj, GRB_MINIMIZE);
    master->Update();

    master->printObjective();
}

Status ColumnGeneration::Solve()
{
    std::cout << "===== start CG =====" << std::endl;
    int iter = 0;

    while (iter <= maxIters) {
        std::cout << "===== CG iter " << iter << " =====" << std::endl;

        UpdateMP();
        bool status = master->Solve(); // RLMP
        if (!status) {
            return ERROR;
        }

        std::vector<double> duals;
        for (size_t i = 0; i < master->GetConstraintsNum(); ++i) {
            duals.push_back(master->GetDual(i));
        }

        bool needUpdate = sub->FindNewPatterns(*problemData, duals);
        if (!needUpdate) {
            return OK;
        }

        iter++;
    }
    return OK;
}

Status ColumnGeneration::Run()
{
    Status status = Init();
    if (status) {
        return status;
    }

    return Solve();
}

ColumnGeneration::ColumnGeneration(ProblemType problemType, int maxIters, double tol): problemType(problemType), maxIters(maxIters), tolerance(tol)
{
    std::cout << "===== build CG =====" << std::endl;
};

ColumnGeneration::~ColumnGeneration()
{
    std::cout << "===== destroy CG =====" << std::endl;
};