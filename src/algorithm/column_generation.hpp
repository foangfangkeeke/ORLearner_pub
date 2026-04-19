#pragma once

#include "basic_solver.hpp"
#include "milp_solver.hpp"

#include <vector>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <iostream>

typedef std::vector<int> Pattern;
struct PatternWithInfo {
    Pattern pattern;
    double coeff;
    double lb;
    double ub;
    bool isNew;

    bool operator<(const PatternWithInfo& other) const {
        return pattern < other.pattern;
    }
};

class IDataInitializationStrategy_CG {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual std::vector<Constraint> ConstrInit(ProblemData& problemData) = 0;
    virtual ~IDataInitializationStrategy_CG() = default;
};

class ISubProblemStrategy {
public:
    virtual void InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet) = 0;
    virtual int GetCurrentStatus(const ProblemData& problemData, Pattern pattern) = 0;
    virtual PatternWithInfo ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx) = 0;
    virtual bool CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi) = 0;

    virtual std::unique_ptr<ISubProblemStrategy> Clone() const = 0;
    virtual ~ISubProblemStrategy() = default;
};

class SubSolver {
public:
    explicit SubSolver(std::unique_ptr<ISubProblemStrategy> strategy);
    std::vector<PatternWithInfo> GetPatternWithInfos() const;
    void GetNewPatternWithInfos(std::vector<PatternWithInfo>& newPatterns);
    void InitPatterns(const ProblemData& problemData);
    bool FindNewPatterns(const ProblemData& problemData, std::vector<double>& duals);
    int GetCurrentStatus(const ProblemData& problemData, Pattern pattern);
    PatternWithInfo ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx);
    bool CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi);

    ~SubSolver();

    SubSolver(const SubSolver& other) {
        this->bfsQueue = other.bfsQueue;
        this->processedPatterns = other.processedPatterns;
        this->paretoSet = other.paretoSet;
        this->strategy = other.strategy->Clone(); // 需给ISubProblemStrategy新增Clone接口
    }

private:
    std::queue<PatternWithInfo> bfsQueue;
    std::set<PatternWithInfo> processedPatterns;
    std::vector<PatternWithInfo> paretoSet;
    std::unique_ptr<ISubProblemStrategy> strategy;

    bool IsParetoImprovement(const PatternWithInfo& newPWI);
};


class PatternUtils {
public:
    PatternUtils()=delete;
    static void print(const Pattern& pattern);
    static double CalculateReducedCost(const PatternWithInfo& pattern, const std::vector<double>& duals);
    static bool IsDominated(const Pattern& a, const Pattern& b);
    ~PatternUtils()=delete;
};

class ColumnGeneration : public IMILPAlgorithmStrategy {
public:
    ColumnGeneration(ProblemType problemType, std::string dataFolder = "test_data", int maxIters=100, double tol=1e-6);
    Status Initialize();
    Status Solve();
    std::shared_ptr<GRBModel> GetModel() const { return model; }
    ~ColumnGeneration();

    ColumnGeneration(const ColumnGeneration& other) {
        this->problemType = other.problemType;
        this->dataFolder = other.dataFolder;
        this->maxIters = other.maxIters;
        this->tolerance = other.tolerance;
        this->initialized = other.initialized;

        this->env = std::make_unique<GRBEnv>(*other.env);
        this->model = std::make_unique<GRBModel>(*other.model);
        this->obj = GRBLinExpr(other.obj);
        this->problemData = std::make_unique<ProblemData>(*other.problemData);

        this->sub = std::make_unique<SubSolver>(*other.sub);
    }

    std::unique_ptr<ColumnGeneration> Clone() const {
        return std::make_unique<ColumnGeneration>(*this);
    }

private:
    ProblemType problemType;
    std::string dataFolder;
    int maxIters;
    double tolerance;

    std::unique_ptr<GRBEnv> env;
    std::shared_ptr<GRBModel> model;
    std::unique_ptr<SubSolver> sub;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy_CG> dataIniter;
    bool initialized = false;

    GRBLinExpr obj = 0;

    void UpdateMP();

    void AddPattern(const std::vector<int>& pattern, GRBVar var);
};