#pragma once

#include "basic_solver.hpp"

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

class IDataInitializationStrategy {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual std::vector<Constraint> ConstrInit(ProblemData& problemData) = 0;
    virtual ~IDataInitializationStrategy() = default;
};

class ISubProblemStrategy {
public:
    virtual void InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet) = 0;
    virtual int GetCurrentStatus(const ProblemData& problemData, Pattern pattern) = 0;
    virtual PatternWithInfo ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx) = 0;
    virtual bool CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi) = 0;

    virtual ~ISubProblemStrategy() = default;
};

class MasterSolver final : public GRBSolver {
public:
    MasterSolver(GRBEnv& env);

    ~MasterSolver();
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

class ColumnGeneration {
public:
    ColumnGeneration(ProblemType problemType, int maxIters=100, double tol=1e-6);
    Status Run();
    ~ColumnGeneration();

private:
    ProblemType problemType;
    int maxIters;
    double tolerance;
    GRBEnv env;
    std::unique_ptr<MasterSolver> master;
    std::unique_ptr<SubSolver> sub;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy> dataIniter;

    GRBLinExpr obj = 0;

    Status Init();
    void UpdateMP();
    Status Solve();
};