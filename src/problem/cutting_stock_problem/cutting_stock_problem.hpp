#pragma once

#include "column_generation.hpp"
#include "direct_solver.hpp"

class CuttingStockSubProblemStrategy : public ISubProblemStrategy {
public:
    void InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet) override;
    int GetCurrentStatus(const ProblemData& problemData, Pattern pattern) override;
    PatternWithInfo ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx) override;
    bool CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi) override;
};

class CuttingStockDataInitializationStrategy : public IDataInitializationStrategy {
public:
    void DataInit(ProblemData& problemData) override;
    std::vector<Constraint> ConstrInit(ProblemData& problemData) override;
};

class CuttingStockDirectSolverStrategy : public IDirectSolverStrategy {
public:
    void BuildModel(GRBSolver& solver, const ProblemData& data) override;
    void PrintSolution(GRBSolver& solver, const ProblemData& data) override;
private:
    std::vector<std::vector<GRBVar>> x;  // x[i][j]: items of type i from stock j
    std::vector<GRBVar> y;               // y[j]: 1 if stock j is used
};