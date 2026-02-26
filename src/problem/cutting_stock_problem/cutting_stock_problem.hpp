#pragma once

#include "column_generation.hpp"
#include "using_solver.hpp"

class CuttingStockSubProblemStrategy : public ISubProblemStrategy {
public:
    void InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet) override;
    int GetCurrentStatus(const ProblemData& problemData, Pattern pattern) override;
    PatternWithInfo ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx) override;
    bool CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi) override;
};

class CuttingStockDataInitializationStrategy_CG : public IDataInitializationStrategy_CG {
public:
    void DataInit(ProblemData& problemData) override;
    std::vector<Constraint> ConstrInit(ProblemData& problemData) override;
};

class CuttingStockDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    void DataInit(ProblemData& problemData) override;
};