#include "cutting_stock_problem.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>

void CuttingStockSubProblemStrategy::InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet)
{
    bfsQueue = std::queue<PatternWithInfo>();
    processedPatterns.clear();
    paretoSet.clear();

    auto itemLengths = problemData.getData<std::vector<int>>("itemLengths");
    std::cout<<"itemLengths: "<<std::endl;
    for (int i = 0; i < itemLengths.size(); i++) {
        PatternWithInfo term = {std::vector<int>(itemLengths.size()), 0, 0, 0};
        term.pattern[i] = 1;
        term.coeff = 1;
        term.lb = 0;
        term.ub = GRB_INFINITY;
        term.isNew = true;
        paretoSet.push_back(term);
        bfsQueue.push(term);
        processedPatterns.insert(term);
    }

    std::cout << "finish InitPatterns, " << paretoSet.size() << " patterns were added" << std::endl;
}

int CuttingStockSubProblemStrategy::GetCurrentStatus(const ProblemData& problemData, Pattern pattern)
{
    int currentLength = 0;
    auto itemLengths = problemData.getData<std::vector<int>>("itemLengths");
    for (size_t i = 0; i < pattern.size(); ++i) {
        currentLength += pattern[i] * itemLengths[i];
    }
    return currentLength;
}

PatternWithInfo CuttingStockSubProblemStrategy::ExpandPattern(const ProblemData& problemData, PatternWithInfo pwi, int idx)
{
    pwi.pattern[idx] += 1;
    return pwi;
}

bool CuttingStockSubProblemStrategy::CheckValid(const ProblemData& problemData, const PatternWithInfo& pwi)
{
    int length = GetCurrentStatus(problemData, pwi.pattern);
    return length <= problemData.getData<int>("stockLength");
}

void CuttingStockDataInitializationStrategy::DataInit(ProblemData& problemData)
{
    problemData.addData("stockLength", 16);
    problemData.addData("itemLengths", std::vector<int>{3, 7, 12});
    problemData.addData("demands", std::vector<int>{15, 20, 10});

    auto stockLength = problemData.getData<int>("stockLength");
    auto itemLengths = problemData.getData<std::vector<int>>("itemLengths");
    auto demands = problemData.getData<std::vector<int>>("demands");

    std::cout << "stockLength=" << stockLength << std::endl;
    std::cout << "demands:" << std::endl;

    for (size_t i = 0; i < itemLengths.size(); ++i) {
        std::cout << "  Length " << itemLengths[i] << ": " << demands[i] << std::endl;
    }
}

std::vector<Constraint> CuttingStockDataInitializationStrategy::ConstrInit(ProblemData& problemData)
{
    std::vector<Constraint> constrs;
    auto demands = problemData.getData<std::vector<int>>("demands");
    for (size_t i = 0; i < demands.size(); ++i) {
        constrs.push_back(std::make_tuple("demand" + std::to_string(i), demands[i], '>='));
    }

    return constrs;
}