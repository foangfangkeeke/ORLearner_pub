#include "cutting_stock_problem.hpp"
#include "basic_solver.hpp"
#include "tools.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {
bool LoadCuttingStockData(const std::string& dataFolder, int& stockLength, std::vector<int>& itemLengths, std::vector<int>& demands)
{
    const std::filesystem::path dataPath =
        std::filesystem::path(__FILE__).parent_path() / dataFolder / "cutting_stock_input.txt";
    std::ifstream fin(dataPath);
    if (!fin.is_open()) {
        std::cerr << "Warning: failed to open " << dataPath << ", using defaults." << std::endl;
        return false;
    }

    auto parseLine = [](const std::string& line) {
        std::vector<int> values;
        std::istringstream iss(line);
        int v;
        while (iss >> v) {
            values.push_back(v);
        }
        return values;
    };

    std::string line;
    if (!std::getline(fin, line)) {
        std::cerr << "Warning: missing stockLength line in " << dataPath << ", using defaults." << std::endl;
        return false;
    }

    try {
        stockLength = std::stoi(line);
    } catch (const std::exception& e) {
        std::cerr << "Warning: invalid stockLength in " << dataPath << " - " << e.what() << ", using defaults." << std::endl;
        return false;
    }

    if (!std::getline(fin, line)) {
        std::cerr << "Warning: missing itemLengths line in " << dataPath << ", using defaults." << std::endl;
        return false;
    }
    itemLengths = parseLine(line);

    if (!std::getline(fin, line)) {
        std::cerr << "Warning: missing demands line in " << dataPath << ", using defaults." << std::endl;
        return false;
    }
    demands = parseLine(line);

    if (itemLengths.empty() || demands.empty() || itemLengths.size() != demands.size()) {
        std::cerr << "Warning: itemLengths/demands mismatch in " << dataPath << ", using defaults." << std::endl;
        return false;
    }

    return true;
}
} // namespace

CuttingStockDataInitializationStrategy_CG::CuttingStockDataInitializationStrategy_CG(std::string dataFolder)
    : dataFolder(dataFolder)
{}

CuttingStockDataInitializationStrategy_Solver::CuttingStockDataInitializationStrategy_Solver(std::string dataFolder)
    : dataFolder(dataFolder)
{}

void CuttingStockSubProblemStrategy::InitPatterns(const ProblemData& problemData, std::queue<PatternWithInfo>& bfsQueue,
        std::set<PatternWithInfo>& processedPatterns, std::vector<PatternWithInfo>& paretoSet)
{
    bfsQueue = std::queue<PatternWithInfo>();
    processedPatterns.clear();
    paretoSet.clear();

    auto itemLengths = problemData.getData<std::vector<int>>("itemLengths");
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

void CuttingStockDataInitializationStrategy_CG::DataInit(ProblemData& problemData)
{
    int stockLength;
    std::vector<int> itemLengths;
    std::vector<int> demands;

    if (!LoadCuttingStockData(dataFolder, stockLength, itemLengths, demands)) {
        throw std::runtime_error("Failed to load cutting stock data from file.");
    }

    std::cout << "Loaded cutting stock data from file." << std::endl;

    problemData.addData("stockLength", stockLength);
    problemData.addData("itemLengths", itemLengths);
    problemData.addData("demands", demands);

    std::cout << "stockLength=" << stockLength << std::endl;
    std::cout << "demands:" << std::endl;

    for (size_t i = 0; i < itemLengths.size(); ++i) {
        std::cout << "  Length " << itemLengths[i] << ": " << demands[i] << std::endl;
    }
}

std::vector<Constraint> CuttingStockDataInitializationStrategy_CG::ConstrInit(ProblemData& problemData)
{
    std::vector<Constraint> constrs;
    auto demands = problemData.getData<std::vector<int>>("demands");
    for (size_t i = 0; i < demands.size(); ++i) {
        constrs.push_back(std::make_tuple("demand" + std::to_string(i), demands[i], '>'));
    }

    return constrs;
}

std::vector<std::vector<int>> findAllCombinations(int stockLength, const std::vector<int>& itemLengths) {
    std::vector<std::vector<int>> result;
    std::vector<int> current;
    current.resize(itemLengths.size(), 0);

    auto backtrack = [&](auto&& self, int index, int remaining) {
        if (index == itemLengths.size()) {
            if (remaining >= 0) {
                result.push_back(current);
            }
            return;
        }

        int itemLen = itemLengths[index];
        int maxCount = remaining / itemLen;

        for (int count = 0; count <= maxCount; ++count) {
            current[index] = count;
            self(self, index + 1, remaining - count * itemLen);
        }
    };

    backtrack(backtrack, 0, stockLength);
    return result;
}

Status CuttingStockDataInitializationStrategy_Solver::DataInit(GRBModel& model)
{
    int stockLength;
    std::vector<int> itemLengths;
    std::vector<int> demands;

    if (!LoadCuttingStockData(dataFolder, stockLength, itemLengths, demands)) {
        throw std::runtime_error("Failed to load cutting stock data from file.");
    }

    std::cout << "Loaded cutting stock data from file." << std::endl;

    auto allPatterns = findAllCombinations(stockLength, itemLengths);
    std::cout << "Generated " << allPatterns.size() << " cutting patterns." << std::endl;

    std::cout << "stockLength=" << stockLength << std::endl;
    std::cout << "demands:" << std::endl;

    for (size_t i = 0; i < itemLengths.size(); ++i) {
        std::cout << "  Length " << itemLengths[i] << ": " << demands[i] << std::endl;
    }

    std::vector<GRBVar> patternVars;
    patternVars.reserve(allPatterns.size());
    GRBLinExpr obj = 0;

    for (size_t i = 0; i < allPatterns.size(); ++i) {
        GRBVar var = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_INTEGER,
            "x" + PatternToString(allPatterns[i]));
        patternVars.push_back(var);
        obj += var;
    }

    for (size_t i = 0; i < demands.size(); ++i) {
        GRBLinExpr lhs = 0;
        for (size_t j = 0; j < allPatterns.size(); ++j) {
            lhs += allPatterns[j][i] * patternVars[j];
        }
        model.addConstr(lhs >= demands[i], "demand_" + std::to_string(i));
    }

    model.setObjective(obj, GRB_MINIMIZE);
    return OK;
}