#include "cutting_stock_problem.hpp"

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
bool LoadCuttingStockData(int& stockLength, std::vector<int>& itemLengths, std::vector<int>& demands)
{
    const std::filesystem::path dataPath = std::filesystem::path(__FILE__).parent_path() / "cutting_stock_input.txt";
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
    int stockLength{};
    std::vector<int> itemLengths;
    std::vector<int> demands;

    if (!LoadCuttingStockData(stockLength, itemLengths, demands)) {
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

std::vector<Constraint> CuttingStockDataInitializationStrategy::ConstrInit(ProblemData& problemData)
{
    std::vector<Constraint> constrs;
    auto demands = problemData.getData<std::vector<int>>("demands");
    for (size_t i = 0; i < demands.size(); ++i) {
        constrs.push_back(std::make_tuple("demand" + std::to_string(i), demands[i], '>='));
    }

    return constrs;
}

void CuttingStockDirectSolverStrategy::BuildModel(GRBSolver& solver, const ProblemData& data)
{
    auto itemLengths = data.getData<std::vector<int>>("itemLengths");
    auto demands     = data.getData<std::vector<int>>("demands");
    int  stockLength = data.getData<int>("stockLength");

    int numItems = static_cast<int>(itemLengths.size());
    // Upper bound on number of stocks: sum of all demands (trivial bound)
    int N = 0;
    for (int d : demands) N += d;

    std::cout << "DirectSolver: numItems=" << numItems << ", N=" << N << ", stockLength=" << stockLength << std::endl;

    // y[j]: binary, 1 if stock j is used
    y.resize(N);
    GRBLinExpr obj = 0;
    for (int j = 0; j < N; ++j) {
        y[j] = solver.AddVariable(0.0, 1.0, 1.0, GRB_BINARY, "y" + std::to_string(j));
        obj += y[j];
    }
    solver.SetObjective(obj, GRB_MINIMIZE);

    // x[i][j]: integer, number of items of type i cut from stock j
    x.resize(numItems, std::vector<GRBVar>(N));
    for (int i = 0; i < numItems; ++i) {
        for (int j = 0; j < N; ++j) {
            x[i][j] = solver.AddVariable(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,
                                          "x_" + std::to_string(i) + "_" + std::to_string(j));
        }
    }

    // Demand constraints: sum_j x[i][j] >= demands[i]
    for (int i = 0; i < numItems; ++i) {
        GRBLinExpr lhs = 0;
        for (int j = 0; j < N; ++j) {
            lhs += x[i][j];
        }
        solver.AddConstraint(lhs, '>', static_cast<double>(demands[i]),
                             "demand_" + std::to_string(i));
    }

    // Capacity constraints: sum_i itemLengths[i]*x[i][j] <= stockLength*y[j]
    for (int j = 0; j < N; ++j) {
        GRBLinExpr lhs = 0;
        for (int i = 0; i < numItems; ++i) {
            lhs += itemLengths[i] * x[i][j];
        }
        GRBLinExpr rhs = static_cast<double>(stockLength) * y[j];
        solver.AddConstraint(lhs - rhs, '<', 0.0, "capacity_" + std::to_string(j));
    }
}

void CuttingStockDirectSolverStrategy::PrintSolution(GRBSolver& solver, const ProblemData& data)
{
    auto itemLengths = data.getData<std::vector<int>>("itemLengths");
    int numItems = static_cast<int>(itemLengths.size());
    int N = static_cast<int>(y.size());

    for (int j = 0; j < N; ++j) {
        double yVal = y[j].get(GRB_DoubleAttr_X);
        if (yVal > 0.5) {
            std::cout << "Stock " << j << " used:";
            for (int i = 0; i < numItems; ++i) {
                double xVal = x[i][j].get(GRB_DoubleAttr_X);
                if (xVal > 0.5) {
                    std::cout << "  item[" << i << "](len=" << itemLengths[i] << ") x" << static_cast<int>(std::round(xVal));
                }
            }
            std::cout << std::endl;
        }
    }
}