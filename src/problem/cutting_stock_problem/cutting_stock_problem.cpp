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

MILPModel CuttingStockMILPFormulator::Formulate()
{
    int stockLength{};
    std::vector<int> itemLengths;
    std::vector<int> demands;

    if (!LoadCuttingStockData(stockLength, itemLengths, demands)) {
        throw std::runtime_error("Failed to load cutting stock data from file.");
    }

    std::cout << "Loaded cutting stock data from file." << std::endl;
    std::cout << "stockLength=" << stockLength << std::endl;
    std::cout << "demands:" << std::endl;
    int numItems = static_cast<int>(itemLengths.size());
    for (int i = 0; i < numItems; ++i) {
        std::cout << "  Length " << itemLengths[i] << ": " << demands[i] << std::endl;
    }

    // Upper bound on number of stocks needed (trivial: sum of all demands)
    int N = 0;
    for (int d : demands) N += d;

    std::cout << "DirectSolver: numItems=" << numItems << ", N=" << N
              << ", stockLength=" << stockLength << std::endl;

    MILPModel model;
    model.objSense = GRB_MINIMIZE;

    // y[j]: binary — 1 if stock j is used (objective coefficient = 1)
    for (int j = 0; j < N; ++j) {
        model.variables.push_back({"y" + std::to_string(j), 0.0, 1.0, GRB_BINARY, 1.0});
    }

    // x[i][j]: integer — items of type i cut from stock j (no obj coeff)
    for (int i = 0; i < numItems; ++i) {
        for (int j = 0; j < N; ++j) {
            model.variables.push_back(
                {"x_" + std::to_string(i) + "_" + std::to_string(j),
                 0.0, GRB_INFINITY, GRB_INTEGER, 0.0});
        }
    }

    // Demand constraints: sum_j x[i][j] >= demands[i]
    for (int i = 0; i < numItems; ++i) {
        ConstrDef cd;
        cd.name  = "demand_" + std::to_string(i);
        cd.sense = '>';
        cd.rhs   = static_cast<double>(demands[i]);
        for (int j = 0; j < N; ++j) {
            cd.terms.push_back({"x_" + std::to_string(i) + "_" + std::to_string(j), 1.0});
        }
        model.constraints.push_back(std::move(cd));
    }

    // Capacity constraints: sum_i itemLengths[i]*x[i][j] - stockLength*y[j] <= 0
    for (int j = 0; j < N; ++j) {
        ConstrDef cd;
        cd.name  = "capacity_" + std::to_string(j);
        cd.sense = '<';
        cd.rhs   = 0.0;
        for (int i = 0; i < numItems; ++i) {
            cd.terms.push_back(
                {"x_" + std::to_string(i) + "_" + std::to_string(j),
                 static_cast<double>(itemLengths[i])});
        }
        cd.terms.push_back({"y" + std::to_string(j),
                            -static_cast<double>(stockLength)});
        model.constraints.push_back(std::move(cd));
    }

    return model;
}

void CuttingStockMILPFormulator::PrintSolution(const MILPModel& model,
                                               const std::vector<double>& varValues)
{
    // Variables are laid out as: y0…y(N-1), then x_0_0…x_(numItems-1)_(N-1)
    int N = 0;
    for (const auto& vd : model.variables) {
        if (vd.name[0] == 'y') ++N; else break;
    }
    int numItems = (N > 0)
        ? static_cast<int>((model.variables.size() - N) / N)
        : 0;

    for (int j = 0; j < N; ++j) {
        double yVal = varValues[j];
        if (yVal > 0.5) {
            std::cout << "Stock " << j << " used:";
            for (int i = 0; i < numItems; ++i) {
                // x[i][j] is at index N + i*N + j
                double xVal = varValues[N + i * N + j];
                if (xVal > 0.5) {
                    std::cout << "  item[" << i << "] x"
                              << static_cast<int>(std::round(xVal));
                }
            }
            std::cout << std::endl;
        }
    }
}