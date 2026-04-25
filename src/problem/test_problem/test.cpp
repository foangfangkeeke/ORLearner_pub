#include <iostream>
#include <limits>

#include "test.hpp"
#include "basic_solver.hpp"
#include "tools.hpp"

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <filesystem>

using namespace std;

TestDataInitializationStrategy_Solver::TestDataInitializationStrategy_Solver(std::string dataFolder)
    : dataFolder(dataFolder)
{}

Status TestDataInitializationStrategy_Solver::DataInit(GRBModel& model)
{
    const std::filesystem::path dataPath =
        std::filesystem::path(__FILE__).parent_path() / dataFolder / "test_input.txt";
    std::ifstream stream(dataPath);
    std::string line;

    if (!Tools::ReadValidLine(stream, line)) {
        throw std::runtime_error("Failed to load test data from file.");
    }
    int numVar = std::stoi(line);

    vector<GRBVar> grbVars;
    grbVars.reserve(static_cast<size_t>(numVar));
    GRBLinExpr objective = 0;

    for (int i = 0; i < numVar; ++i) {
        if (!Tools::ReadValidLine(stream, line)) {
            throw std::runtime_error("Failed to load test variable data.");
        }
        auto parts = Tools::SplitAndTrim(line, ',');
        double lb = std::stod(parts[0]);
        double ub = (parts[1] == "inf" || parts[1] == "INF") ?
            std::numeric_limits<double>::infinity() : std::stod(parts[1]);
        double obj = std::stod(parts[2]);
        char type = (parts[3] == "GRB_INTEGER") ? GRB_INTEGER : GRB_CONTINUOUS;
        GRBVar var = model.addVar(lb, ub, obj, type, "var_" + std::to_string(i));
        grbVars.push_back(var);
        objective += obj * var;
    }

    if (!Tools::ReadValidLine(stream, line)) {
        throw std::runtime_error("Failed to load test constraint count.");
    }
    int numConstr = std::stoi(line);

    for (int i = 0; i < numConstr; ++i) {
        if (!Tools::ReadValidLine(stream, line)) {
            throw std::runtime_error("Failed to load test constraint data.");
        }
        auto parts = Tools::SplitAndTrim(line, ',');
        GRBLinExpr lhs = 0;
        for (size_t i = 0; i < grbVars.size(); ++i) {
            lhs += std::stod(parts[i]) * grbVars[i];
        }
        char sense = parts[grbVars.size()][0];
        double rhs = std::stod(parts[grbVars.size() + 1]);
        model.addConstr(lhs, sense, rhs, "constr_" + std::to_string(i));
    }

    if (!Tools::ReadValidLine(stream, line)) {
        throw std::runtime_error("Failed to load test objective sense.");
    }
    int objSense = (line == "min") ? GRB_MINIMIZE : GRB_MAXIMIZE;
    model.setObjective(objective, objSense);
    return OK;
}