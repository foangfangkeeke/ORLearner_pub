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

namespace {
bool LoadTestData(vector<ProblemDataVar>& vars, vector<ProblemDataConstr>& constrs, int& obj)
{
    vars.clear();
    constrs.clear();
    obj = 0;

    try {
        const std::filesystem::path dataPath = std::filesystem::path(__FILE__).parent_path() / "test_input.txt";
        std::ifstream stream(dataPath);
        std::string line;

        if (!Tools::ReadValidLine(stream, line)) return true;
        int numVar = std::stoi(line);

        for (int i = 0; i < numVar; ++i) {
            if (!Tools::ReadValidLine(stream, line)) return true;
            auto parts = Tools::SplitAndTrim(line, ',');
            
            ProblemDataVar var;
            var.lb = std::stod(parts[0]);
            var.ub = (parts[1] == "inf" || parts[1] == "INF") ? 
                     std::numeric_limits<double>::infinity() : std::stod(parts[1]);
            var.obj = std::stod(parts[2]);
            var.type = (parts[3] == "GRB_INTEGER") ? 'I' : 'C';
            var.name = "var_" + std::to_string(i);
            
            vars.push_back(var);
        }

        if (!Tools::ReadValidLine(stream, line)) return true;
        int numConstr = std::stoi(line);

        for (int i = 0; i < numConstr; ++i) {
            if (!Tools::ReadValidLine(stream, line)) return true;
            auto parts = Tools::SplitAndTrim(line, ',');
            
            ProblemDataConstr constr;
            constr.coeffs.push_back(std::stod(parts[0]));
            constr.coeffs.push_back(std::stod(parts[1]));
            constr.sense = parts[2][0];
            constr.rhs = std::stod(parts[3]);
            constr.name = "constr_" + std::to_string(i);
            
            constrs.push_back(constr);
        }

        if (!Tools::ReadValidLine(stream, line)) return true;
        obj = (line == "min") ? 1 : -1; 

        return false;
    } catch (...) {
        return true;
    }
}
} // namespace

void TestDataInitializationStrategy_Solver::DataInit(ProblemData& problemData)
{
    vector<ProblemDataVar> vars;
    vector<ProblemDataConstr> constrs;
    int obj;

    if (LoadTestData(vars, constrs, obj)) {
        throw std::runtime_error("Failed to load test data from file.");
    }

    problemData.addData("vars", vars);

    problemData.addData("constrs", constrs);

    problemData.addData("obj", obj);
}