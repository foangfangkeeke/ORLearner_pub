#include <iostream>
#include <limits>

#include "test.hpp"
#include "milp_solver.hpp"
#include "basic_solver.hpp"

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <filesystem>

using namespace std;

std::string trim(const std::string& s) {
    auto start = s.begin();
    while (start != s.end() && std::isspace(static_cast<unsigned char>(*start))) {
        start++;
    }
    auto end = s.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(static_cast<unsigned char>(*end)));
    return std::string(start, end + 1);
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(trim(token));
    }
    return tokens;
}

bool readValidLine(std::istream& stream, std::string& line) {
    while (std::getline(stream, line)) {
        line = trim(line);
        if (!line.empty() && line[0] != '#') {
            return true;
        }
    }
    return false;
}

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

        if (!readValidLine(stream, line)) return true;
        int numVar = std::stoi(line);

        for (int i = 0; i < numVar; ++i) {
            if (!readValidLine(stream, line)) return true;
            auto parts = split(line, ',');
            
            ProblemDataVar var;
            var.lb = std::stod(parts[0]);
            var.ub = (parts[1] == "inf" || parts[1] == "INF") ? 
                     std::numeric_limits<double>::infinity() : std::stod(parts[1]);
            var.obj = std::stod(parts[2]);
            var.type = (parts[3] == "GRB_INTEGER") ? 'I' : 'C';
            var.name = "var_" + std::to_string(i);
            
            vars.push_back(var);
        }

        if (!readValidLine(stream, line)) return true;
        int numConstr = std::stoi(line);

        for (int i = 0; i < numConstr; ++i) {
            if (!readValidLine(stream, line)) return true;
            auto parts = split(line, ',');
            
            ProblemDataConstr constr;
            constr.coeffs.push_back(std::stod(parts[0]));
            constr.coeffs.push_back(std::stod(parts[1]));
            constr.sense = parts[2][0];
            constr.rhs = std::stod(parts[3]);
            constr.name = "constr_" + std::to_string(i);
            
            constrs.push_back(constr);
        }

        if (!readValidLine(stream, line)) return true;
        obj = (line == "min") ? 1 : -1; 

        return false;
    } catch (...) {
        return true;
    }
}
} // namespace

void TestDataInitializationStrategy::DataInit(ProblemData& data)
{
    vector<ProblemDataVar> vars;
    vector<ProblemDataConstr> constrs;
    int obj;

    if (LoadTestData(vars, constrs, obj)) {
        throw std::runtime_error("Failed to load test data from file.");
    }

    data.addData("vars", vars);

    data.addData("constrs", constrs);

    data.addData("obj", obj);
}