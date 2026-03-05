#include "FCTP.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace std;

string Trim(const string& text)
{
    size_t left = 0;
    while (left < text.size() && std::isspace(static_cast<unsigned char>(text[left]))) {
        ++left;
    }
    size_t right = text.size();
    while (right > left && std::isspace(static_cast<unsigned char>(text[right - 1]))) {
        --right;
    }
    return text.substr(left, right - left);
}

vector<int> ParseIntList(const string& line)
{
    vector<int> values;
    istringstream stream(line);
    string token;
    while (getline(stream, token, ',')) {
        token = Trim(token);
        if (!token.empty()) {
            values.push_back(stoi(token));
        }
    }
    return values;
}

bool LoadFCTPData(
    vector<int>& supplys,
    vector<int>& demands,
    vector<vector<int>>& transCosts,
    vector<vector<int>>& fixedCharges)
{
    const std::filesystem::path dataPath = std::filesystem::path(__FILE__).parent_path() / "FCTP.txt";
    ifstream fin(dataPath);
    if (!fin.is_open()) {
        cerr << "Warning: failed to open " << dataPath << endl;
        return false;
    }

    enum class Section {
        None,
        Supply,
        Demand,
        TransCost,
        FixedCharge
    };

    supplys.clear();
    demands.clear();
    transCosts.clear();
    fixedCharges.clear();

    Section section = Section::None;
    string line;
    try {
        while (getline(fin, line)) {
            line = Trim(line);
            if (line.empty()) {
                continue;
            }

            if (line[0] == '#') {
                string header = line.substr(1);
                transform(header.begin(), header.end(), header.begin(),
                    [](unsigned char c) { return static_cast<char>(tolower(c)); });

                if (header.find("supply") != string::npos) {
                    section = Section::Supply;
                } else if (header.find("demand") != string::npos) {
                    section = Section::Demand;
                } else if (header.find("transcost") != string::npos || header.find("transport") != string::npos) {
                    section = Section::TransCost;
                } else if (header.find("fixedcharge") != string::npos || header.find("fixed") != string::npos) {
                    section = Section::FixedCharge;
                } else {
                    section = Section::None;
                }
                continue;
            }

            vector<int> values = ParseIntList(line);
            if (values.empty()) {
                continue;
            }

            switch (section) {
            case Section::Supply:
                supplys.insert(supplys.end(), values.begin(), values.end());
                break;
            case Section::Demand:
                demands.insert(demands.end(), values.begin(), values.end());
                break;
            case Section::TransCost:
                transCosts.push_back(values);
                break;
            case Section::FixedCharge:
                fixedCharges.push_back(values);
                break;
            case Section::None:
                break;
            }
        }
    } catch (const std::exception& e) {
        cerr << "Warning: invalid FCTP data in " << dataPath << " - " << e.what() << endl;
        return false;
    }

    if (supplys.empty() || demands.empty() || transCosts.empty() || fixedCharges.empty()) {
        cerr << "Warning: missing supply/demand/cost data in " << dataPath << endl;
        return false;
    }

    const size_t supplySize = supplys.size();
    const size_t demandSize = demands.size();
    if (transCosts.size() != supplySize || fixedCharges.size() != supplySize) {
        cerr << "Warning: transCost/fixedCharge row count must equal supply size in " << dataPath << endl;
        return false;
    }
    for (size_t i = 0; i < supplySize; ++i) {
        if (transCosts[i].size() != demandSize || fixedCharges[i].size() != demandSize) {
            cerr << "Warning: transCost/fixedCharge column count must equal demand size in " << dataPath << endl;
            return false;
        }
    }

    return true;
}
} // namespace

void FCTPDataInitializationStrategy_Solver::DataInit(ProblemData& problemData)
{
    vector<int> supplys;
    vector<int> demands;
    vector<vector<int>> transCosts;
    vector<vector<int>> fixedCharges;

    if (!LoadFCTPData(supplys, demands, transCosts, fixedCharges)) {
        throw std::runtime_error("Failed to load FCTP data from file.");
    }

    const int supplyCount = static_cast<int>(supplys.size());
    const int demandCount = static_cast<int>(demands.size());
    const int arcCount = supplyCount * demandCount;
    const int varCount = arcCount * 2;

    auto xIndex = [demandCount](int i, int j) {
        return i * demandCount + j;
    };
    auto yIndex = [arcCount, demandCount](int i, int j) {
        return arcCount + i * demandCount + j;
    };

    vector<ProblemDataVar> vars;
    vars.reserve(varCount);

    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            ProblemDataVar xVar;
            xVar.lb = 0.0;
            xVar.ub = static_cast<double>(min(supplys[i], demands[j]));
            xVar.obj = static_cast<double>(transCosts[i][j]);
            xVar.type = 'C';
            xVar.name = "x_" + to_string(i) + "_" + to_string(j);
            vars.push_back(xVar);
        }
    }

    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            ProblemDataVar yVar;
            yVar.lb = 0.0;
            yVar.ub = 1.0;
            yVar.obj = static_cast<double>(fixedCharges[i][j]);
            yVar.type = 'B';
            yVar.name = "y_" + to_string(i) + "_" + to_string(j);
            vars.push_back(yVar);
        }
    }

    vector<ProblemDataConstr> constrs;
    constrs.reserve(supplyCount + demandCount + arcCount);

    for (int i = 0; i < supplyCount; ++i) {
        ProblemDataConstr constr;
        constr.coeffs.assign(varCount, 0.0);
        for (int j = 0; j < demandCount; ++j) {
            constr.coeffs[xIndex(i, j)] = 1.0;
        }
        constr.sense = '<';
        constr.rhs = static_cast<double>(supplys[i]);
        constr.name = "supply_" + to_string(i);
        constrs.push_back(constr);
    }

    for (int j = 0; j < demandCount; ++j) {
        ProblemDataConstr constr;
        constr.coeffs.assign(varCount, 0.0);
        for (int i = 0; i < supplyCount; ++i) {
            constr.coeffs[xIndex(i, j)] = 1.0;
        }
        constr.sense = '=';
        constr.rhs = static_cast<double>(demands[j]);
        constr.name = "demand_" + to_string(j);
        constrs.push_back(constr);
    }

    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            ProblemDataConstr constr;
            constr.coeffs.assign(varCount, 0.0);
            const double bigM = static_cast<double>(max(supplys[i], demands[j]));
            constr.coeffs[xIndex(i, j)] = 1.0;
            constr.coeffs[yIndex(i, j)] = -bigM;
            constr.sense = '<';
            constr.rhs = 0.0;
            constr.name = "link_" + to_string(i) + "_" + to_string(j);
            constrs.push_back(constr);
        }
    }

    const int objSense = GRB_MINIMIZE;

    problemData.addData("vars", vars);
    problemData.addData("constrs", constrs);
    problemData.addData("obj", objSense);

    cout << "Loaded FCTP data from file." << endl;
    cout << "Supply nodes: " << supplyCount << ", Demand nodes: " << demandCount << endl;
    cout << "Variables: " << vars.size() << ", Constraints: " << constrs.size() << endl;
}