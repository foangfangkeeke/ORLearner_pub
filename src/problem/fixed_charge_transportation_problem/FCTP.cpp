#include "FCTP.hpp"
#include "tools.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace std;

constexpr const char* kVarGroupU = "u";
constexpr const char* kVarGroupV = "v";
constexpr const char* kVarGroupW = "w";
constexpr const char* kConstrGroupDualCap = "dual_cap";

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
            line = Tools::Trim(line);
            if (line.empty()) {
                continue;
            }

            if (line[0] == '#') {
                string header = Tools::ToLower(line.substr(1));

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

            vector<int> values = Tools::ParseIntList(line);
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

void FCTPDataInitializationStrategy_Benders::DataInit(ProblemData& problemData)
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

    vector<vector<int>> capacities(supplyCount, vector<int>(demandCount, 0));
    vector<ProblemDataVar> masterVars;
    masterVars.reserve(arcCount);

    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            capacities[i][j] = max(supplys[i], demands[j]);

            ProblemDataVar yVar;
            yVar.lb = 0.0;
            yVar.ub = 1.0;
            yVar.obj = static_cast<double>(fixedCharges[i][j]);
            yVar.type = 'B';
            yVar.name = "y_" + to_string(i) + "_" + to_string(j);
            masterVars.push_back(yVar);
        }
    }

    problemData.addData("supplys", supplys);
    problemData.addData("demands", demands);
    problemData.addData("transCosts", transCosts);
    problemData.addData("fixedCharges", fixedCharges);
    problemData.addData("capacities", capacities);
    problemData.addData("supplyCount", supplyCount);
    problemData.addData("demandCount", demandCount);
    problemData.addData("arcCount", arcCount);
    problemData.addData("masterVars", masterVars);

    cout << "Loaded FCTP data for Benders decomposition." << endl;
    cout << "Master vars(y): " << masterVars.size() << ", supply nodes: "
         << supplyCount << ", demand nodes: " << demandCount << endl;
}

std::vector<ProblemDataConstr> FCTPDataInitializationStrategy_Benders::ConstrInit(ProblemData& problemData)
{
    const auto& demands = problemData.getData<std::vector<int>>("demands");
    const auto& capacities = problemData.getData<std::vector<std::vector<int>>>("capacities");
    int supplyCount = problemData.getData<int>("supplyCount");
    int demandCount = problemData.getData<int>("demandCount");
    int arcCount = problemData.getData<int>("arcCount");

    auto yIndex = [demandCount](int i, int j) {
        return i * demandCount + j;
    };

    std::vector<ProblemDataConstr> constrs;

    return constrs;
}

void FCTPSubProblemStrategy_Benders::InitSubProblem(const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context)
{
    const auto& transCosts = problemData.getData<std::vector<std::vector<int>>>("transCosts");
    int supplyCount = problemData.getData<int>("supplyCount");
    int demandCount = problemData.getData<int>("demandCount");
    int arcCount = problemData.getData<int>("arcCount");

    auto yIndex = [demandCount](int i, int j) {
        return i * demandCount + j;
    };

    context.Clear();
    auto& uVars = context.EnsureVarGroup(kVarGroupU);
    auto& vVars = context.EnsureVarGroup(kVarGroupV);
    auto& wVars = context.EnsureVarGroup(kVarGroupW);
    auto& dualCapConstrs = context.EnsureConstrGroup(kConstrGroupDualCap);

    uVars.reserve(supplyCount);
    vVars.reserve(demandCount);
    wVars.reserve(arcCount);
    dualCapConstrs.reserve(arcCount);

    subModel.set(GRB_IntParam_InfUnbdInfo, 1);
    subModel.set(GRB_IntParam_DualReductions, 0);

    for (int i = 0; i < supplyCount; ++i) {
        uVars.push_back(
            subModel.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "u_" + to_string(i)));
    }
    for (int j = 0; j < demandCount; ++j) {
        vVars.push_back(
            subModel.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "v_" + to_string(j)));
    }
    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            wVars.push_back(
                subModel.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "w_" + to_string(i) + "_" + to_string(j)));
        }
    }

    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            int idx = yIndex(i, j);
            GRBLinExpr lhs = -uVars[i] + vVars[j] - wVars[idx];
            dualCapConstrs.push_back(
                subModel.addConstr(lhs, '<', static_cast<double>(transCosts[i][j]), "dual_cap_" + to_string(i) + "_" + to_string(j)));
        }
    }

    subModel.setObjective(GRBLinExpr(0.0), GRB_MAXIMIZE);
    subModel.update();
}

void FCTPSubProblemStrategy_Benders::UpdateSubProblem(
    const ProblemData& problemData,
    GRBModel& subModel,
    BendersSubProblemContext& context,
    const std::vector<double>& yValues)
{
    const auto& supplys = problemData.getData<std::vector<int>>("supplys");
    const auto& demands = problemData.getData<std::vector<int>>("demands");
    const auto& capacities = problemData.getData<std::vector<std::vector<int>>>("capacities");
    int supplyCount = problemData.getData<int>("supplyCount");
    int demandCount = problemData.getData<int>("demandCount");
    int arcCount = problemData.getData<int>("arcCount");

    auto yIndex = [demandCount](int i, int j) {
        return i * demandCount + j;
    };

    const auto& uVars = context.RequireVarGroup(kVarGroupU, static_cast<size_t>(supplyCount));
    const auto& vVars = context.RequireVarGroup(kVarGroupV, static_cast<size_t>(demandCount));
    const auto& wVars = context.RequireVarGroup(kVarGroupW, static_cast<size_t>(arcCount));

    GRBLinExpr dualObj = 0;
    for (int i = 0; i < supplyCount; ++i) {
        dualObj += -static_cast<double>(supplys[i]) * uVars[i];
    }
    for (int j = 0; j < demandCount; ++j) {
        dualObj += static_cast<double>(demands[j]) * vVars[j];
    }
    for (int i = 0; i < supplyCount; ++i) {
        for (int j = 0; j < demandCount; ++j) {
            int idx = yIndex(i, j);
            dualObj += -static_cast<double>(capacities[i][j]) * yValues[idx] * wVars[idx];
        }
    }

    subModel.setObjective(dualObj, GRB_MAXIMIZE);
    subModel.update();
}

Status FCTPSubProblemStrategy_Benders::SolveSubProblem(
    const ProblemData& problemData,
    GRBModel& subModel,
    BendersSubProblemContext& context,
    const std::vector<double>& yValues,
    BendersCutInfo& cutInfo,
    double& subObj)
{
    const auto& supplys = problemData.getData<std::vector<int>>("supplys");
    const auto& demands = problemData.getData<std::vector<int>>("demands");
    const auto& capacities = problemData.getData<std::vector<std::vector<int>>>("capacities");
    int supplyCount = problemData.getData<int>("supplyCount");
    int demandCount = problemData.getData<int>("demandCount");
    int arcCount = problemData.getData<int>("arcCount");

    auto yIndex = [demandCount](int i, int j) {
        return i * demandCount + j;
    };

    const auto& uVars = context.RequireVarGroup(kVarGroupU, static_cast<size_t>(supplyCount));
    const auto& vVars = context.RequireVarGroup(kVarGroupV, static_cast<size_t>(demandCount));
    const auto& wVars = context.RequireVarGroup(kVarGroupW, static_cast<size_t>(arcCount));

    subModel.optimize();
    int status = subModel.get(GRB_IntAttr_Status);
    if (status == GRB_OPTIMAL) {
        cutInfo.isOptimalityCut = true;
        cutInfo.sense = '>';
        cutInfo.constant = 0.0;
        cutInfo.rhs = 0.0;
        cutInfo.yCoeffs.assign(arcCount, 0.0);

        for (int i = 0; i < supplyCount; ++i) {
            double uVal = uVars[i].get(GRB_DoubleAttr_X);
            cutInfo.constant += -static_cast<double>(supplys[i]) * uVal;
        }
        for (int j = 0; j < demandCount; ++j) {
            double vVal = vVars[j].get(GRB_DoubleAttr_X);
            cutInfo.constant += static_cast<double>(demands[j]) * vVal;
        }
        for (int i = 0; i < supplyCount; ++i) {
            for (int j = 0; j < demandCount; ++j) {
                int idx = yIndex(i, j);
                double wVal = wVars[idx].get(GRB_DoubleAttr_X);
                cutInfo.yCoeffs[idx] = -static_cast<double>(capacities[i][j]) * wVal;
            }
        }

        subObj = subModel.get(GRB_DoubleAttr_ObjVal);
        return OK;
    }

    if (status == GRB_UNBOUNDED) {
        cutInfo.isOptimalityCut = false;
        cutInfo.sense = '<';
        cutInfo.constant = 0.0;
        cutInfo.yCoeffs.assign(arcCount, 0.0);

        double rayConstant = 0.0;
        for (int i = 0; i < supplyCount; ++i) {
            double ray = uVars[i].get(GRB_DoubleAttr_UnbdRay);
            rayConstant += -static_cast<double>(supplys[i]) * ray;
        }
        for (int j = 0; j < demandCount; ++j) {
            double ray = vVars[j].get(GRB_DoubleAttr_UnbdRay);
            rayConstant += static_cast<double>(demands[j]) * ray;
        }
        for (int i = 0; i < supplyCount; ++i) {
            for (int j = 0; j < demandCount; ++j) {
                int idx = yIndex(i, j);
                double ray = wVars[idx].get(GRB_DoubleAttr_UnbdRay);
                cutInfo.yCoeffs[idx] = -static_cast<double>(capacities[i][j]) * ray;
            }
        }

        cutInfo.rhs = -rayConstant;
        subObj = std::numeric_limits<double>::infinity();
        return OK;
    }

    std::cerr << "FCTP sub-problem ended with unexpected status: " << status << std::endl;
    return ERROR;
}