#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <memory>
#include <filesystem>
#include <cctype>
#include <math.h>
#include <fstream>
#include <sstream>
#include <optional>
#include <algorithm>
#include <stdexcept>
#include "wireless_charging_strategies.hpp"
#include "gurobi_c++.h"
#include "tools.hpp"

using namespace std;
namespace fs = std::filesystem;

#define OFFSET_OPERATION_CFG 0
#define OFFSET_CHARGING_CFG (OFFSET_OPERATION_CFG + 5)
#define OFFSET_NETWORK_CFG (OFFSET_CHARGING_CFG + 4)
#define OFFSET_SCHEDULING_CFG (OFFSET_NETWORK_CFG + 2)
#define OFFSET_CHARGER_CFG(numEB) (OFFSET_SCHEDULING_CFG + 3 + (numEB))
#define GAMMA (0)

using linkTimeType = unordered_map<int, unordered_map<int, double>>;
using stationTimeType = vector<vector<int>>;
using stationType = vector<vector<int>>;
using stringCfgType = vector<vector<string>>;

namespace {
inline fs::path ResolveDataFolderPath(const string& dataFolder) {
    return fs::path(__FILE__).parent_path() / dataFolder;
}

static optional<string> LoadModelTypeFromConfig(const fs::path& filePath) {
    ifstream fin(filePath);
    if (!fin.is_open()) {
        return nullopt;
    }

    string line;
    while (getline(fin, line)) {
        line = Tools::Trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        size_t sepPos = line.find('=');
        if (sepPos == string::npos) {
            sepPos = line.find(':');
        }
        if (sepPos == string::npos) {
            continue;
        }

        string key = Tools::ToLower(Tools::Trim(line.substr(0, sepPos)));
        string value = Tools::ToUpper(Tools::Trim(line.substr(sepPos + 1)));
        if ((key == "modeltype" || key == "model_type" || key == "model") && !value.empty()) {
            return value;
        }
    }

    return nullopt;
}

static optional<stringCfgType> LoadStringTable(const fs::path& filePath) {
    ifstream fin(filePath);
    if (!fin.is_open()) {
        cerr << "Warning: failed to open " << filePath << endl;
        return nullopt;
    }

    stringCfgType rows;
    string line;
    while (getline(fin, line)) {
        line = Tools::Trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        rows.push_back(Tools::SplitAndTrim(line, ',', true));
    }

    if (rows.empty()) {
        cerr << "Warning: no valid data in " << filePath << endl;
        return nullopt;
    }
    return rows;
}

static optional<stationTimeType> LoadIntTable(const fs::path& filePath) {
    auto rows = LoadStringTable(filePath);
    if (!rows.has_value()) {
        return nullopt;
    }

    stationTimeType table;
    table.reserve(rows->size());
    try {
        for (const auto& row : *rows) {
            vector<int> vals;
            vals.reserve(row.size());
            for (const auto& token : row) {
                vals.push_back(stoi(token));
            }
            table.push_back(vals);
        }
    } catch (const std::exception& e) {
        cerr << "Warning: invalid integer table in " << filePath << " - " << e.what() << endl;
        return nullopt;
    }

    return table;
}

static optional<linkTimeType> LoadLinkDistance(const fs::path& filePath) {
    auto rows = LoadStringTable(filePath);
    if (!rows.has_value()) {
        return nullopt;
    }

    linkTimeType links;
    try {
        for (const auto& row : *rows) {
            if (row.size() < 3) {
                continue;
            }
            int from = stoi(row[0]);
            int to = stoi(row[1]);
            double dist = stod(row[2]);
            links[from][to] = dist;
        }
    } catch (const std::exception& e) {
        cerr << "Warning: invalid link table in " << filePath << " - " << e.what() << endl;
        return nullopt;
    }

    if (links.empty()) {
        cerr << "Warning: no valid links in " << filePath << endl;
        return nullopt;
    }
    return links;
}

inline bool isSupportStaticWireless(const string& modelType) {
    return modelType == "M2" || modelType == "M4";
}

inline bool isSupportDynamicWireless(const string& modelType) {
    return modelType == "M3" || modelType == "M4";
}

inline bool isValidModelType(const string& modelType) {
    return modelType == "M1" || modelType == "M2" || modelType == "M3" || modelType == "M4";
}

inline double getSoH(const int& y, const vector<double>& batteryDegradation) {
    double soh = 1.0;
    for (int i = 0; i < y && i < batteryDegradation.size(); ++i) {
        soh *= (1 - batteryDegradation[i]);
    }
    return soh;
}

constexpr const char* kWirelessVarGroupAllVars = "wireless_all_vars";

static bool StartsWith(const string& value, const string& prefix)
{
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

static bool IsWirelessMasterVarName(const string& varName)
{
    return StartsWith(varName, "E_choice_") ||
           StartsWith(varName, "w_station_") ||
           StartsWith(varName, "w_link_");
}
} // namespace

static bool ExtractProblemDataFromModel(GRBModel& model, vector<ProblemDataVar>& vars,
    vector<ProblemDataConstr>& constrs, int& objSense)
{
    try {
        model.update();

        int numVars = model.get(GRB_IntAttr_NumVars);
        int numConstrs = model.get(GRB_IntAttr_NumConstrs);
        objSense = model.get(GRB_IntAttr_ModelSense);

        GRBVar* grbVars = model.getVars();
        GRBConstr* grbConstrs = model.getConstrs();

        vars.clear();
        vars.reserve(static_cast<size_t>(numVars));
        for (int i = 0; i < numVars; ++i) {
            ProblemDataVar var;
            var.lb = grbVars[i].get(GRB_DoubleAttr_LB);
            var.ub = grbVars[i].get(GRB_DoubleAttr_UB);
            var.obj = grbVars[i].get(GRB_DoubleAttr_Obj);
            var.type = grbVars[i].get(GRB_CharAttr_VType);
            var.name = grbVars[i].get(GRB_StringAttr_VarName);
            vars.push_back(var);
        }

        constrs.clear();
        constrs.reserve(static_cast<size_t>(numConstrs));
        for (int c = 0; c < numConstrs; ++c) {
            ProblemDataConstr constr;
            constr.coeffs.assign(static_cast<size_t>(numVars), 0.0);
            constr.sense = grbConstrs[c].get(GRB_CharAttr_Sense);
            constr.rhs = grbConstrs[c].get(GRB_DoubleAttr_RHS);
            constr.name = grbConstrs[c].get(GRB_StringAttr_ConstrName);

            for (int v = 0; v < numVars; ++v) {
                double coeff = model.getCoeff(grbConstrs[c], grbVars[v]);
                if (std::abs(coeff) > 1e-12) {
                    constr.coeffs[static_cast<size_t>(v)] = coeff;
                }
            }

            constrs.push_back(std::move(constr));
        }

        delete[] grbVars;
        delete[] grbConstrs;
        return true;
    } catch (const GRBException& e) {
        cerr << "Failed to extract wireless model to ProblemData: " << e.getMessage() << endl;
        return false;
    }
}

struct OperationCfg {
    double priceOfPowerDay; // CNY per kW
    double priceOfPowerNight; // CNY per kW
    double priceOfUnitBat; // CNY per kW.h
    double socMax;
    double socMin;
    double capacityMin;
    double capacityMax;
    double curbWeight;
    double energyPerWeright;
    double mu;
    double g;
    int timeMin;
    int timeMax;
    int timeInterval;
    int batteryLifespan;
    vector<double> batteryDegradation;
    double recyclingPriceRate;
    int planningYears;
};

// charger information: fast charging, static wireless charging, dynamic wireless charging
struct ChargerCfg {
    int num;
    vector<double> chargeRate; // kW per minute;
    vector<double> chargeEfficiency;
    vector<double> priceOfUnitCharger; // unit price of facility construction
    double priceOfInverterPerkW; // unit price of inverter (CNY/kW)
};

struct SchedulingCfg {
    vector<int> EBs;
    int numTrip = 0;
    int numEB;
    vector<int> tripNumEBs;
    vector<vector<int>> tripEBs;
    vector<vector<int>> stationTrips;
    vector<vector<int>> stationArrTimes;
    vector<vector<int>> stationDepTimes;
};

struct NetworkCfg {
    int chargeStationNum;
    unordered_set<int> stationPhys;
    unordered_set<int> chargerPhys;
    unordered_map<int, unordered_map<int, double>> linkDistances;
    vector<double> startDistanceEBs;
    vector<double> endDistanceEBs;
    unordered_map<int, int> chargerNums;
};

class ModelData {
public:
    ModelData(string dataFolder)
        : dataFolder(std::move(dataFolder)), dataFolderPath(ResolveDataFolderPath(this->dataFolder)) {
        Init();
    }
    ~ModelData() = default;

    const OperationCfg& GetOperationCfg() const {
        return operationCfg;
    }
    const ChargerCfg& GetChargerCfg() const {
        return chargerCfg;
    }
    const SchedulingCfg& GetSchedulingCfg() const {
        return schedulingCfg;
    }
    const NetworkCfg& GetNetworkCfg() const {
        return networkCfg;
    }

    const fs::path& GetDataFolderPath() const {
        return dataFolderPath;
    }

private:
    struct OperationCfg operationCfg;
    struct ChargerCfg chargerCfg;
    struct SchedulingCfg schedulingCfg;
    struct NetworkCfg networkCfg;
    string dataFolder;
    fs::path dataFolderPath;

    fs::path Resolve(const string& file) const {
        return dataFolderPath / file;
    }

    void Init() {
        LoadConfigFromTxt();

        for (const auto& stationTrip : schedulingCfg.stationTrips) {
            for (int station : stationTrip) {
                networkCfg.stationPhys.insert(station);
            }
        }
    }

    bool LoadConfigFromTxt() {
        auto tmpDis = LoadLinkDistance(Resolve("linkDistance.txt"));
        auto tmpTripArrTimes = LoadIntTable(Resolve("tripArrTimes.txt"));
        auto tmpTripDepTimes = LoadIntTable(Resolve("tripDepTimes.txt"));
        auto tmpTripStations = LoadIntTable(Resolve("tripStations.txt"));
        if (tmpDis.has_value()) {
            networkCfg.linkDistances = *tmpDis;
        }
        if (tmpTripDepTimes.has_value()) {
            schedulingCfg.stationDepTimes = *tmpTripDepTimes;
        }
        if (tmpTripArrTimes.has_value()) {
            schedulingCfg.stationArrTimes = *tmpTripArrTimes;
        }
        if (tmpTripStations.has_value()) {
            schedulingCfg.stationTrips = *tmpTripStations;
        }

        auto tmpMiscCfg = LoadStringTable(Resolve("miscCfg.txt")); // TODO:名称过长
        if (tmpMiscCfg.has_value()) {
            vector<string> operationCfgDouble = tmpMiscCfg->at(OFFSET_OPERATION_CFG);
            vector<string> operationCfgInt = tmpMiscCfg->at(OFFSET_OPERATION_CFG + 1);
            vector<string> operationCfgBus = tmpMiscCfg->at(OFFSET_OPERATION_CFG + 2);
            vector<string> operationCfgBattery = tmpMiscCfg->at(OFFSET_OPERATION_CFG + 3);
            vector<string> operationCfgPeriod = tmpMiscCfg->at(OFFSET_OPERATION_CFG + 4);
            operationCfg.priceOfPowerDay = stod(operationCfgDouble[0]);
            operationCfg.priceOfPowerNight = stod(operationCfgDouble[1]);
            operationCfg.priceOfUnitBat = stod(operationCfgDouble[2]);
            operationCfg.socMax = stod(operationCfgDouble[3]);
            operationCfg.socMin = stod(operationCfgDouble[4]);
            operationCfg.capacityMin = stod(operationCfgDouble[5]);
            operationCfg.capacityMax = stod(operationCfgDouble[6]);
            operationCfg.timeMin = stoi(operationCfgInt[0]);
            operationCfg.timeMax = stoi(operationCfgInt[1]);
            operationCfg.timeInterval = stoi(operationCfgInt[2]);
            operationCfg.curbWeight = stod(operationCfgBus[0]);
            operationCfg.energyPerWeright = stod(operationCfgBus[1]);
            operationCfg.mu = stod(operationCfgBus[2]);
            operationCfg.g = stod(operationCfgBus[3]);
            operationCfg.batteryLifespan = stoi(operationCfgBattery[0]);
            for (int i = 0; i < operationCfg.batteryLifespan; i++) {
                operationCfg.batteryDegradation.push_back(stod(operationCfgBattery[1 + i]));
            }
            operationCfg.recyclingPriceRate = stod(operationCfgBattery[1 + operationCfg.batteryLifespan]);
            operationCfg.planningYears = stoi(operationCfgPeriod[0]);

            vector<string> chargerCfgNum = tmpMiscCfg->at(OFFSET_CHARGING_CFG);
            vector<string> chargerCfgRate = tmpMiscCfg->at(OFFSET_CHARGING_CFG + 1);
            vector<string> chargerCfgEfficiency = tmpMiscCfg->at(OFFSET_CHARGING_CFG + 2);
            vector<string> chargerCfgInt = tmpMiscCfg->at(OFFSET_CHARGING_CFG + 3);
            chargerCfg.num = stoi(chargerCfgNum[0]);
            for (int i = 0; i < chargerCfg.num; i++) {
                chargerCfg.chargeRate.push_back(stod(chargerCfgRate[i]));
                chargerCfg.chargeEfficiency.push_back(stod(chargerCfgEfficiency[i]));
                chargerCfg.priceOfUnitCharger.push_back(stoi(chargerCfgInt[i]));
            }
            chargerCfg.priceOfInverterPerkW = stod(chargerCfgInt[chargerCfg.num]);

            vector<string> schedulingCfgNum = tmpMiscCfg->at(OFFSET_SCHEDULING_CFG);
            vector<string> schedulingCfgEBs = tmpMiscCfg->at(OFFSET_SCHEDULING_CFG + 1);
            vector<string> schedulingCfgTripNumEBs = tmpMiscCfg->at(OFFSET_SCHEDULING_CFG + 2);
            schedulingCfg.numEB = stoi(schedulingCfgNum[0]);
            for (int i = 0; i < schedulingCfg.numEB; i++) {
                schedulingCfg.EBs.push_back(stoi(schedulingCfgEBs[i]));
                schedulingCfg.tripNumEBs.push_back(stoi(schedulingCfgTripNumEBs[i]));
                schedulingCfg.numTrip += stoi(schedulingCfgTripNumEBs[i]);
                vector<string> schedulingCfgTrips = tmpMiscCfg->at(OFFSET_SCHEDULING_CFG + 3 + i);
                vector<int> tmp;
                for (int j = 0; j < schedulingCfg.tripNumEBs[i]; j++) {
                    tmp.push_back(stoi(schedulingCfgTrips[j]));
                }
                schedulingCfg.tripEBs.push_back(tmp);
            }

            vector<string> networkCfgStart = tmpMiscCfg->at(OFFSET_NETWORK_CFG);
            vector<string> networkCfgEnd = tmpMiscCfg->at(OFFSET_NETWORK_CFG + 1);
            for (int i = 0; i < schedulingCfg.numEB; i++) {
                networkCfg.startDistanceEBs.push_back(stod(networkCfgStart[i]));
                networkCfg.endDistanceEBs.push_back(stod(networkCfgEnd[i]));
            }

            vector<string> chargeCfgNums = tmpMiscCfg->at(OFFSET_CHARGER_CFG(schedulingCfg.numEB));
            networkCfg.chargeStationNum = stoi(chargeCfgNums[0]);
            for (int i = 0; i < networkCfg.chargeStationNum; i++) {
                vector<string> chargerCfgNums = tmpMiscCfg->at(OFFSET_CHARGER_CFG(schedulingCfg.numEB) + 1 + i);
                networkCfg.chargerNums[stoi(chargerCfgNums[0])] = stoi(chargerCfgNums[1]);
                networkCfg.chargerPhys.insert(stoi(chargerCfgNums[0]));
            }
        }

        return true;
    }
};

static bool SolveWithGurobiWirelessChargingStratgiesInternal(const string& modelType, const string& dataFolder,
    bool allowWarmStart, bool allowOutputSolution, double givenGap, double givenTime, bool buildProblemDataOnly,
    vector<ProblemDataVar>* outVars, vector<ProblemDataConstr>* outConstrs, int* outObjSense) {

    auto timeRecordStart = chrono::high_resolution_clock::now();
    unique_ptr<ModelData> data = make_unique<ModelData>(dataFolder);
    auto timeRecordFinishDataInit = chrono::high_resolution_clock::now();

    OperationCfg operationCfg = data->GetOperationCfg();
    ChargerCfg chargerCfg = data->GetChargerCfg();
    SchedulingCfg schedulingCfg = data->GetSchedulingCfg();
    NetworkCfg networkCfg = data->GetNetworkCfg();
    fs::path dataFolderPath = data->GetDataFolderPath();
    string runId = modelType;
    fs::path resultDir = dataFolderPath / runId;
    fs::create_directories(resultDir);
    fs::path solutionPath = resultDir / "solution.sol";
    fs::path summaryPath = resultDir / "summary.txt";

    int planningYears = operationCfg.planningYears;
    int batteryLifespan = operationCfg.batteryLifespan;
    int batteryRounds = planningYears / batteryLifespan ;
    int residualLife = planningYears - batteryRounds * batteryLifespan;

    fs::path logPath = resultDir / "gurobi_log.txt";

    GRBEnv env = GRBEnv(true);
    env.set("LogFile", logPath.string());
    env.set(GRB_IntParam_OutputFlag, buildProblemDataOnly ? 0 : 1);
    env.start();

    GRBModel model = GRBModel(env);
    model.set(GRB_StringAttr_ModelName, runId);

    model.set(GRB_DoubleParam_MIPGap, givenGap);
    model.set(GRB_DoubleParam_TimeLimit, givenTime);
    model.set(GRB_IntParam_Presolve, 2);
    model.set(GRB_IntParam_MIPFocus, 1); // 优先改进可行解
    model.set(GRB_DoubleParam_Heuristics, 0.5); // 提高启发式强度
    // model.set(GRB_IntParam_PumpPasses, 50); // 强化可行性泵
    // model.set(GRB_IntParam_RINS, 50); // 强化RINS启发
    model.set(GRB_IntParam_Threads, 14);
    model.set(GRB_IntParam_Method, 1);

    // decision variable
    GRBVar E = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "E"); // 初始电池容量

    // 预先计算每种备选容量下，一整天路径的总能耗及对应SoC下降，便于后续分析/约束扩展
    auto calcTotalEnergyForCapacity = [&](double cap) {
        double total = 0.0;
        auto segmentEnergy = [&](double dist) {
            return dist * (operationCfg.curbWeight + cap / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
        };

        for (int ebIdx = 0; ebIdx < schedulingCfg.numEB; ++ebIdx) {
            int eb = schedulingCfg.EBs[ebIdx];

            // depot -> first station
            total += segmentEnergy(networkCfg.startDistanceEBs[eb]);

            // station-to-station legs
            for (int tripIdx = 0; tripIdx < static_cast<int>(schedulingCfg.tripEBs[eb].size()); ++tripIdx) {
                int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                int numStations = static_cast<int>(schedulingCfg.stationTrips[tripDep].size());
                for (int stationIdx = 0; stationIdx < numStations; ++stationIdx) {
                    bool isLastStation = stationIdx == numStations - 1;
                    bool isLastTrip = tripIdx == static_cast<int>(schedulingCfg.tripEBs[eb].size()) - 1;
                    if (isLastStation && isLastTrip) {
                        break; // handled by depot leg
                    }

                    int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                    int tripArr = isLastStation ? schedulingCfg.tripEBs[eb][tripIdx + 1] : tripDep;
                    int stationArr = isLastStation ? schedulingCfg.stationTrips[tripArr][0]
                                                   : schedulingCfg.stationTrips[tripArr][stationIdx + 1];

                    auto distIt = networkCfg.linkDistances.find(stationDep);
                    if (distIt != networkCfg.linkDistances.end()) {
                        auto innerIt = distIt->second.find(stationArr);
                        if (innerIt != distIt->second.end()) {
                            total += segmentEnergy(innerIt->second);
                        }
                    }
                }
            }

            // last station -> depot
            total += segmentEnergy(networkCfg.endDistanceEBs[eb]);
        }
        return total;
    };

    vector<double> batteryOptions;
    vector<double> batterySocDrops; // 总路程能耗占容量的比值
    // for (int cap = ((modelType == "M1") ? 200 : 30); cap <= ((modelType == "M1") ? 300 : 100); cap += 10) {
    for (double cap = operationCfg.capacityMin; cap <= operationCfg.capacityMax; cap += 10.0) {
        if (cap <= 0.0) {
            continue;
        }
        batteryOptions.push_back(cap);
        double totalEnergy = calcTotalEnergyForCapacity(cap);
        batterySocDrops.push_back(totalEnergy / cap);
    }

    vector<GRBVar> batteryChoice;
    batteryChoice.reserve(batteryOptions.size());
    for (double cap : batteryOptions) {
        string varName = "E_choice_" + to_string(static_cast<int>(cap));
        batteryChoice.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName));
    }
    vector<vector<GRBVar>> chargeNight_yb; // [year][eb]
    unordered_map<int, GRBVar> wirelessStation; // M2/M4: construction of facility (一次性建设)
    unordered_map<int, unordered_map<int, GRBVar>> wirelessLink; // M3/M4: construction of facility (一次性建设)
    vector<vector<vector<unordered_map<int, GRBVar>>>> charge_ybns; // [year][eb][trip][station]
    vector<vector<vector<unordered_map<int, unordered_map<int, GRBVar>>>>> charge_ybnl; // M3/M4: [year][eb][trip][start][end]
    vector<vector<vector<vector<GRBVar>>>> energyDep_ybns; // [year][eb][trip][station]
    vector<vector<vector<vector<GRBVar>>>> energyArr_ybns; // [year][eb][trip][station]
    vector<unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, GRBVar>>>>> x_ybns_t; // [year][eb][trip][station][time]

    size_t numEB = schedulingCfg.numEB;
    size_t numStations = networkCfg.stationPhys.size();
    size_t numChargeStations = networkCfg.chargerPhys.size();
    size_t numLink = 0;
    for (const auto& destDist : networkCfg.linkDistances) {
        numLink += destDist.second.size();
    }

    chargeNight_yb.reserve(batteryLifespan);
    charge_ybns.reserve(batteryLifespan);
    charge_ybnl.reserve(batteryLifespan);
    energyDep_ybns.reserve(batteryLifespan);
    energyArr_ybns.reserve(batteryLifespan);
    x_ybns_t.reserve(batteryLifespan);

    vector<unordered_map<int, vector<GRBLinExpr>>> chargerOccupy;
    chargerOccupy.reserve(batteryLifespan);
    for (int year = 0; year < batteryLifespan; year++) {
        unordered_map<int, vector<GRBLinExpr>> chargerOccupyYearly;
        for (int chargerPhy : networkCfg.chargerPhys) {
            chargerOccupyYearly[chargerPhy].resize(operationCfg.timeMax - operationCfg.timeMin);
        }
        chargerOccupy.push_back(chargerOccupyYearly);
    }

    for (int year = 0; year < batteryLifespan; year++) {
        vector<GRBVar> chargeNight_y;
        chargeNight_y.reserve(numEB);
        for (int eb : schedulingCfg.EBs) {
            string varName = "c_Night_y" + to_string(year) + "_b" + to_string(eb);
            chargeNight_y.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varName));
        }
        chargeNight_yb.push_back(chargeNight_y);
    }

    for (int year = 0; year < batteryLifespan; year++) {
        vector<vector<unordered_map<int, GRBVar>>> charge_y;
        vector<vector<vector<GRBVar>>> energyDep_y;
        vector<vector<vector<GRBVar>>> energyArr_y;
        charge_y.reserve(numEB);
        energyDep_y.reserve(numEB);
        energyArr_y.reserve(numEB);

        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
            vector<unordered_map<int, GRBVar>> charge_yb;
            vector<vector<GRBVar>> energyDep_yb;
            vector<vector<GRBVar>> energyArr_yb;
            charge_yb.reserve(numEBTrip);
            energyDep_yb.reserve(numEBTrip);
            energyArr_yb.reserve(numEBTrip);

            for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
                unordered_map<int, GRBVar> charge_ybn;
                vector<GRBVar> energyDep_ybn;
                vector<GRBVar> energyArr_ybn;
                energyDep_ybn.reserve(numEBTripStations);
                energyArr_ybn.reserve(numEBTripStations);

                for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                    string varNameEnergyDep = "e_Dep" + sub;
                    string varNameEnergyArr = "e_Arr" + sub;
                    energyDep_ybn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varNameEnergyDep));
                    energyArr_ybn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varNameEnergyArr));
                    string varNameCharge = "c" + sub;
                    if (isSupportStaticWireless(modelType) || networkCfg.chargerPhys.count(station)) {
                        charge_ybn[station] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varNameCharge);
                    }
                }
                charge_yb.push_back(charge_ybn);
                energyDep_yb.push_back(energyDep_ybn);
                energyArr_yb.push_back(energyArr_ybn);
            }
            charge_y.push_back(charge_yb);
            energyDep_y.push_back(energyDep_yb);
            energyArr_y.push_back(energyArr_yb);
        }
        charge_ybns.push_back(charge_y);
        energyDep_ybns.push_back(energyDep_y);
        energyArr_ybns.push_back(energyArr_y);
    }

    // stationOccupy
    for (int year = 0; year < batteryLifespan; year++) {
        unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, GRBVar>>>> x_y;
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
            for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
                for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    if (!networkCfg.chargerPhys.count(station)) {
                        continue;
                    }
                    int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                    int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                    for (int time = arrTime; time < depTime; time++) {
                        string varName = "x_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) +
                                                                 "_s" + to_string(station) + "_i" + to_string(stationIdx) + "_t" + to_string(time);
                        x_y[eb][trip][station][time] = model.addVar(0, 1, 0, GRB_BINARY, varName);
                    }
                }
            }
        }
        x_ybns_t.push_back(x_y);
    }

    // static wireless charging facility
    if (isSupportStaticWireless(modelType)) {
        for (const int& station : networkCfg.stationPhys) {
            if (!networkCfg.chargerPhys.count(station)) {
                string varName = "w_station_s" + to_string(station);
                // if (modelType == "M2") {
                //     if (station == 396 || station == 312 || station == 399 || station == 283 || station == 115 || station == 75 || station == 118 || station == 178 || station == 57 || station == 107 || station == 139 || station == 154 || station == 347 || station == 180 || station == 86 || station == 305 || station == 289 || station == 371 || station == 88 || station == 18 || station == 34 || station == 102 || station == 32 || station == 408 || station == 411) {
                //         wirelessStation[station] = model.addVar(1, 1, 1, GRB_BINARY, varName);
                //     } else {
                //         wirelessStation[station] = model.addVar(0, 0, 0, GRB_BINARY, varName);
                //     }
                // } else if (modelType == "M4") {
                //     if (station == 396 || station == 130 || station == 194 || station == 283 || station == 145 || station == 115 || station == 75 || station == 118 || station == 178 || station == 57 || station == 107 || station == 139 || station == 154 || station == 347 || station == 180 || station == 371 || station == 18 || station == 221 || station == 34 || station == 102 || station == 32 || station == 408 || station == 411) {
                //         wirelessStation[station] = model.addVar(1, 1, 1, GRB_BINARY, varName);
                //     } else {
                //         wirelessStation[station] = model.addVar(0, 0, 0, GRB_BINARY, varName);
                //     }
                // } else {
                    wirelessStation[station] = model.addVar(0.0, 1, 0.0, GRB_BINARY, varName);
                // }
            }
        }
    }

    // dynamic wireless charging
    if (isSupportDynamicWireless(modelType)) {
        for (int year = 0; year < batteryLifespan; year++) {
            vector<vector<unordered_map<int, unordered_map<int, GRBVar>>>> charge_y;
            for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                auto eb = schedulingCfg.EBs[ebIdx];
                size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
                vector<unordered_map<int, unordered_map<int, GRBVar>>> charge_yb;
                charge_yb.reserve(numEBTrip);
                for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                    auto tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                    size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
                    unordered_map<int, unordered_map<int, GRBVar>> charge_ybn;
                    for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                        int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                        int tripArr;
                        int stationArr;
                        int stationArrIdx;
                        if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                            if (tripIdx == schedulingCfg.tripEBs[eb].size() - 1) {
                                continue;
                            }
                            tripArr = schedulingCfg.tripEBs[eb][tripIdx + 1];
                            stationArrIdx = 0;
                            stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                        } else {
                            tripArr = tripDep;
                            stationArrIdx = stationIdx + 1;
                            stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                        }

                        string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                                     "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                        charge_ybn[stationDep][stationArr] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "c" + sub);
                    }
                    charge_yb.push_back(charge_ybn);
                }
                charge_y.push_back(charge_yb);
            }
            charge_ybnl.push_back(charge_y);
        }

        // facility
        for (const auto& [start, endMap] : networkCfg.linkDistances) {
            for (const auto& [end, _] : endMap) {
                string varName = "w_link_l" + to_string(start) + "To" + to_string(end);
                // if (modelType == "M3") {
                //     if ((start == 396 && end == 107) || (start == 75 && end == 313) || (start == 118 && end == 139) ||
                //         (start == 150 && end == 149) || (start == 141 && end == 142) || (start == 107 && end == 118) ||
                //         (start == 139 && end == 154) || (start == 154 && end == 347) || (start == 347 && end == 180) || (start == 180 && end == 178)) {
                //         wirelessLink[start][end] = model.addVar(1, 1, 1, GRB_BINARY, varName);
                //     } else {
                //         wirelessLink[start][end] = model.addVar(0, 0, 0, GRB_BINARY, varName);
                //     }
                // } else if (modelType == "M4") {
                //     if ((start == 75 && end == 313) || (start == 139 && end == 154) || (start == 154 && end == 347)) {
                //         wirelessLink[start][end] = model.addVar(1, 1, 1, GRB_BINARY, varName);
                //     } else {
                //         wirelessLink[start][end] = model.addVar(0, 0, 0, GRB_BINARY, varName);
                //     }
                // } else {
                    wirelessLink[start][end] = model.addVar(0.0, 1, 0.0, GRB_BINARY, varName);
                // }
            }
        }
    }

    model.update();

    // objective function
    GRBLinExpr obj = 0;

    for (int year = 0; year < batteryLifespan; year++) {
        GRBLinExpr sum_day = 0;
        for (int eb : schedulingCfg.EBs) {
            for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                for (const auto& [_, chargeVar] : charge_ybns[year][eb][tripIdx]) {
                    sum_day += chargeVar;
                }
            }
        }

        if (isSupportDynamicWireless(modelType)) {
            for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                int eb = schedulingCfg.EBs[ebIdx];
                size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
                for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                    int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                    size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
                    for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                        int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                        int tripArr;
                        int stationArr;
                        if (stationIdx == numEBTripStations - 1) {
                            if (tripIdx == numEBTrip - 1) {
                                continue;
                            }
                            tripArr = schedulingCfg.tripEBs[eb][tripIdx + 1];
                            stationArr = schedulingCfg.stationTrips[tripArr][0];
                        } else {
                            tripArr = tripDep;
                            stationArr = schedulingCfg.stationTrips[tripArr][stationIdx + 1];
                        }

                        sum_day += charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr];
                    }
                }
            }
        }

        GRBLinExpr sum_night = 0;
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            sum_night += chargeNight_yb[year][ebIdx];
        }

        int yearCount = batteryRounds;
        if (year < residualLife) {
            yearCount = batteryRounds + 1;
        }
        obj += 365 * yearCount * (operationCfg.priceOfPowerDay * sum_day + operationCfg.priceOfPowerNight * sum_night);
    }

    vector<double> wearPattern = operationCfg.batteryDegradation;

    auto sohAfterYears = [&](int years) {
        return getSoH(years, wearPattern); // multiplicative SoH from yearly degradation
    };

    int fullCycles = planningYears / batteryLifespan;           // packs that run full lifespan
    int tailYears = planningYears % batteryLifespan;            // remaining years on the last pack

    double sohAtEol = sohAfterYears(batteryLifespan);           // remaining capacity after a full lifespan
    double sohTail = (tailYears > 0) ? sohAfterYears(tailYears) : 0.0; // remaining capacity of the final partial pack

    int batteriesNeeded = fullCycles + (tailYears > 0 ? 1 : 0);
    double salvageCapacitySum = fullCycles * sohAtEol + sohTail;

    GRBLinExpr batteryPricePerUnit = numEB * operationCfg.priceOfUnitBat * E;
    obj += batteryPricePerUnit * static_cast<double>(batteriesNeeded);
    obj -= batteryPricePerUnit * salvageCapacitySum * operationCfg.recyclingPriceRate;

    if (isSupportStaticWireless(modelType)) {
        GRBLinExpr sum_station_charger = 0;
        for (const auto& [_, wirelessStationVar] : wirelessStation) {
            sum_station_charger += wirelessStationVar;
        }
        obj += (60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[1] + chargerCfg.priceOfUnitCharger[1]) * sum_station_charger * 2;
    }

    // cost of constructing dynamic wireless charging facility
    if (isSupportDynamicWireless(modelType)) {
        GRBLinExpr sum_link_charger = 0;
        GRBLinExpr sum_link_charger_cnt = 0;
        for (const auto& [start, endMap] : networkCfg.linkDistances) {
            for (const auto& [end, dist] : endMap) {
                sum_link_charger += wirelessLink[start][end] * dist;
                sum_link_charger_cnt += wirelessLink[start][end];
            }
        }
        obj += chargerCfg.priceOfUnitCharger[2] * sum_link_charger;
        obj += 60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[2] * sum_link_charger_cnt;
    }

    model.setObjective(obj, GRB_MINIMIZE);

    // constraints
    GRBLinExpr capacitySelector = 0;
    for (size_t i = 0; i < batteryOptions.size(); ++i) {
        capacitySelector += batteryOptions[i] * batteryChoice[i];
    }
    GRBLinExpr capacityPick = 0;
    for (const auto& choice : batteryChoice) {
        capacityPick += choice;
    }
    model.addConstr(capacityPick == 1, "constrEChooseOne");
    model.addConstr(E == capacitySelector, "constrEChoiceTie");

    // energy range
    for (int year = 0; year < batteryLifespan; year++) {
        double yearlyBatteryCapacityFactor = getSoH(year, operationCfg.batteryDegradation);
        double yearlySocMax = operationCfg.socMax * yearlyBatteryCapacityFactor;
        double yearlySocMin = operationCfg.socMin * yearlyBatteryCapacityFactor;
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
            for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
                for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                    GRBVar arrE = energyArr_ybns[year][ebIdx][tripIdx][stationIdx];
                    GRBVar depE = energyDep_ybns[year][ebIdx][tripIdx][stationIdx];
                    model.addConstr(arrE >= E * yearlySocMin, "constrArrMin" + sub);
                    model.addConstr(arrE <= E * yearlySocMax, "constrArrMax" + sub);
                    model.addConstr(depE >= E * yearlySocMin, "constrDepMin" + sub);
                    model.addConstr(depE <= E * yearlySocMax, "constrDepMax" + sub);
                }
            }
        }

    // number of fast charger
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
            for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
                for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    if (!networkCfg.chargerPhys.count(station)) {
                        continue;
                    }
                    int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                    int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                    for (int time = arrTime; time < depTime; time++) {
                        chargerOccupy[year][station][time] += x_ybns_t[year][eb][trip][station][time];
                    }
                }
            }
        }
        for (int chargerPhy : networkCfg.chargerPhys) {
            for (int time = operationCfg.timeMin; time < operationCfg.timeMax; time++) {
                if (chargerOccupy[year][chargerPhy][time].size() > 0) {
                    string sub = "_y" + to_string(year) + "_t" + to_string(time) + "_c" + to_string(chargerPhy);
                    model.addConstr(chargerOccupy[year][chargerPhy][time] <= networkCfg.chargerNums[chargerPhy], "constChargerNum" + sub);
                }
            }
        }

    // c_bns in fast charging stations
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    if (!networkCfg.chargerPhys.count(station)) {
                        continue;
                    }
                    string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                    GRBVar charge = charge_ybns[year][ebIdx][tripIdx][station];
                    GRBLinExpr occupyTime;
                    for (int time = schedulingCfg.stationArrTimes[trip][stationIdx];
                             time < schedulingCfg.stationDepTimes[trip][stationIdx]; time++) {
                        occupyTime += x_ybns_t[year][eb][trip][station][time];
                    }
                    model.addConstr(charge <= chargerCfg.chargeRate[0] * occupyTime, "constrChargeFast" + sub);
                }
            }
        }

    // c_bns in static wireless charging stations
        if (isSupportStaticWireless(modelType)) {
            for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                int eb = schedulingCfg.EBs[ebIdx];
                for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                    int trip = schedulingCfg.tripEBs[eb][tripIdx];
                    for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                        int station = schedulingCfg.stationTrips[trip][stationIdx];
                        if (networkCfg.chargerPhys.count(station)) {
                            continue;
                        }
                        string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                        GRBVar charge = charge_ybns[year][ebIdx][tripIdx][station];
                        GRBVar isBuilt = wirelessStation[station];
                        int stayTime = schedulingCfg.stationDepTimes[trip][stationIdx] - schedulingCfg.stationArrTimes[trip][stationIdx];
                        model.addConstr(charge <= chargerCfg.chargeRate[1] * isBuilt * stayTime,
                                        "constrChargeWirelessStation" + sub);
                    }
                }
            }
        }

    // c_bnl in dynamic wireless charging links
        if (isSupportDynamicWireless(modelType)) {
            for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                int eb = schedulingCfg.EBs[ebIdx];
                for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                    int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                    for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                        int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                        int tripArrIdx;
                        int stationArrIdx;
                        if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                            if (tripIdx == schedulingCfg.tripEBs[eb].size() - 1) {
                                continue;
                            }
                            tripArrIdx = tripIdx + 1;
                            stationArrIdx = 0;
                        } else {
                            tripArrIdx = tripIdx;
                            stationArrIdx = stationIdx + 1;
                        }

                        int tripArr = schedulingCfg.tripEBs[eb][tripArrIdx];
                        int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];

                        string subCharge = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                                           "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                        GRBVar linkCharge = charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr];
                        GRBVar isBuilt = wirelessLink[stationDep][stationArr];
                        int timeDep = schedulingCfg.stationDepTimes[tripDep][stationIdx];
                        int timeArr = schedulingCfg.stationArrTimes[tripArr][stationArrIdx];
                        int travel_time = timeArr - timeDep;
                        model.addConstr(linkCharge <= chargerCfg.chargeRate[2] * travel_time * isBuilt,
                            "constrChargeWirelessLink" + subCharge);
                    }
                }
            }
        }

    // energy of the first and the last trips
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            const auto& trips = schedulingCfg.tripEBs[eb];
            int startTripIdx = 0;
            int startTrip = trips[startTripIdx];
            int startStationIdx = 0;
            int startStation = schedulingCfg.stationTrips[startTrip][startStationIdx];
            string subStart = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(startTrip) + "_s" + to_string(startStation) + "_i" + to_string(startStationIdx);
            double distStart = networkCfg.startDistanceEBs[eb];
            GRBLinExpr consumptionStart = distStart * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
            GRBVar startEnergy = energyArr_ybns[year][eb][startTripIdx][startStationIdx];
            model.addConstr(startEnergy == E * yearlySocMax - consumptionStart, "e_Start" + subStart);

            int endTripIdx = trips.size() - 1;
            int endTrip = trips[endTripIdx];
            int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
            int endStation = schedulingCfg.stationTrips[endTrip][endStationIdx];
            string subEnd = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(endStation) + "_i" + to_string(endStationIdx);
            double distEnd = networkCfg.endDistanceEBs[eb];
            GRBLinExpr consumptionEnd = distEnd * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
            GRBVar endEnergy = energyDep_ybns[year][eb][endTripIdx][endStationIdx];
            model.addConstr(endEnergy >= E * yearlySocMin + consumptionEnd,
                            "e_End" + subEnd);
        }

    // inner station energy connection
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                int trip = schedulingCfg.tripEBs[eb][tripIdx];
                for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                    int station = schedulingCfg.stationTrips[trip][stationIdx];
                    string sub = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                    GRBVar arrE = energyArr_ybns[year][ebIdx][tripIdx][stationIdx];
                    GRBVar depE = energyDep_ybns[year][ebIdx][tripIdx][stationIdx];
                    if (networkCfg.chargerPhys.count(station)) {
                        GRBVar charge = charge_ybns[year][ebIdx][tripIdx][station];
                        model.addConstr(depE == arrE + charge * chargerCfg.chargeEfficiency[0], "e_InnerStation" + sub);
                    } else {
                        if (isSupportStaticWireless(modelType)) {
                            GRBVar charge = charge_ybns[year][ebIdx][tripIdx][station];
                            model.addConstr(depE == arrE + charge * chargerCfg.chargeEfficiency[1], "e_InnerStation" + sub);
                        } else {
                            model.addConstr(depE == arrE, "e_InnerStation" + sub);
                        }
                    }
                }
            }
        }

    // inter station energy connection
        for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
            int eb = schedulingCfg.EBs[ebIdx];
            for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
                int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                    int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                    string subDep = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) + "_i" + to_string(stationIdx);
                    GRBVar depE = energyDep_ybns[year][ebIdx][tripIdx][stationIdx];
                    int tripArrIdx;
                    int stationArrIdx;
                    if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                        if (tripIdx == schedulingCfg.tripEBs[eb].size() - 1) {
                            continue;
                        }
                        tripArrIdx = tripIdx + 1;
                        stationArrIdx = 0;
                    } else {
                        tripArrIdx = tripIdx;
                        stationArrIdx = stationIdx + 1;
                    }

                    int tripArr = schedulingCfg.tripEBs[eb][tripArrIdx];
                    int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                    GRBVar arrE = energyArr_ybns[year][ebIdx][tripArrIdx][stationArrIdx];
                    double dist = networkCfg.linkDistances[stationDep][stationArr];
                    GRBLinExpr consumption = dist * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
                    if (isSupportDynamicWireless(modelType)) {
                        GRBVar linkCharge = charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr];
                        model.addConstr(arrE == depE - consumption + linkCharge * chargerCfg.chargeEfficiency[2],
                                        "e_InterStation" + subDep);
                    } else {
                        model.addConstr(arrE == depE - consumption, "e_InterStation" + subDep);
                    }
                }
            }
        }

    // overnight charging
        for (int eb : schedulingCfg.EBs) {
            const auto& trips = schedulingCfg.tripEBs[eb];
            int endTripIdx = trips.size() - 1;
            int endTrip = trips[endTripIdx];
            int stationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
            int station = schedulingCfg.stationTrips[endTrip][stationIdx];
            string subEnd = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
            GRBVar endDepE = energyDep_ybns[year][eb][endTripIdx][stationIdx];
            GRBVar nightCharge = chargeNight_yb[year][eb];
            double dist = networkCfg.endDistanceEBs[eb];
            GRBLinExpr consumption = dist * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
            model.addConstr(nightCharge * chargerCfg.chargeEfficiency[0] == E * yearlySocMax - (endDepE - consumption),
                            "e_Overnight_y" + to_string(year) + "_b" + to_string(eb));
        }
    }

    model.update();
    if (buildProblemDataOnly) {
        if (outVars == nullptr || outConstrs == nullptr || outObjSense == nullptr) {
            return false;
        }
        return ExtractProblemDataFromModel(model, *outVars, *outConstrs, *outObjSense);
    }
    // if (allowWarmStart && fs::exists(solutionPath)) {
    //     try {
    //         model.read(solutionPath.string());
    //         cout << "检测到已有解，作为起点继续求解: " << solutionPath.string() << endl;
    //     } catch (const GRBException& e) {
    //         cout << "读取已有解失败，改为重新求解: " << e.getMessage() << endl;
    //     }
    // }

    auto timeRecordFinishModelInit = chrono::high_resolution_clock::now();
    model.optimize();
    auto timeRecordFinishSolve = chrono::high_resolution_clock::now();

    int solveStatus = model.get(GRB_IntAttr_Status);
    if (solveStatus == GRB_OPTIMAL || (solveStatus == GRB_TIME_LIMIT && model.get(GRB_IntAttr_SolCount)>0)) {
        model.write(solutionPath.string());

        fs::path socPath = resultDir / "soc_trace.txt";
        ofstream socFile(socPath);
        if (socFile.is_open()) {
            for (int year = 0; year < batteryLifespan; year++) {
                double yearlyBatteryCapacityFactor = getSoH(year, operationCfg.batteryDegradation);
                double batteryCapacityYear = E.get(GRB_DoubleAttr_X) * yearlyBatteryCapacityFactor; // 年度容量上限
                socFile << "year " << year << ", batteryCapacity " << batteryCapacityYear << "\n";

                for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                    int eb = schedulingCfg.EBs[ebIdx];
                    socFile << "eb " << eb << ": " << operationCfg.socMax << ", ";

                    size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
                    for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                        int trip = schedulingCfg.tripEBs[eb][tripIdx];
                        size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
                        for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                            int station = schedulingCfg.stationTrips[trip][stationIdx];
                            double arrSoc = energyArr_ybns[year][ebIdx][tripIdx][stationIdx].get(GRB_DoubleAttr_X) / batteryCapacityYear;
                            double depSoc = energyDep_ybns[year][ebIdx][tripIdx][stationIdx].get(GRB_DoubleAttr_X) / batteryCapacityYear;
                            socFile << arrSoc << ", ";
                            socFile << depSoc << ", ";
                        }
                    }

                    int endTripIdx = schedulingCfg.tripEBs[eb].size() - 1;
                    int endTrip = schedulingCfg.tripEBs[eb][endTripIdx];
                    int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
                    double distEnd = networkCfg.endDistanceEBs[eb];
                    GRBLinExpr consumptionEnd = distEnd * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
                    double depotArrE = energyDep_ybns[year][ebIdx][endTripIdx][endStationIdx].get(GRB_DoubleAttr_X) - consumptionEnd.getValue();
                    double depotArrSoc = depotArrE / batteryCapacityYear;
                    socFile << depotArrSoc << "\n";
                }
                socFile << "\n";
            }
        } else {
            cout << "写入SoC轨迹文件失败: " << socPath.string() << endl;
        }

        fs::path dischargePath = resultDir / "discharge_events.txt";
        ofstream dischargeFile(dischargePath);
        if (dischargeFile.is_open()) {
            for (int year = 0; year < batteryLifespan; year++) {
                double yearlyBatteryCapacityFactor = getSoH(year, operationCfg.batteryDegradation);
                double batteryCapacityYear = E.get(GRB_DoubleAttr_X) * yearlyBatteryCapacityFactor;
                dischargeFile << "year " << year << ", batteryCapacity " << batteryCapacityYear << "\n";

                for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                    int eb = schedulingCfg.EBs[ebIdx];
                    size_t numEBTrip = schedulingCfg.tripEBs[eb].size();

                    // depot -> first station
                    int startTripIdx = 0;
                    int startTrip = schedulingCfg.tripEBs[eb][startTripIdx];
                    int startStationIdx = 0;
                    int startStation = schedulingCfg.stationTrips[startTrip][startStationIdx];
                    double distStart = networkCfg.startDistanceEBs[eb];
                    GRBLinExpr consumptionStartExpr = distStart * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
                    double arrEStart = energyArr_ybns[year][ebIdx][startTripIdx][startStationIdx].get(GRB_DoubleAttr_X);
                    double depSocStart = operationCfg.socMax; // 出发SoC限制
                    double arrSocStart = arrEStart / batteryCapacityYear;
                    double socAmountStart = consumptionStartExpr.getValue() / batteryCapacityYear;
                    double socMeanStart = 0.5 * (depSocStart + arrSocStart);
                    dischargeFile << socMeanStart << ", " << socAmountStart << "\n";

                    for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                        int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                        size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
                        for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                            int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                            int tripArrIdx;
                            int stationArrIdx;
                            if (stationIdx == numEBTripStations - 1) {
                                if (tripIdx == numEBTrip - 1) {
                                    // last station of last trip handled after loop
                                    continue;
                                }
                                tripArrIdx = tripIdx + 1;
                                stationArrIdx = 0;
                            } else {
                                tripArrIdx = tripIdx;
                                stationArrIdx = stationIdx + 1;
                            }

                            int tripArr = schedulingCfg.tripEBs[eb][tripArrIdx];
                            int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];

                            double depEVal = energyDep_ybns[year][ebIdx][tripIdx][stationIdx].get(GRB_DoubleAttr_X);
                            double arrEVal = energyArr_ybns[year][ebIdx][tripArrIdx][stationArrIdx].get(GRB_DoubleAttr_X);
                            double linkCharge = 0.0;
                            if (isSupportDynamicWireless(modelType)) {
                                linkCharge = charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr].get(GRB_DoubleAttr_X) * chargerCfg.chargeEfficiency[2];
                            }
                            double dischargeEnergy = depEVal - arrEVal + linkCharge; // 等价于路段消耗
                            double socAmount = dischargeEnergy / batteryCapacityYear;
                            double depSoc = depEVal / batteryCapacityYear;
                            double arrSoc = arrEVal / batteryCapacityYear;
                            double socMean = 0.5 * (depSoc + arrSoc);

                            dischargeFile << socMean << ", " << socAmount << "\n";
                        }
                    }

                    // end leg to depot
                    int endTripIdx = schedulingCfg.tripEBs[eb].size() - 1;
                    int endTrip = schedulingCfg.tripEBs[eb][endTripIdx];
                    int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
                    int stationDep = schedulingCfg.stationTrips[endTrip][endStationIdx];
                    double distEnd = networkCfg.endDistanceEBs[eb];
                    GRBLinExpr consumptionEndExpr = distEnd * (operationCfg.curbWeight + E / operationCfg.energyPerWeright) * operationCfg.mu * operationCfg.g / 3.6;
                    double depEVal = energyDep_ybns[year][ebIdx][endTripIdx][endStationIdx].get(GRB_DoubleAttr_X);
                    double depotArrE = depEVal - consumptionEndExpr.getValue();
                    double dischargeEnergy = consumptionEndExpr.getValue();
                    double socAmount = dischargeEnergy / batteryCapacityYear;
                    double depSoc = depEVal / batteryCapacityYear;
                    double arrSoc = depotArrE / batteryCapacityYear;
                    double socMean = 0.5 * (depSoc + arrSoc);
                    dischargeFile << socMean << ", " << socAmount << "\n";
                }
                dischargeFile << "\n";
            }
        } else {
            cout << "写入放电事件文件失败: " << dischargePath.string() << endl;
        }

        if (!allowOutputSolution) {
            return true;
        }
        cout << "\n================ success ================" << endl;
        // for (int year = 0; year < batteryLifespan; year++) {
        //     cout << "\n  第 " << year << " 年数据：" << endl;
        //     cout << "\n================ 每辆车行程详情 ================" << endl;
        //     for (int eb : schedulingCfg.EBs) {
        //         cout << "\n【车辆 EB" << eb << "】" << endl;
        //         cout << "包含行程数量：" << schedulingCfg.tripEBs[eb].size() << endl;

        //         for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[eb].size(); tripIdx++) {
        //             int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
        //             cout << "  行程 " << tripDep << "：" << endl;
        //             cout << "    " << left << setw(16) << "station"
        //                            << setw(8) << "arrT" << setw(8) << "depT"
        //                            << setw(8) << "arrE" << setw(8) << "depE"
        //                            << setw(16) << "stationCharge" << setw(16) << "linkCharge"
        //                            << setw(20) << "stationConsumption" << setw(20) << "linkConsumption" << endl;

        //             for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
        //                 int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
        //                 int timeArr = schedulingCfg.stationArrTimes[tripDep][stationIdx];
        //                 int timeDep = schedulingCfg.stationDepTimes[tripDep][stationIdx];
        //                 GRBVar arrE = energyArr_ybns[year][eb][tripIdx][stationIdx];
        //                 GRBVar depE = energyDep_ybns[year][eb][tripIdx][stationIdx];
        //                 double stationConsumption = 0.0;
        //                 double linkConsumption = 0.0;
        //                 double chargeStation = 0.0;
        //                 double chargeLink = 0.0;
        //                 bool flag = true;

        //                 if ((networkCfg.chargerPhys.count(stationDep))) {
        //                     stationConsumption = charge_ybns[year][eb][tripIdx][stationDep].get(GRB_DoubleAttr_X);
        //                     chargeStation = stationConsumption * chargerCfg.chargeEfficiency[0];
        //                 } else if (isSupportStaticWireless(modelType)) {
        //                     stationConsumption = charge_ybns[year][eb][tripIdx][stationDep].get(GRB_DoubleAttr_X);
        //                     chargeStation = stationConsumption * chargerCfg.chargeEfficiency[1];
        //                 }

        //                 if (isSupportDynamicWireless(modelType)) {
        //                     int tripArr;
        //                     int stationArr;
        //                     if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
        //                         if (tripIdx == schedulingCfg.tripEBs[eb].size() - 1) {
        //                             flag = false;
        //                         } else {
        //                         tripArr = schedulingCfg.tripEBs[eb][tripIdx + 1];
        //                         stationArr = schedulingCfg.stationTrips[tripArr][0];
        //                         }
        //                     } else {
        //                         tripArr = tripDep; stationArr = schedulingCfg.stationTrips[tripArr][stationIdx + 1];
        //                     }
        //                     if (flag) {
        //                         string subLink = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
        //                                          "To_n" + to_string(tripArr) + "_s" + to_string(stationArr);
        //                         linkConsumption = wirelessLink[stationDep][stationArr].get(GRB_DoubleAttr_X);
        //                         chargeLink = linkConsumption * chargerCfg.chargeEfficiency[2];
        //                     }
        //                 }

        //                 string chargeInfo = networkCfg.chargerPhys.count(stationDep) ? "fast" : "wireless";
        //                 cout << "    " << left << setw(16) << to_string(stationDep) + "|" + chargeInfo
        //                                 << setw(8) << timeArr << setw(8) << timeDep
        //                                 << setw(10) << fixed << setprecision(4) << arrE.get(GRB_DoubleAttr_X)
        //                                 << setw(10) << fixed << setprecision(4) << depE.get(GRB_DoubleAttr_X)
        //                                 << setw(18) << fixed << setprecision(4) << chargeStation
        //                                 << setw(18) << fixed << setprecision(4) << chargeLink
        //                                 << setw(20) << fixed << setprecision(4) << stationConsumption
        //                                 << setw(20) << fixed << setprecision(4) << linkConsumption << endl;
        //             }
        //         }
        //         cout << "夜间充电：" << fixed << setprecision(4) <<
        //                 chargeNight_yb[year][eb].get(GRB_DoubleAttr_X) << endl;
        //     }
        // }

        double batteryCapacity = E.get(GRB_DoubleAttr_X);
        double batteryPricePerUnitVal = numEB * operationCfg.priceOfUnitBat * batteryCapacity;

        int fullCyclesSumm = planningYears / batteryLifespan;
        int tailYearsSumm = planningYears % batteryLifespan;
        double sohAtEolSumm = getSoH(batteryLifespan, operationCfg.batteryDegradation);
        double sohTailSumm = (tailYearsSumm > 0) ? getSoH(tailYearsSumm, operationCfg.batteryDegradation) : 0.0;
        int batteriesNeededSumm = fullCyclesSumm + (tailYearsSumm > 0 ? 1 : 0);
        double salvageCapacitySumSumm = fullCyclesSumm * sohAtEolSumm + sohTailSumm;

        double totalBattery = batteryPricePerUnitVal * batteriesNeededSumm;
        double totalBatteryRecycling = batteryPricePerUnitVal * salvageCapacitySumSumm * operationCfg.recyclingPriceRate;
        totalBattery -= totalBatteryRecycling; // deduct recycling salvage
        

        int numTrips = 0;
        for (int eb : schedulingCfg.EBs) {
            numTrips += eb;
        }
        double overNightCharge = 0.f;
        double energyConsumption[] = {0.f, 0.f, 0.f};
        double energyCharging[] = {0.f, 0.f, 0.f};
        for (int year = 0; year < batteryLifespan; year++) {
            double energyConsumptionYearly[] = {0.f, 0.f, 0.f};
            double overNightChargeYearly = 0.f;
            for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
                int eb = schedulingCfg.EBs[ebIdx];
                size_t numEBTrip = schedulingCfg.tripEBs[eb].size();
                for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
                    int tripDep = schedulingCfg.tripEBs[eb][tripIdx];
                    size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
                    for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                        int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                        if (networkCfg.chargerPhys.count(stationDep)) {
                            energyConsumptionYearly[0] += charge_ybns[year][ebIdx][tripIdx][stationDep].get(GRB_DoubleAttr_X);
                        } else if (isSupportStaticWireless(modelType)) {
                            energyConsumptionYearly[1] += charge_ybns[year][ebIdx][tripIdx][stationDep].get(GRB_DoubleAttr_X);
                        }
                        if (isSupportDynamicWireless(modelType)) {
                            int tripArr;
                            int stationArr;
                            if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                                if (tripIdx == schedulingCfg.tripEBs[eb].size() - 1) {
                                    continue;
                                }
                                tripArr = schedulingCfg.tripEBs[eb][tripIdx + 1];
                                stationArr = schedulingCfg.stationTrips[tripArr][0];
                            } else {
                                tripArr = tripDep;
                                stationArr = schedulingCfg.stationTrips[tripArr][stationIdx + 1];
                            }
                            energyConsumptionYearly[2] += charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr].get(GRB_DoubleAttr_X);
                        }
                    }
                }
                overNightChargeYearly += chargeNight_yb[year][eb].get(GRB_DoubleAttr_X);
            }
            if (year < residualLife) {
                overNightCharge += overNightChargeYearly * (batteryRounds + 1);
                energyConsumption[0] += energyConsumptionYearly[0] * (batteryRounds + 1);
                energyConsumption[1] += energyConsumptionYearly[1] * (batteryRounds + 1);
                energyConsumption[2] += energyConsumptionYearly[2] * (batteryRounds + 1);
            } else {
                overNightCharge += overNightChargeYearly * (batteryRounds);
                energyConsumption[0] += energyConsumptionYearly[0] * (batteryRounds);
                energyConsumption[1] += energyConsumptionYearly[1] * (batteryRounds);
                energyConsumption[2] += energyConsumptionYearly[2] * (batteryRounds);
            }
        }
        overNightCharge = overNightCharge * 365;
        energyConsumption[0] = energyConsumption[0] * 365;
        energyConsumption[1] = energyConsumption[1] * 365;
        energyConsumption[2] = energyConsumption[2] * 365;
        energyCharging[0] = energyConsumption[0] * chargerCfg.chargeEfficiency[0];
        energyCharging[1] = energyConsumption[1] * chargerCfg.chargeEfficiency[1];
        energyCharging[2] = energyConsumption[2] * chargerCfg.chargeEfficiency[2];
        double totalEnergy = energyConsumption[0] + energyConsumption[1] + energyConsumption[2] + overNightCharge;
        double totalEnergyCost = (energyConsumption[0] + energyConsumption[1] + energyConsumption[2]) * operationCfg.priceOfPowerDay + overNightCharge * operationCfg.priceOfPowerNight;

        cout << "model: " << modelType << ", planningPeriod:" << planningYears << "years, objective function" << model.get(GRB_DoubleAttr_ObjVal) << endl;
        cout << "EBsNum: " << numEB << ", tripsNum: " << numTrips << ", stationsNum: " << numStations << endl;

        std::cout << "batteryLife: " << batteryLifespan << std::endl;
        for (int i = 0; i < batteryLifespan; ++i) {
            std::cout << operationCfg.batteryDegradation[i] << std::endl;
        }

        cout << "BatteryCapacity E = " << batteryCapacity << ", Total Battery Price" << totalBattery << endl;
        cout << "ChargingEfficiency:" << chargerCfg.chargeEfficiency[0] << ", " << chargerCfg.chargeEfficiency[1] << ", " << chargerCfg.chargeEfficiency[2] << endl;
        cout << "EnergyConsumption: " << energyConsumption[0] << ", " << energyConsumption[1] << ", " << energyConsumption[2] << endl;
        cout << "EnergyCharging: " << energyCharging[0] << ", " << energyCharging[1] << ", " << energyCharging[2] << endl;
        cout << "OvernightCharge: " << overNightCharge << ", Total Energy Consumption: " << totalEnergy << ", Total Energy Cost: " << totalEnergyCost << endl;

        double totalWirelessStations = 0;
        if (isSupportStaticWireless(modelType)) {
            cout << "\n================ Usage of Wireless Charging Stations ================" << endl;
            for (const int& station : networkCfg.stationPhys) {
                if (!networkCfg.chargerPhys.count(station) && (wirelessStation[station].get(GRB_DoubleAttr_X) > 0.5)) {
                    cout << station << "   ";
                    totalWirelessStations += chargerCfg.priceOfUnitCharger[1] * 2;
                    totalWirelessStations += 60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[1] * 2;
                }
            }
            cout << "\n Total Construction Cost of Wireless Stations: " << totalWirelessStations << endl;
        } else {
            cout << "\n no wireless charging station" << endl;
        }

        double totalWirelessLinks = 0;
        if (isSupportDynamicWireless(modelType)) {
            cout << "\n================ Usage of Wireless Charging Links ================" << endl;
            for (const auto& [start, endMap] : networkCfg.linkDistances) {
                for (const auto& [end, _] : endMap) {
                    if (wirelessLink[start][end].get(GRB_DoubleAttr_X) > 0.5) {
                        cout << to_string(start) + " To " + to_string(end) << " : " << setw(4) << networkCfg.linkDistances[start][end] << " km   " << endl;
                        totalWirelessLinks += networkCfg.linkDistances[start][end] * chargerCfg.priceOfUnitCharger[2];
                        totalWirelessLinks += 60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[2];
                    }
                }
            }
            cout << "\n Total Construction Cost of Wireless Links: " << totalWirelessLinks << endl;
        } else {
            cout << "\n no wireless charging link" << endl;
        }
        double totalConstructionCost = totalWirelessStations + totalWirelessLinks;
        cout << "\n cost summary: \n construction: " << totalConstructionCost << ", battery: " << totalBattery
                                                     << ", energy: " << totalEnergyCost
                                                     << ", total: " << (totalConstructionCost + totalEnergyCost + totalBattery) << endl;

        cout << "\n================ summary of perf ================" << endl;
        cout << "init data: " <<
            chrono::duration_cast<chrono::milliseconds>(timeRecordFinishDataInit - timeRecordStart).count() << " ms" << endl;
        cout << "init model: " <<
            chrono::duration_cast<chrono::milliseconds>(timeRecordFinishModelInit - timeRecordFinishDataInit).count() << " ms" << endl;
        cout << "solve: " <<
            chrono::duration_cast<chrono::milliseconds>(timeRecordFinishSolve - timeRecordFinishModelInit).count() << " ms" << endl;

        ofstream summaryFile(summaryPath);
        if (summaryFile.is_open()) {
            summaryFile << "model: " << modelType << "\n";
            summaryFile << "objective: " << model.get(GRB_DoubleAttr_ObjVal) << "\n";
            summaryFile << "batteryCapacity: " << batteryCapacity << "\n";
            int degYears = static_cast<int>(min(operationCfg.batteryLifespan, static_cast<int>(operationCfg.batteryDegradation.size())));
            summaryFile << "degradationRate: ";
            for (int y = 0; y < degYears - 1; ++y) {
                summaryFile << operationCfg.batteryDegradation[y] << ", ";
            }
            summaryFile << operationCfg.batteryDegradation[degYears - 1] << "\n";
            summaryFile << "totalBatteryCost: " << totalBattery << "\n";
            summaryFile << "energyCost: " << totalEnergyCost << "\n";
            summaryFile << "constructionCost: " << totalConstructionCost << "\n";
            summaryFile << "totalCost: " << (totalConstructionCost + totalEnergyCost + totalBattery) << "\n";
            summaryFile << "overnightCharge: " << overNightCharge << "\n";
            summaryFile << "energyConsumptionFast: " << energyConsumption[0] << "\n";
            summaryFile << "energyConsumptionWirelessStation: " << energyConsumption[1] << "\n";
            summaryFile << "energyConsumptionWirelessLink: " << energyConsumption[2] << "\n";
            summaryFile << "initDataMs: " << chrono::duration_cast<chrono::milliseconds>(timeRecordFinishDataInit - timeRecordStart).count() << "\n";
            summaryFile << "initModelMs: " << chrono::duration_cast<chrono::milliseconds>(timeRecordFinishModelInit - timeRecordFinishDataInit).count() << "\n";
            summaryFile << "solveMs: " << chrono::duration_cast<chrono::milliseconds>(timeRecordFinishSolve - timeRecordFinishModelInit).count() << "\n";
        } else {
            cout << "结果摘要文件写入失败: " << summaryPath.string() << endl;
        }

        return true;
    } else if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
        cout << "求解失败，模型不可行，获取ilp中" << endl;
        model.computeIIS();
        model.write("model_iis.ilp");
        return false;
    } else{
        cout << "求解失败，状态码：" << solveStatus << endl;
        return false;
    }
}

WirelessChargingDataInitializationStrategy_Solver::WirelessChargingDataInitializationStrategy_Solver(
    std::string dataFolder)
    : dataFolder(std::move(dataFolder)) {}

void WirelessChargingDataInitializationStrategy_Solver::DataInit(ProblemData& data)
{
    std::string normalizedModelType =
        LoadModelTypeFromConfig(ResolveDataFolderPath(dataFolder) / "config.txt").value();

    vector<ProblemDataVar> vars;
    vector<ProblemDataConstr> constrs;
    int objSense = GRB_MINIMIZE;
    bool built = SolveWithGurobiWirelessChargingStratgiesInternal(normalizedModelType, dataFolder, false, false, 0.02,
        18000.0, true, &vars, &constrs, &objSense);
    if (!built) {
        throw std::runtime_error("Failed to build wireless charging vars/constrs from model.");
    }

    data.addData("vars", vars);
    data.addData("constrs", constrs);
    data.addData("obj", objSense);
    data.addData("wireless_model_type", normalizedModelType);
    data.addData("wireless_data_folder", dataFolder);

    std::cout << "Loaded wireless charging data from file." << std::endl;
    std::cout << "modelType=" << normalizedModelType
              << ", vars=" << vars.size()
              << ", constrs=" << constrs.size() << std::endl;
}

WirelessChargingDataInitializationStrategy_Benders::WirelessChargingDataInitializationStrategy_Benders(
    std::string dataFolder)
    : dataFolder(dataFolder) {}

void WirelessChargingDataInitializationStrategy_Benders::DataInit(ProblemData& data)
{
    std::string normalizedModelType =
        LoadModelTypeFromConfig(ResolveDataFolderPath(dataFolder) / "config.txt").value();

    vector<ProblemDataVar> vars;
    vector<ProblemDataConstr> constrs;
    int objSense = GRB_MINIMIZE;
    bool built = SolveWithGurobiWirelessChargingStratgiesInternal(normalizedModelType, dataFolder, false, false, 0,
        18000.0, true, &vars, &constrs, &objSense);
    if (!built) {
        throw std::runtime_error("Failed to build wireless charging model data for Benders.");
    }

    vector<int> masterIndices;
    vector<ProblemDataVar> masterVars;
    masterIndices.reserve(vars.size());
    masterVars.reserve(vars.size());

    for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
        if (IsWirelessMasterVarName(vars[static_cast<size_t>(i)].name)) {
            masterIndices.push_back(i);
            masterVars.push_back(vars[static_cast<size_t>(i)]);
        }
    }

    data.addData("masterVars", masterVars);
    data.addData("wireless_master_indices", masterIndices);
    data.addData("wireless_all_vars", vars);
    data.addData("wireless_all_constrs", constrs);
    data.addData("wireless_obj_sense", objSense);
    data.addData("wireless_model_type", normalizedModelType);
    data.addData("wireless_data_folder", dataFolder);

    std::cout << "Loaded wireless charging data for Benders decomposition." << std::endl;
    std::cout << "modelType=" << normalizedModelType
              << ", masterVars=" << masterVars.size()
              << ", subVars=" << vars.size()
              << ", subConstrs=" << constrs.size() << std::endl;
}

std::vector<ProblemDataConstr> WirelessChargingDataInitializationStrategy_Benders::ConstrInit(ProblemData& data)
{
    const auto& allConstrs = data.getData<std::vector<ProblemDataConstr>>("wireless_all_constrs");
    const auto& masterIndices = data.getData<std::vector<int>>("wireless_master_indices");

    std::vector<ProblemDataConstr> masterConstrs;

    for (const auto& constr : allConstrs) {
        if (constr.name != "constrEChooseOne") {
            continue;
        }

        ProblemDataConstr projected;
        projected.coeffs.assign(masterIndices.size(), 0.0);
        projected.sense = constr.sense;
        projected.rhs = constr.rhs;
        projected.name = constr.name;

        for (size_t k = 0; k < masterIndices.size(); ++k) {
            int fullIdx = masterIndices[k];
            projected.coeffs[k] = constr.coeffs[static_cast<size_t>(fullIdx)];
        }

        masterConstrs.push_back(projected);
        break;
    }

    return masterConstrs;
}

void WirelessChargingSubProblemStrategy_Benders::InitSubProblem(
    const ProblemData& problemData,
    GRBModel& subModel,
    BendersSubProblemContext& context)
{
    const auto& vars = problemData.getData<std::vector<ProblemDataVar>>("wireless_all_vars");
    const auto& constrs = problemData.getData<std::vector<ProblemDataConstr>>("wireless_all_constrs");
    const auto& masterIndices = problemData.getData<std::vector<int>>("wireless_master_indices");
    int objSense = problemData.getData<int>("wireless_obj_sense");

    context.Clear();
    auto& allVars = context.EnsureVarGroup(kWirelessVarGroupAllVars);
    allVars.reserve(vars.size());

    subModel.set(GRB_IntParam_InfUnbdInfo, 1);
    subModel.set(GRB_IntParam_DualReductions, 0);

    std::vector<double> objCoeffs(vars.size(), 0.0);
    for (size_t i = 0; i < vars.size(); ++i) {
        objCoeffs[i] = vars[i].obj;
    }
    for (int idx : masterIndices) {
        objCoeffs[static_cast<size_t>(idx)] = 0.0;
    }

    for (size_t i = 0; i < vars.size(); ++i) {
        const auto& var = vars[i];
        allVars.push_back(subModel.addVar(var.lb, var.ub, objCoeffs[i], var.type, var.name));
    }

    for (const auto& constr : constrs) {
        if (constr.coeffs.size() != allVars.size()) {
            throw std::runtime_error("Wireless sub-problem constraint size mismatch.");
        }
        GRBLinExpr lhs = 0.0;
        for (size_t i = 0; i < allVars.size(); ++i) {
            if (std::abs(constr.coeffs[i]) > 1e-12) {
                lhs += constr.coeffs[i] * allVars[i];
            }
        }
        subModel.addConstr(lhs, constr.sense, constr.rhs, constr.name);
    }

    GRBLinExpr objective = 0.0;
    for (size_t i = 0; i < vars.size(); ++i) {
        if (std::abs(objCoeffs[i]) > 1e-12) {
            objective += objCoeffs[i] * allVars[i];
        }
    }
    subModel.setObjective(objective, objSense);
    subModel.update();

    lowerBoundReady = false;
    qLowerBound = 0.0;
    lowerBoundCutAdded = false;

    subModel.optimize();
    int lbStatus = subModel.get(GRB_IntAttr_Status);
    if (lbStatus == GRB_OPTIMAL || (lbStatus == GRB_TIME_LIMIT && subModel.get(GRB_IntAttr_SolCount) > 0)) {
        qLowerBound = subModel.get(GRB_DoubleAttr_ObjVal);
        lowerBoundReady = true;
    } else {
        std::cerr << "Wireless Benders: failed to pre-compute recourse lower bound, status="
                  << lbStatus << ". fallback lower bound = 0." << std::endl;
    }
}

void WirelessChargingSubProblemStrategy_Benders::UpdateSubProblem(
    const ProblemData& problemData,
    GRBModel& subModel,
    BendersSubProblemContext& context,
    const std::vector<double>& yValues)
{
    const auto& masterIndices = problemData.getData<std::vector<int>>("wireless_master_indices");
    auto& allVars = context.EnsureVarGroup(kWirelessVarGroupAllVars);

    for (size_t i = 0; i < masterIndices.size(); ++i) {
        int varIdx = masterIndices[i];
        double fixedValue = (yValues[i] > 0.5) ? 1.0 : 0.0;
        allVars[static_cast<size_t>(varIdx)].set(GRB_DoubleAttr_LB, fixedValue);
        allVars[static_cast<size_t>(varIdx)].set(GRB_DoubleAttr_UB, fixedValue);
    }

    subModel.update();
}

Status WirelessChargingSubProblemStrategy_Benders::SolveSubProblem(
    const ProblemData& problemData,
    GRBModel& subModel,
    BendersSubProblemContext& context,
    const std::vector<double>& yValues,
    BendersCutInfo& cutInfo,
    double& subObj)
{
    subModel.optimize();
    int status = subModel.get(GRB_IntAttr_Status);

    int selectedCount = 0;
    for (double value : yValues) {
        if (value > 0.5) {
            ++selectedCount;
        }
    }

    if (status == GRB_OPTIMAL || (status == GRB_TIME_LIMIT && subModel.get(GRB_IntAttr_SolCount) > 0)) {
        subObj = subModel.get(GRB_DoubleAttr_ObjVal);

        if (lowerBoundReady && !lowerBoundCutAdded) {
            cutInfo.isOptimalityCut = true;
            cutInfo.sense = '>';
            cutInfo.rhs = 0.0;
            cutInfo.constant = qLowerBound;
            cutInfo.yCoeffs.assign(yValues.size(), 0.0);
            lowerBoundCutAdded = true;
            return OK;
        }

        double lowerBound = lowerBoundReady ? qLowerBound : 0.0;
        double delta = std::max(0.0, subObj - lowerBound);
        cutInfo.isOptimalityCut = true;
        cutInfo.sense = '>';
        cutInfo.rhs = 0.0;
        cutInfo.constant = lowerBound + delta * (1.0 - static_cast<double>(selectedCount));
        cutInfo.yCoeffs.assign(yValues.size(), 0.0);

        for (size_t i = 0; i < yValues.size(); ++i) {
            if (yValues[i] > 0.5) {
                cutInfo.yCoeffs[i] = delta;
            } else {
                cutInfo.yCoeffs[i] = -delta;
            }
        }
        return OK;
    }

    if (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED) {
        cutInfo.isOptimalityCut = false;
        cutInfo.sense = '>';
        cutInfo.constant = 0.0;
        cutInfo.yCoeffs.assign(yValues.size(), 0.0);

        for (size_t i = 0; i < yValues.size(); ++i) {
            if (yValues[i] > 0.5) {
                cutInfo.yCoeffs[i] = -1.0;
            } else {
                cutInfo.yCoeffs[i] = 1.0;
            }
        }

        cutInfo.rhs = 1.0 - static_cast<double>(selectedCount);
        subObj = GRB_INFINITY;
        return OK;
    }

    std::cerr << "Wireless Benders sub-problem ended with unexpected status: " << status << std::endl;
    return ERROR;
}

