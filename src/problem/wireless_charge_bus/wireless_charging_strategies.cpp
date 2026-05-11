#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <memory>
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

using linkTimeType = unordered_map<int, unordered_map<int, double>>;
using stationTimeType = vector<vector<int>>;
using stringCfgType = vector<vector<string>>;

namespace {
inline fs::path ResolveDataFolderPath(const string& dataFolder) {
    return fs::path(__FILE__).parent_path() / dataFolder;
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

inline double getSoH(const int& y, const vector<double>& batteryDegradation) {
    double soh = 1.0;
    for (int i = 0; i < y && i < batteryDegradation.size(); ++i) {
        soh *= (1 - batteryDegradation[i]);
    }
    return soh;
}

constexpr const char* kWirelessVarGroupAllVars = "wireless_all_vars";
constexpr const char* kWirelessBatteryChoicePrefix = "E_choice_";

static bool StartsWith(const string& value, const string& prefix)
{
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

static bool TryParseBatteryChoiceCapacity(const string& varName, double& capacity)
{
    if (!StartsWith(varName, kWirelessBatteryChoicePrefix)) {
        return false;
    }

    const string suffix = varName.substr(string(kWirelessBatteryChoicePrefix).size());
    if (suffix.empty()) {
        return false;
    }

    try {
        capacity = stod(suffix);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

} // namespace

struct OperationCfg {
    double priceOfPowerDay; // CNY per kW
    double priceOfPowerNight; // CNY per kW
    double priceOfUnitBat; // CNY per kW.h
    double socMax;
    double socMin;
    double capacityMin;
    double capacityMax;
    double curbWeight;
    double weightPerEnergy;
    double mu;
    double g;
    int timeMin;
    int timeMax;
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

        auto tmpMiscCfg = LoadStringTable(Resolve("miscCfg.txt"));
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
            operationCfg.curbWeight = stod(operationCfgBus[0]);
            operationCfg.weightPerEnergy = stod(operationCfgBus[1]);
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

static double CalcWirelessBatteryChoiceObjCoeff(const ModelData& data)
{
    const OperationCfg& operationCfg = data.GetOperationCfg();
    const SchedulingCfg& schedulingCfg = data.GetSchedulingCfg();

    auto sohAfterYears = [&](int years) {
        return getSoH(years, operationCfg.batteryDegradation);
    };

    const int fullCycles = operationCfg.planningYears / operationCfg.batteryLifespan;
    const int tailYears = operationCfg.planningYears % operationCfg.batteryLifespan;
    const double sohAtEol = sohAfterYears(operationCfg.batteryLifespan);
    const double sohTail = (tailYears > 0) ? sohAfterYears(tailYears) : 0.0;
    const int batteriesNeeded = fullCycles + (tailYears > 0 ? 1 : 0);
    const double salvageCapacitySum = fullCycles * sohAtEol + sohTail;

    return schedulingCfg.numEB * operationCfg.priceOfUnitBat *
        (static_cast<double>(batteriesNeeded) - salvageCapacitySum * operationCfg.recyclingPriceRate);
}

static vector<ProblemDataVar> BuildWirelessMasterVars(const ModelData& data)
{
    const OperationCfg& operationCfg = data.GetOperationCfg();
    const ChargerCfg& chargerCfg = data.GetChargerCfg();
    const NetworkCfg& networkCfg = data.GetNetworkCfg();

    vector<ProblemDataVar> masterVars;
    const double batteryChoiceObjCoeff = CalcWirelessBatteryChoiceObjCoeff(data);
    for (double cap = operationCfg.capacityMin; cap <= operationCfg.capacityMax; cap += 10.0) {
        if (cap <= 0.0) {
            continue;
        }

        ProblemDataVar var;
        var.lb = 0.0;
        var.ub = 1.0;
        var.obj = batteryChoiceObjCoeff * cap;
        var.type = GRB_BINARY;
        var.name = "E_choice_" + to_string(static_cast<int>(cap));
        masterVars.push_back(var);
    }

    vector<int> stations(networkCfg.stationPhys.begin(), networkCfg.stationPhys.end());
    sort(stations.begin(), stations.end());
    const double stationCost =
        (60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[1] + chargerCfg.priceOfUnitCharger[1]) * 2;
    for (int station : stations) {
        if (networkCfg.chargerPhys.count(station)) {
            continue;
        }

        ProblemDataVar var;
        var.lb = 0.0;
        var.ub = 1.0;
        var.obj = stationCost;
        var.type = GRB_BINARY;
        var.name = "w_station_s" + to_string(station);
        masterVars.push_back(var);
    }

    vector<pair<int, int>> links;
    for (const auto& [start, endMap] : networkCfg.linkDistances) {
        for (const auto& [end, _] : endMap) {
            links.emplace_back(start, end);
        }
    }
    sort(links.begin(), links.end());

    for (const auto& [start, end] : links) {
        const double dist = networkCfg.linkDistances.at(start).at(end);
        ProblemDataVar var;
        var.lb = 0.0;
        var.ub = 1.0;
        var.obj = chargerCfg.priceOfUnitCharger[2] * dist +
            60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[2];
        var.type = GRB_BINARY;
        var.name = "w_link_l" + to_string(start) + "To" + to_string(end);
        masterVars.push_back(var);
    }

    return masterVars;
}

static void InitWirelessMasterData(ProblemData& data, const string& dataFolder, const string& usageLabel)
{
    ModelData modelData(dataFolder);
    vector<ProblemDataVar> masterVars = BuildWirelessMasterVars(modelData);

    data.addData("masterVars", masterVars);
    data.addData("wireless_data_folder", dataFolder);
    data.addData("wireless_scenario_count", 1);

    std::cout << "Loaded wireless charging data for " << usageLabel << "." << std::endl;
    std::cout << "scenarios=1"
              << ", masterVars=" << masterVars.size() << std::endl;
}

static std::vector<ProblemDataConstr> BuildWirelessMasterConstrs(const ProblemData& data)
{
    const auto& masterVars = data.getData<std::vector<ProblemDataVar>>("masterVars");

    ProblemDataConstr capacityPick;
    capacityPick.coeffs.assign(masterVars.size(), 0.0);
    capacityPick.sense = '=';
    capacityPick.rhs = 1.0;
    capacityPick.name = "constrEChooseOne";

    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        if (StartsWith(masterVars[idx].name, kWirelessBatteryChoicePrefix)) {
            capacityPick.coeffs[idx] = 1.0;
        }
    }

    return {capacityPick};
}

static void BuildWirelessNoGoodFeasibilityCut(
    const vector<double>& zValues, IntegerLShapedCutInfo& cutInfo)
{
    int selectedCount = 0;
    for (double value : zValues) {
        if (value > 0.5) {
            ++selectedCount;
        }
    }

    cutInfo.yCoeffs.assign(zValues.size(), 0.0);
    for (size_t i = 0; i < zValues.size(); ++i) {
        cutInfo.yCoeffs[i] = (zValues[i] > 0.5) ? -1.0 : 1.0;
    }
    cutInfo.rhs = 1.0 - static_cast<double>(selectedCount);
    cutInfo.isOptimalityCut = false;
    cutInfo.sense = '>';
    cutInfo.constant = 0.0;
}

static Status BuildWirelessFarkasFeasibilityCut(GRBModel& relaxedModel,
    const vector<GRBConstr>& relaxedFixConstrs, const vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo)
{
    try {
        const double beta = relaxedModel.get(GRB_DoubleAttr_FarkasProof);
        if (!std::isfinite(beta) || beta <= 1e-9) {
            BuildWirelessNoGoodFeasibilityCut(zValues, cutInfo);
            return OK;
        }

        cutInfo.isOptimalityCut = false;
        cutInfo.sense = '>';
        cutInfo.rhs = beta;
        cutInfo.yCoeffs.assign(zValues.size(), 0.0);
        cutInfo.constant = 0.0;

        for (size_t i = 0; i < zValues.size(); ++i) {
            const double lambda = relaxedFixConstrs[i].get(GRB_DoubleAttr_FarkasDual);
            cutInfo.yCoeffs[i] = lambda;
            cutInfo.constant -= lambda * zValues[i];
        }

        return OK;
    } catch (const GRBException&) {
        BuildWirelessNoGoodFeasibilityCut(zValues, cutInfo);
        return OK;
    }
}

static Status SolveWirelessRelaxedForFarkas(GRBModel& relaxedModel)
{
    try {
        relaxedModel.set(GRB_IntParam_Method, 1);
        relaxedModel.set(GRB_IntParam_Crossover, 1);
        relaxedModel.optimize();
        const int status = relaxedModel.get(GRB_IntAttr_Status);
        relaxedModel.set(GRB_IntParam_Method, 2);
        relaxedModel.set(GRB_IntParam_Crossover, 0);
        return (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED) ? OK : ERROR;
    } catch (const GRBException&) {
        relaxedModel.set(GRB_IntParam_Method, 2);
        relaxedModel.set(GRB_IntParam_Crossover, 0);
        return ERROR;
    }
}

struct WirelessGurobiOptions {
    double mipGap = -1.0;
    double timeLimit = 18000.0;
    bool includeFirstStageObjective = true;
    bool configureSolverParameters = true;
};

static bool BuildWirelessChargingModelInternal(const string& dataFolder, GRBModel& model,
    const WirelessGurobiOptions& options)
{
    ModelData data(dataFolder);

    OperationCfg operationCfg = data.GetOperationCfg();
    ChargerCfg chargerCfg = data.GetChargerCfg();
    SchedulingCfg schedulingCfg = data.GetSchedulingCfg();
    NetworkCfg networkCfg = data.GetNetworkCfg();
    int planningYears = operationCfg.planningYears;
    int batteryLifespan = operationCfg.batteryLifespan;
    int batteryRounds = planningYears / batteryLifespan ;
    int residualLife = planningYears - batteryRounds * batteryLifespan;

    if (options.configureSolverParameters) {
        if (options.mipGap >= 0.0) {
            model.set(GRB_DoubleParam_MIPGap, options.mipGap);
        }
        model.set(GRB_DoubleParam_TimeLimit, options.timeLimit);
        model.set(GRB_IntParam_Presolve, 2);
        model.set(GRB_IntParam_MIPFocus, 1);
        model.set(GRB_DoubleParam_Heuristics, 0.5);
        model.set(GRB_IntParam_Threads, 14);
        model.set(GRB_IntParam_Method, 1);
    }
    // decision variable
    GRBVar E = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "E");

    vector<double> batteryOptions;
    for (double cap = operationCfg.capacityMin; cap <= operationCfg.capacityMax; cap += 10.0) {
        if (cap <= 0.0) {
            continue;
        }
        batteryOptions.push_back(cap);
    }

    vector<GRBVar> batteryChoice;
    batteryChoice.reserve(batteryOptions.size());
    for (double cap : batteryOptions) {
        string varName = "E_choice_" + to_string(static_cast<int>(cap));
        batteryChoice.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, varName));
    }
    vector<vector<GRBVar>> chargeNight_yb; // [year][eb]
    unordered_map<int, GRBVar> wirelessStation;
    unordered_map<int, unordered_map<int, GRBVar>> wirelessLink;
    vector<vector<vector<unordered_map<int, GRBVar>>>> charge_ybns; // [year][eb][trip][station]
    vector<vector<vector<unordered_map<int, unordered_map<int, GRBVar>>>>> charge_ybnl; // M3/M4: [year][eb][trip][start][end]
    vector<vector<vector<vector<GRBVar>>>> energyDep_ybns; // [year][eb][trip][station]
    vector<vector<vector<vector<GRBVar>>>> energyArr_ybns; // [year][eb][trip][station]
    vector<unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, GRBVar>>>>> x_ybns_t; // [year][eb][trip][station][time]

    size_t numEB = schedulingCfg.numEB;

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
                    charge_ybn[station] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varNameCharge);
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
    for (const int& station : networkCfg.stationPhys) {
        if (!networkCfg.chargerPhys.count(station)) {
            string varName = "w_station_s" + to_string(station);
            wirelessStation[station] = model.addVar(0.0, 1, 0.0, GRB_BINARY, varName);
        }
    }

    // dynamic wireless charging
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

    // dynamic wireless charging facility
    for (const auto& [start, endMap] : networkCfg.linkDistances) {
        for (const auto& [end, _] : endMap) {
            string varName = "w_link_l" + to_string(start) + "To" + to_string(end);
            wirelessLink[start][end] = model.addVar(0.0, 1, 0.0, GRB_BINARY, varName);
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

    const double batteryChoiceObjCoeff = numEB * operationCfg.priceOfUnitBat *
        (static_cast<double>(batteriesNeeded) - salvageCapacitySum * operationCfg.recyclingPriceRate);
    for (size_t i = 0; i < batteryChoice.size(); ++i) {
        const double coeff = batteryChoiceObjCoeff * batteryOptions[i];
        if (options.includeFirstStageObjective) {
            batteryChoice[i].set(GRB_DoubleAttr_Obj, coeff);
            obj += coeff * batteryChoice[i];
        } else {
            batteryChoice[i].set(GRB_DoubleAttr_Obj, 0.0);
        }
    }

    if (options.includeFirstStageObjective) {
        GRBLinExpr sum_station_charger = 0;
        for (const auto& [_, wirelessStationVar] : wirelessStation) {
            sum_station_charger += wirelessStationVar;
        }

        obj += (60 * chargerCfg.priceOfInverterPerkW * chargerCfg.chargeRate[1] + chargerCfg.priceOfUnitCharger[1]) * sum_station_charger * 2;

        // cost of constructing dynamic wireless charging facility
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

    // c_bnl in dynamic wireless charging links
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
            GRBLinExpr consumptionStart = distStart * (operationCfg.curbWeight + E * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
            GRBVar startEnergy = energyArr_ybns[year][eb][startTripIdx][startStationIdx];
            model.addConstr(startEnergy == E * yearlySocMax - consumptionStart, "e_Start" + subStart);

            int endTripIdx = trips.size() - 1;
            int endTrip = trips[endTripIdx];
            int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
            int endStation = schedulingCfg.stationTrips[endTrip][endStationIdx];
            string subEnd = "_y" + to_string(year) + "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(endStation) + "_i" + to_string(endStationIdx);
            double distEnd = networkCfg.endDistanceEBs[eb];
            GRBLinExpr consumptionEnd = distEnd * (operationCfg.curbWeight + E * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
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
                        GRBVar charge = charge_ybns[year][ebIdx][tripIdx][station];
                        model.addConstr(depE == arrE + charge * chargerCfg.chargeEfficiency[1], "e_InnerStation" + sub);
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
                    GRBLinExpr consumption = dist * (operationCfg.curbWeight + E * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
                    GRBVar linkCharge = charge_ybnl[year][ebIdx][tripIdx][stationDep][stationArr];
                    model.addConstr(arrE == depE - consumption + linkCharge * chargerCfg.chargeEfficiency[2],
                                    "e_InterStation" + subDep);
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
            GRBLinExpr consumption = dist * (operationCfg.curbWeight + E * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
            model.addConstr(nightCharge * chargerCfg.chargeEfficiency[0] == E * yearlySocMax - (endDepE - consumption),
                            "e_Overnight_y" + to_string(year) + "_b" + to_string(eb));
        }
    }

    model.update();
    return true;
}

static bool BuildWirelessChargingSolverModel(const string& dataFolder, GRBModel& model)
{
    WirelessGurobiOptions options;
    options.timeLimit = 18000.0;
    options.includeFirstStageObjective = true;
    options.configureSolverParameters = true;
    return BuildWirelessChargingModelInternal(dataFolder, model, options);
}

static bool BuildWirelessChargingLShapedSubProblem(
    const string& dataFolder, const vector<ProblemDataVar>& masterVars,
    GRBModel& subModel, IntegerLShapedSubProblemContext& context)
{
    context.Clear();
    auto& masterVarRefs = context.EnsureVarGroup(kWirelessVarGroupAllVars);
    masterVarRefs.reserve(masterVars.size());

    WirelessGurobiOptions options;
    options.mipGap = 0.0;
    options.timeLimit = 18000.0;
    options.includeFirstStageObjective = false;
    options.configureSolverParameters = false;

    if (!BuildWirelessChargingModelInternal(dataFolder, subModel, options)) {
        return false;
    }

    subModel.set(GRB_IntParam_InfUnbdInfo, 1);
    subModel.set(GRB_IntParam_DualReductions, 0);
    subModel.set(GRB_IntParam_OutputFlag, 0);
    subModel.set(GRB_DoubleParam_MIPGap, 0.0);
    subModel.set(GRB_DoubleParam_TimeLimit, 18000.0);
    subModel.set(GRB_IntParam_Presolve, 2);
    subModel.set(GRB_IntParam_Method, 1);

    for (const auto& varData : masterVars) {
        GRBVar var = subModel.getVarByName(varData.name);
        masterVarRefs.push_back(var);
    }

    subModel.update();
    return true;
}

WirelessChargingDataInitializationStrategy_Solver::WirelessChargingDataInitializationStrategy_Solver(
    std::string dataFolder)
    : dataFolder(std::move(dataFolder)) {}

Status WirelessChargingDataInitializationStrategy_Solver::DataInit(GRBModel& model)
{
    std::cout << "Loaded wireless charging data from file." << std::endl;

    bool built = BuildWirelessChargingSolverModel(dataFolder, model);
    return built ? OK : ERROR;
}

WirelessChargingDataInitializationStrategy_LShaped::WirelessChargingDataInitializationStrategy_LShaped(
    std::string dataFolder)
    : dataFolder(std::move(dataFolder)) {}

void WirelessChargingDataInitializationStrategy_LShaped::DataInit(ProblemData& data)
{
    InitWirelessMasterData(data, dataFolder, "integer L-shaped decomposition");
}

std::vector<ProblemDataConstr> WirelessChargingDataInitializationStrategy_LShaped::ConstrInit(ProblemData& data)
{
    return BuildWirelessMasterConstrs(data);
}

std::vector<double> WirelessChargingDataInitializationStrategy_LShaped::BuildWarmStartMasterValues(
    const ProblemData& data) const
{
    const auto& masterVars = data.getData<std::vector<ProblemDataVar>>("masterVars");
    std::vector<double> warmStart(masterVars.size(), 0.0);

    int maxCapacityIdx = -1;
    double maxCapacity = -1.0;
    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        double capacity = 0.0;
        if (TryParseBatteryChoiceCapacity(masterVars[idx].name, capacity)) {
            if (capacity > maxCapacity) {
                maxCapacity = capacity;
                maxCapacityIdx = static_cast<int>(idx);
            }
            continue;
        }

        warmStart[idx] = 1.0;
    }

    if (maxCapacityIdx < 0) {
        throw std::runtime_error("Wireless L-shaped warm start requires at least one battery choice variable.");
    }

    warmStart[static_cast<size_t>(maxCapacityIdx)] = 1.0;
    return warmStart;
}

bool WirelessChargingDataInitializationStrategy_LShaped::IsWarmStartMasterFeasible(const ProblemData& data,
    const std::vector<double>& zValues, double tolerance) const
{
    return true;
}

void WirelessChargingSubProblemStrategy_LShaped::InitSubProblem(const ProblemData& problemData, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context)
{
    const auto& dataFolder = problemData.getData<std::string>("wireless_data_folder");
    const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");

    bool built = BuildWirelessChargingLShapedSubProblem(dataFolder, masterVars, subModel, context);
    if (!built) {
        throw std::runtime_error("Failed to build wireless charging L-shaped sub-problem.");
    }

    lowerBoundReady = false;
    qLowerBound = 0.0;
    lowerBoundCutAdded = false;

    subModel.optimize();
    int lbStatus = subModel.get(GRB_IntAttr_Status);
    if (lbStatus == GRB_OPTIMAL || (lbStatus == GRB_TIME_LIMIT && subModel.get(GRB_IntAttr_SolCount) > 0)) {
        qLowerBound = subModel.get(GRB_DoubleAttr_ObjVal);
        lowerBoundReady = true;
    } else {
        std::cerr << "Wireless L-shaped: failed to pre-compute recourse lower bound, status="
                  << lbStatus << ". fallback lower bound = 0." << std::endl;
    }

    InitRelaxedSubProblem(problemData, subModel);
}

void WirelessChargingSubProblemStrategy_LShaped::UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues)
{
    const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");
    auto& masterVarRefs = context.EnsureVarGroup(kWirelessVarGroupAllVars);
    if (masterVarRefs.size() != masterVars.size() || zValues.size() != masterVars.size()) {
        throw std::runtime_error("Wireless L-shaped master variable context size mismatch.");
    }

    for (size_t i = 0; i < masterVars.size(); ++i) {
        double fixedValue = (zValues[i] > 0.5) ? 1.0 : 0.0;
        masterVarRefs[i].set(GRB_DoubleAttr_LB, fixedValue);
        masterVarRefs[i].set(GRB_DoubleAttr_UB, fixedValue);
    }

    subModel.update();
}

void WirelessChargingSubProblemStrategy_LShaped::InitRelaxedSubProblem(
    const ProblemData& problemData, GRBModel& subModel)
{
    const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");

    relaxedModel = std::make_unique<GRBModel>(subModel.relax());
    relaxedModel->set(GRB_IntParam_OutputFlag, 0);
    relaxedModel->set(GRB_IntParam_InfUnbdInfo, 1);
    relaxedModel->set(GRB_IntParam_DualReductions, 0);
    relaxedModel->set(GRB_IntParam_Method, 2);
    relaxedModel->set(GRB_IntParam_Crossover, 0);

    relaxedMasterVars.clear();
    relaxedFixConstrs.clear();
    relaxedMasterVars.reserve(masterVars.size());
    relaxedFixConstrs.reserve(masterVars.size());

    for (size_t i = 0; i < masterVars.size(); ++i) {
        const auto& varData = masterVars[i];
        GRBVar yVar = relaxedModel->getVarByName(varData.name);
        yVar.set(GRB_DoubleAttr_LB, -GRB_INFINITY);
        yVar.set(GRB_DoubleAttr_UB, GRB_INFINITY);

        const std::string constrName = "wireless_relax_fix_" + std::to_string(i);
        GRBConstr fixConstr = relaxedModel->addConstr(yVar == 0.0, constrName);
        relaxedMasterVars.push_back(yVar);
        relaxedFixConstrs.push_back(fixConstr);
    }

    relaxedModel->update();
}

Status WirelessChargingSubProblemStrategy_LShaped::SolveRelaxedModel(
    const ProblemData& problemData, const std::vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo, double& subObj)
{
    try {
        const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");
        if (zValues.size() != masterVars.size() ||
            relaxedMasterVars.size() != masterVars.size() ||
            relaxedFixConstrs.size() != masterVars.size() ||
            !relaxedModel) {
            std::cerr << "Wireless relaxed sub-problem: relaxed model is not initialized" << std::endl;
            return ERROR;
        }

        for (size_t i = 0; i < zValues.size(); ++i) {
            relaxedMasterVars[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
            relaxedMasterVars[i].set(GRB_DoubleAttr_UB, GRB_INFINITY);
            relaxedFixConstrs[i].set(GRB_DoubleAttr_RHS, zValues[i]);
        }

        relaxedModel->update();
        relaxedModel->optimize();

        const int status = relaxedModel->get(GRB_IntAttr_Status);
        if (status == GRB_OPTIMAL) {
            subObj = relaxedModel->get(GRB_DoubleAttr_ObjVal);

            cutInfo.isOptimalityCut = true;
            cutInfo.sense = '>';
            cutInfo.rhs = 0.0;
            cutInfo.yCoeffs.assign(zValues.size(), 0.0);
            cutInfo.constant = subObj;

            for (size_t i = 0; i < relaxedFixConstrs.size(); ++i) {
                const double pi = relaxedFixConstrs[i].get(GRB_DoubleAttr_Pi);
                cutInfo.yCoeffs[i] = pi;
                cutInfo.constant -= pi * zValues[i];
            }
            return OK;
        }

        if (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED) {
            Status certStatus = SolveWirelessRelaxedForFarkas(*relaxedModel);
            if (certStatus == OK) {
                Status cutStatus = BuildWirelessFarkasFeasibilityCut(*relaxedModel, relaxedFixConstrs, zValues, cutInfo);
                if (cutStatus != OK) {
                    return cutStatus;
                }
            } else {
                BuildWirelessNoGoodFeasibilityCut(zValues, cutInfo);
            }
            subObj = GRB_INFINITY;
            return OK;
        }

        std::cerr << "Wireless relaxed sub-problem ended with unexpected status: " << status << std::endl;
        return ERROR;
    } catch (const GRBException& e) {
        std::cerr << "Wireless relaxed sub-problem failed: " << e.getMessage() << std::endl;
        return ERROR;
    } catch (const std::exception& e) {
        std::cerr << "Wireless relaxed sub-problem failed: " << e.what() << std::endl;
        return ERROR;
    }
}

Status WirelessChargingSubProblemStrategy_LShaped::SolveSubProblem(const ProblemData& problemData, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo, double& subObj)
{
    subModel.optimize();
    int status = subModel.get(GRB_IntAttr_Status);

    int selectedCount = 0;
    for (double value : zValues) {
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
            cutInfo.yCoeffs.assign(zValues.size(), 0.0);
            lowerBoundCutAdded = true;
            return OK;
        }

        double lowerBound = lowerBoundReady ? qLowerBound : 0.0;
        double delta = std::max(0.0, subObj - lowerBound);
        cutInfo.isOptimalityCut = true;
        cutInfo.sense = '>';
        cutInfo.rhs = 0.0;
        cutInfo.constant = lowerBound + delta * (1.0 - static_cast<double>(selectedCount));
        cutInfo.yCoeffs.assign(zValues.size(), 0.0);

        for (size_t i = 0; i < zValues.size(); ++i) {
            cutInfo.yCoeffs[i] = (zValues[i] > 0.5) ? delta : -delta;
        }
        return OK;
    }

    if (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED) {
        BuildWirelessNoGoodFeasibilityCut(zValues, cutInfo);
        subObj = GRB_INFINITY;
        return OK;
    }

    std::cerr << "Wireless L-shaped sub-problem ended with unexpected status: " << status << std::endl;
    return ERROR;
}

Status WirelessChargingSubProblemStrategy_LShaped::SolveRelaxedSubProblem(const ProblemData& problemData, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo, double& subObj)
{
    UpdateSubProblem(problemData, subModel, context, zValues);
    return SolveRelaxedModel(problemData, zValues, cutInfo, subObj);
}

