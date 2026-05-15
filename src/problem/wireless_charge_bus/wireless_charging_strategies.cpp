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
#define OFFSET_CHARGING_CFG (OFFSET_OPERATION_CFG + 3)
#define OFFSET_SCENARIO_NETWORK_CFG 0
#define OFFSET_SCENARIO_SCHEDULING_CFG (OFFSET_SCENARIO_NETWORK_CFG + 2)
#define OFFSET_SCENARIO_CHARGER_CFG(numEB) (OFFSET_SCENARIO_SCHEDULING_CFG + 3 + (numEB))

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

constexpr double kWirelessEnergySlackPenalty = 1e6;
constexpr const char* kWirelessFacilityRhsConstrGroup = "wireless_facility_rhs_constraints";
constexpr const char* kWirelessFacilityRhsConstrPrefix = "wireless_facility_rhs_m";

static string MakeWirelessFacilityRhsConstrName(size_t masterIdx, size_t serial)
{
    return string(kWirelessFacilityRhsConstrPrefix) + to_string(masterIdx) + "_" + to_string(serial);
}

static bool TryParseWirelessFacilityRhsMasterIndex(const string& constrName, size_t& masterIdx)
{
    const string prefix = kWirelessFacilityRhsConstrPrefix;
    if (constrName.size() <= prefix.size() || constrName.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }

    const size_t splitPos = constrName.find('_', prefix.size());
    if (splitPos == string::npos) {
        return false;
    }

    masterIdx = static_cast<size_t>(stoull(constrName.substr(prefix.size(), splitPos - prefix.size())));
    return true;
}

static bool TryGetWirelessFacilityRhsMasterIndex(const GRBConstr& constr, size_t masterCount, size_t& masterIdx)
{
    const string constrName = constr.get(GRB_StringAttr_ConstrName);
    return TryParseWirelessFacilityRhsMasterIndex(constrName, masterIdx) && masterIdx < masterCount;
}

static void SetWirelessFacilityRhsConstrs(vector<GRBConstr>& constrs, double rhs)
{
    for (auto& constr : constrs) {
        constr.set(GRB_DoubleAttr_RHS, rhs);
    }
}

static void SetWirelessFacilityRhsConstrs(vector<GRBConstr>& constrs, const vector<double>& zValues)
{
    for (auto& constr : constrs) {
        size_t masterIdx = 0;
        if (TryGetWirelessFacilityRhsMasterIndex(constr, zValues.size(), masterIdx)) {
            constr.set(GRB_DoubleAttr_RHS, zValues[masterIdx]);
        }
    }
}

static void CollectWirelessFacilityRhsConstrs(GRBModel& model, vector<GRBConstr>& out)
{
    out.clear();
    const int numConstrs = model.get(GRB_IntAttr_NumConstrs);
    GRBConstr* constrs = model.getConstrs();
    for (int idx = 0; idx < numConstrs; ++idx) {
        size_t masterIdx = 0;
        if (TryParseWirelessFacilityRhsMasterIndex(constrs[idx].get(GRB_StringAttr_ConstrName), masterIdx)) {
            out.push_back(constrs[idx]);
        }
    }
    delete[] constrs;
}

static string MakeRouteBatteryCapacityVarName(int routeId)
{
    return "E_r" + to_string(routeId);
}

struct WirelessScenarioConfig {
    string dataFolder;
    double probability = 1.0;
};

static vector<WirelessScenarioConfig> LoadWirelessScenarioConfigs(const string& dataRootFolder)
{
    const fs::path scenarioCfgPath = ResolveDataFolderPath(dataRootFolder) / "scenarioCfg.txt";
    auto rows = LoadStringTable(scenarioCfgPath);
    if (!rows.has_value()) {
        return {};
    }

    const int numScenarios = stoi(rows->at(0).at(0));
    vector<WirelessScenarioConfig> scenarios;
    scenarios.reserve(numScenarios);
    for (int idx = 0; idx < numScenarios; ++idx) {
        const auto& row = rows->at(static_cast<size_t>(idx + 1));
        WirelessScenarioConfig scenario;
        scenario.dataFolder = row.at(0);
        scenario.probability = stod(row.at(1));
        scenarios.push_back(scenario);
    }
    return scenarios;
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

    // RouteCfg is read from the active scenario's scenarioMiscCfg.txt.
    // routeIdByEBs is aligned with ebIdx / EBs ordering, not raw EB ids.
    int numRoutes = 1;
    vector<string> routeNames;
    vector<int> routeIdByEBs;
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
    ModelData(string dataRootFolder)
        : ModelData(dataRootFolder, LoadWirelessScenarioConfigs(dataRootFolder).front()) {}

    ModelData(string dataRootFolder, WirelessScenarioConfig scenarioConfig)
        : dataRootFolder(std::move(dataRootFolder)),
          dataRootFolderPath(ResolveDataFolderPath(this->dataRootFolder)),
          scenarioConfig(std::move(scenarioConfig)),
          scenarioFolderPath(dataRootFolderPath / fs::path(this->scenarioConfig.dataFolder)) {
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
        return dataRootFolderPath;
    }

private:
    struct OperationCfg operationCfg;
    struct ChargerCfg chargerCfg;
    struct SchedulingCfg schedulingCfg;
    struct NetworkCfg networkCfg;
    string dataRootFolder;
    fs::path dataRootFolderPath;
    WirelessScenarioConfig scenarioConfig;
    fs::path scenarioFolderPath;

    fs::path ResolveRoot(const string& file) const {
        return dataRootFolderPath / file;
    }

    fs::path ResolveScenario(const string& file) const {
        return scenarioFolderPath / file;
    }

    void Init() {
        LoadGlobalConfigFromTxt();
        LoadScenarioOperationalTables();
        LoadScenarioConfigFromTxt();

        for (const auto& stationTrip : schedulingCfg.stationTrips) {
            for (int station : stationTrip) {
                networkCfg.stationPhys.insert(station);
            }
        }
    }

    bool LoadScenarioOperationalTables() {
        auto tmpDis = LoadLinkDistance(ResolveScenario("linkDistance.txt"));
        auto tmpTripArrTimes = LoadIntTable(ResolveScenario("tripArrTimes.txt"));
        auto tmpTripDepTimes = LoadIntTable(ResolveScenario("tripDepTimes.txt"));
        auto tmpTripStations = LoadIntTable(ResolveScenario("tripStations.txt"));
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

        return true;
    }

    bool LoadGlobalConfigFromTxt() {
        auto tmpGlobalCfg = LoadStringTable(ResolveRoot("globalCfg.txt"));
        if (tmpGlobalCfg.has_value()) {
            vector<string> operationCfgDouble = tmpGlobalCfg->at(OFFSET_OPERATION_CFG);
            vector<string> operationCfgInt = tmpGlobalCfg->at(OFFSET_OPERATION_CFG + 1);
            vector<string> operationCfgBus = tmpGlobalCfg->at(OFFSET_OPERATION_CFG + 2);
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

            vector<string> chargerCfgNum = tmpGlobalCfg->at(OFFSET_CHARGING_CFG);
            vector<string> chargerCfgRate = tmpGlobalCfg->at(OFFSET_CHARGING_CFG + 1);
            vector<string> chargerCfgEfficiency = tmpGlobalCfg->at(OFFSET_CHARGING_CFG + 2);
            vector<string> chargerCfgInt = tmpGlobalCfg->at(OFFSET_CHARGING_CFG + 3);
            chargerCfg.num = stoi(chargerCfgNum[0]);
            for (int i = 0; i < chargerCfg.num; i++) {
                chargerCfg.chargeRate.push_back(stod(chargerCfgRate[i]));
                chargerCfg.chargeEfficiency.push_back(stod(chargerCfgEfficiency[i]));
                chargerCfg.priceOfUnitCharger.push_back(stoi(chargerCfgInt[i]));
            }
            chargerCfg.priceOfInverterPerkW = stod(chargerCfgInt[chargerCfg.num]);
        }

        return true;
    }

    bool LoadScenarioConfigFromTxt() {
        auto tmpScenarioCfg = LoadStringTable(ResolveScenario("scenarioMiscCfg.txt"));
        if (tmpScenarioCfg.has_value()) {
            vector<string> networkCfgStart = tmpScenarioCfg->at(OFFSET_SCENARIO_NETWORK_CFG);
            vector<string> networkCfgEnd = tmpScenarioCfg->at(OFFSET_SCENARIO_NETWORK_CFG + 1);

            vector<string> schedulingCfgNum = tmpScenarioCfg->at(OFFSET_SCENARIO_SCHEDULING_CFG);
            vector<string> schedulingCfgEBs = tmpScenarioCfg->at(OFFSET_SCENARIO_SCHEDULING_CFG + 1);
            vector<string> schedulingCfgTripNumEBs = tmpScenarioCfg->at(OFFSET_SCENARIO_SCHEDULING_CFG + 2);
            schedulingCfg.numEB = stoi(schedulingCfgNum[0]);
            for (int i = 0; i < schedulingCfg.numEB; i++) {
                networkCfg.startDistanceEBs.push_back(stod(networkCfgStart[i]));
                networkCfg.endDistanceEBs.push_back(stod(networkCfgEnd[i]));
                schedulingCfg.EBs.push_back(stoi(schedulingCfgEBs[i]));
                schedulingCfg.tripNumEBs.push_back(stoi(schedulingCfgTripNumEBs[i]));
                schedulingCfg.numTrip += stoi(schedulingCfgTripNumEBs[i]);
                vector<string> schedulingCfgTrips = tmpScenarioCfg->at(OFFSET_SCENARIO_SCHEDULING_CFG + 3 + i);
                vector<int> tmp;
                for (int j = 0; j < schedulingCfg.tripNumEBs[i]; j++) {
                    tmp.push_back(stoi(schedulingCfgTrips[j]));
                }
                schedulingCfg.tripEBs.push_back(tmp);
            }

            vector<string> chargeCfgNums = tmpScenarioCfg->at(OFFSET_SCENARIO_CHARGER_CFG(schedulingCfg.numEB));
            networkCfg.chargeStationNum = stoi(chargeCfgNums[0]);
            for (int i = 0; i < networkCfg.chargeStationNum; i++) {
                vector<string> chargerCfgNums = tmpScenarioCfg->at(OFFSET_SCENARIO_CHARGER_CFG(schedulingCfg.numEB) + 1 + i);
                networkCfg.chargerNums[stoi(chargerCfgNums[0])] = stoi(chargerCfgNums[1]);
                networkCfg.chargerPhys.insert(stoi(chargerCfgNums[0]));
            }

            const size_t routeCfgOffset = static_cast<size_t>(
                OFFSET_SCENARIO_CHARGER_CFG(schedulingCfg.numEB) + 1 + networkCfg.chargeStationNum);
            schedulingCfg.numRoutes = stoi(tmpScenarioCfg->at(routeCfgOffset)[0]);
            schedulingCfg.routeNames = tmpScenarioCfg->at(routeCfgOffset + 1);

            const auto& routeIdTokens = tmpScenarioCfg->at(routeCfgOffset + 2);
            schedulingCfg.routeIdByEBs.reserve(routeIdTokens.size());
            for (const auto& token : routeIdTokens) {
                schedulingCfg.routeIdByEBs.push_back(stoi(token));
            }
        }

        return true;
    }
};

static vector<int> CountEBsByRoute(const SchedulingCfg& schedulingCfg)
{
    vector<int> routeEBCounts(static_cast<size_t>(schedulingCfg.numRoutes), 0);
    for (int ebIdx = 0; ebIdx < schedulingCfg.numEB; ++ebIdx) {
        const int routeId = schedulingCfg.routeIdByEBs.at(static_cast<size_t>(ebIdx));
        routeEBCounts.at(static_cast<size_t>(routeId)) += 1;
    }
    return routeEBCounts;
}

static double CalcWirelessBatteryCostCoeffPerEB(const ModelData& data)
{
    return data.GetOperationCfg().priceOfUnitBat;
}

static vector<ProblemDataVar> BuildWirelessMasterVars(const ModelData& data)
{
    const ChargerCfg& chargerCfg = data.GetChargerCfg();
    const NetworkCfg& networkCfg = data.GetNetworkCfg();

    vector<ProblemDataVar> masterVars;

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

static void PrintWirelessScenarioSummary(const vector<WirelessScenarioConfig>& scenarios)
{
    std::cout << "scenario_count=" << scenarios.size() << std::endl;
    for (size_t idx = 0; idx < scenarios.size(); ++idx) {
        std::cout << "scenario_" << idx
                  << " probability=" << scenarios[idx].probability
                  << " folder=" << scenarios[idx].dataFolder << std::endl;
    }
}

static void InitWirelessMasterData(ProblemData& data, const string& dataFolder, const string& usageLabel)
{
    vector<WirelessScenarioConfig> scenarios = LoadWirelessScenarioConfigs(dataFolder);
    // Current stage keeps a single scenario. Future multi-scenario runs should keep
    // first-stage facility candidates consistent across scenarios, or build them from
    // a shared global candidate set.
    ModelData modelData(dataFolder, scenarios.front());
    vector<ProblemDataVar> masterVars = BuildWirelessMasterVars(modelData);

    data.addData("masterVars", masterVars);
    data.addData("wireless_data_root_folder", dataFolder);
    data.addData("wireless_scenarios", scenarios);
    data.addData("wireless_scenario_count", static_cast<int>(scenarios.size()));

    std::cout << "Loaded wireless charging data for " << usageLabel << "." << std::endl;
    PrintWirelessScenarioSummary(scenarios);
    std::cout << "masterVars=" << masterVars.size() << std::endl;
}

static std::vector<ProblemDataConstr> BuildWirelessMasterConstrs(const ProblemData& data)
{
    (void)data;
    return {};
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

        for (auto& constr : relaxedFixConstrs) {
            size_t masterIdx = 0;
            if (TryGetWirelessFacilityRhsMasterIndex(constr, zValues.size(), masterIdx)) {
                const double lambda = constr.get(GRB_DoubleAttr_FarkasDual);
                cutInfo.yCoeffs[masterIdx] += lambda;
                cutInfo.constant -= lambda * zValues[masterIdx];
            }
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

    // Route-level continuous battery capacity decisions.
    vector<GRBVar> batteryCapacityByRoute;
    batteryCapacityByRoute.reserve(static_cast<size_t>(schedulingCfg.numRoutes));
    for (int routeId = 0; routeId < schedulingCfg.numRoutes; ++routeId) {
        batteryCapacityByRoute.push_back(
            model.addVar(operationCfg.capacityMin, operationCfg.capacityMax, 0.0, GRB_CONTINUOUS, MakeRouteBatteryCapacityVarName(routeId)));
    }

    auto batteryCapacityForEB = [&](int ebIdx) -> GRBVar {
        const int routeId = schedulingCfg.routeIdByEBs.at(static_cast<size_t>(ebIdx));
        return batteryCapacityByRoute.at(static_cast<size_t>(routeId));
    };

    vector<GRBVar> chargeNight_b; // [eb]
    unordered_map<int, GRBVar> wirelessStation;
    unordered_map<int, unordered_map<int, GRBVar>> wirelessLink;
    vector<vector<unordered_map<int, GRBVar>>> charge_bns; // [eb][trip][station]
    vector<vector<unordered_map<int, unordered_map<int, GRBVar>>>> charge_bnl; // [eb][trip][start][end]
    vector<vector<vector<GRBVar>>> energyDep_bns; // [eb][trip][station]
    vector<vector<vector<GRBVar>>> energyArr_bns; // [eb][trip][station]
    vector<vector<vector<GRBVar>>> energyShortageArr_bns; // [eb][trip][station]
    vector<GRBVar> energyShortageArrEnd_b; // [eb]
    unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, GRBVar>>>> x_bns_t; // [eb][trip][station][time]

    size_t numEB = schedulingCfg.numEB;

    chargeNight_b.reserve(numEB);
    energyShortageArrEnd_b.reserve(numEB);
    for (int eb : schedulingCfg.EBs) {
        string varName = "c_Night_b" + to_string(eb);
        chargeNight_b.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varName));
        string shortageName = "e_ShortageArrEnd_b" + to_string(eb);
        energyShortageArrEnd_b.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, shortageName));
    }

    unordered_map<int, vector<GRBLinExpr>> chargerOccupy;
    for (int chargerPhy : networkCfg.chargerPhys) {
        chargerOccupy[chargerPhy].resize(operationCfg.timeMax - operationCfg.timeMin);
    }

    charge_bns.reserve(numEB);
    energyDep_bns.reserve(numEB);
    energyArr_bns.reserve(numEB);
    energyShortageArr_bns.reserve(numEB);

    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        vector<unordered_map<int, GRBVar>> charge_b;
        vector<vector<GRBVar>> energyDep_b;
        vector<vector<GRBVar>> energyArr_b;
        vector<vector<GRBVar>> energyShortageArr_b;
        charge_b.reserve(numEBTrip);
        energyDep_b.reserve(numEBTrip);
        energyArr_b.reserve(numEBTrip);
        energyShortageArr_b.reserve(numEBTrip);

        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            unordered_map<int, GRBVar> charge_bn;
            vector<GRBVar> energyDep_bn;
            vector<GRBVar> energyArr_bn;
            vector<GRBVar> energyShortageArr_bn;
            energyDep_bn.reserve(numEBTripStations);
            energyArr_bn.reserve(numEBTripStations);
            energyShortageArr_bn.reserve(numEBTripStations);

            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                energyDep_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "e_Dep" + sub));
                energyArr_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "e_Arr" + sub));
                energyShortageArr_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "e_ShortageArr" + sub));
                charge_bn[station] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "c" + sub);
            }
            charge_b.push_back(charge_bn);
            energyDep_b.push_back(energyDep_bn);
            energyArr_b.push_back(energyArr_bn);
            energyShortageArr_b.push_back(energyShortageArr_bn);
        }
        charge_bns.push_back(charge_b);
        energyDep_bns.push_back(energyDep_b);
        energyArr_bns.push_back(energyArr_b);
        energyShortageArr_bns.push_back(energyShortageArr_b);
    }

    // stationOccupy
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                for (int time = arrTime; time < depTime; time++) {
                    string varName = "x_b" + to_string(eb) + "_n" + to_string(trip) +
                                     "_s" + to_string(station) + "_i" + to_string(stationIdx) + "_t" + to_string(time);
                    x_bns_t[eb][trip][station][time] = model.addVar(0, 1, 0, GRB_BINARY, varName);
                }
            }
        }
    }

    // static wireless charging facility
    for (const int& station : networkCfg.stationPhys) {
        if (!networkCfg.chargerPhys.count(station)) {
            string varName = "w_station_s" + to_string(station);
            wirelessStation[station] = model.addVar(0.0, 1, 0.0, GRB_BINARY, varName);
        }
    }

    // dynamic wireless charging
    charge_bnl.reserve(numEB);
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        auto eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        vector<unordered_map<int, unordered_map<int, GRBVar>>> charge_b;
        charge_b.reserve(numEBTrip);
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            auto tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
            unordered_map<int, unordered_map<int, GRBVar>> charge_bn;
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArr;
                int stationArr;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArr = schedulingCfg.tripEBs[ebIdx][tripIdx + 1];
                    stationArrIdx = 0;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                } else {
                    tripArr = tripDep;
                    stationArrIdx = stationIdx + 1;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                }

                string sub = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                             "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                charge_bn[stationDep][stationArr] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "c" + sub);
            }
            charge_b.push_back(charge_bn);
        }
        charge_bnl.push_back(charge_b);
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

    GRBLinExpr sum_day = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            for (const auto& [_, chargeVar] : charge_bns[ebIdx][tripIdx]) {
                sum_day += chargeVar;
            }
        }
    }

    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArr;
                int stationArr;
                if (stationIdx == numEBTripStations - 1) {
                    if (tripIdx == numEBTrip - 1) {
                        continue;
                    }
                    tripArr = schedulingCfg.tripEBs[ebIdx][tripIdx + 1];
                    stationArr = schedulingCfg.stationTrips[tripArr][0];
                } else {
                    tripArr = tripDep;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationIdx + 1];
                }

                sum_day += charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
            }
        }
    }

    GRBLinExpr sum_night = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        sum_night += chargeNight_b[ebIdx];
    }

    GRBLinExpr sum_energy_shortage = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                sum_energy_shortage += energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
            }
        }
        sum_energy_shortage += energyShortageArrEnd_b[ebIdx];
    }

    obj += operationCfg.priceOfPowerDay * sum_day + operationCfg.priceOfPowerNight * sum_night;
    obj += kWirelessEnergySlackPenalty * sum_energy_shortage;

    const vector<int> routeEBCounts = CountEBsByRoute(schedulingCfg);
    const double batteryCostPerUnitCapacityPerEB = CalcWirelessBatteryCostCoeffPerEB(data);
    for (int routeId = 0; routeId < schedulingCfg.numRoutes; ++routeId) {
        const double routeBatteryCostCoeff =
            static_cast<double>(routeEBCounts.at(static_cast<size_t>(routeId))) *
            batteryCostPerUnitCapacityPerEB;
        obj += routeBatteryCostCoeff * batteryCapacityByRoute.at(static_cast<size_t>(routeId));
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


    // energy range
    const double socMax = operationCfg.socMax;
    const double socMin = operationCfg.socMin;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar arrE = energyArr_bns[ebIdx][tripIdx][stationIdx];
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                GRBVar shortageArr = energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
                model.addConstr(arrE + shortageArr >= routeE * socMin, "constrArrMin" + sub);
                model.addConstr(arrE <= routeE * socMax, "constrArrMax" + sub);
                model.addConstr(depE >= routeE * socMin, "constrDepMin" + sub);
                model.addConstr(depE <= routeE * socMax, "constrDepMax" + sub);
            }
        }
    }

    // number of fast charger
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                for (int time = arrTime; time < depTime; time++) {
                    chargerOccupy[station][time] += x_bns_t[eb][trip][station][time];
                }
            }
        }
    }
    for (int chargerPhy : networkCfg.chargerPhys) {
        for (int time = operationCfg.timeMin; time < operationCfg.timeMax; time++) {
            if (chargerOccupy[chargerPhy][time].size() > 0) {
                string sub = "_t" + to_string(time) + "_c" + to_string(chargerPhy);
                model.addConstr(chargerOccupy[chargerPhy][time] <= networkCfg.chargerNums[chargerPhy], "constChargerNum" + sub);
            }
        }
    }

    // c_bns in fast charging stations
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                GRBLinExpr occupyTime;
                for (int time = schedulingCfg.stationArrTimes[trip][stationIdx];
                         time < schedulingCfg.stationDepTimes[trip][stationIdx]; time++) {
                    occupyTime += x_bns_t[eb][trip][station][time];
                }
                model.addConstr(charge <= chargerCfg.chargeRate[0] * occupyTime, "constrChargeFast" + sub);
            }
        }
    }

    // c_bns in static wireless charging stations
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar charge = charge_bns[ebIdx][tripIdx][station];
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
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArrIdx;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArrIdx = tripIdx + 1;
                    stationArrIdx = 0;
                } else {
                    tripArrIdx = tripIdx;
                    stationArrIdx = stationIdx + 1;
                }

                int tripArr = schedulingCfg.tripEBs[ebIdx][tripArrIdx];
                int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];

                string subCharge = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                                   "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                GRBVar linkCharge = charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
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
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        const auto& trips = schedulingCfg.tripEBs[ebIdx];
        int startTripIdx = 0;
        int startTrip = trips[startTripIdx];
        int startStationIdx = 0;
        int startStation = schedulingCfg.stationTrips[startTrip][startStationIdx];
        string subStart = "_b" + to_string(eb) + "_n" + to_string(startTrip) + "_s" + to_string(startStation) + "_i" + to_string(startStationIdx);
        double distStart = networkCfg.startDistanceEBs[ebIdx];
        GRBLinExpr consumptionStart = distStart * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        GRBVar startEnergy = energyArr_bns[ebIdx][startTripIdx][startStationIdx];
        model.addConstr(startEnergy == routeE * socMax - consumptionStart, "e_Start" + subStart);

        int endTripIdx = trips.size() - 1;
        int endTrip = trips[endTripIdx];
        int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
        int endStation = schedulingCfg.stationTrips[endTrip][endStationIdx];
        string subEnd = "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(endStation) + "_i" + to_string(endStationIdx);
        double distEnd = networkCfg.endDistanceEBs[ebIdx];
        GRBLinExpr consumptionEnd = distEnd * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        GRBVar endEnergy = energyDep_bns[ebIdx][endTripIdx][endStationIdx];
        GRBVar endShortage = energyShortageArrEnd_b[ebIdx];
        model.addConstr(endEnergy + endShortage >= routeE * socMin + consumptionEnd, "e_End" + subEnd);
    }

    // inner station energy connection
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar arrE = energyArr_bns[ebIdx][tripIdx][stationIdx];
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                GRBVar shortageArr = energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
                if (networkCfg.chargerPhys.count(station)) {
                    GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                    model.addConstr(depE == arrE + shortageArr + charge * chargerCfg.chargeEfficiency[0],
                                    "e_InnerStation" + sub);
                } else {
                    GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                    model.addConstr(depE == arrE + shortageArr + charge * chargerCfg.chargeEfficiency[1],
                                    "e_InnerStation" + sub);
                }
            }
        }
    }

    // inter station energy connection
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                string subDep = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) + "_i" + to_string(stationIdx);
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                int tripArrIdx;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArrIdx = tripIdx + 1;
                    stationArrIdx = 0;
                } else {
                    tripArrIdx = tripIdx;
                    stationArrIdx = stationIdx + 1;
                }

                int tripArr = schedulingCfg.tripEBs[ebIdx][tripArrIdx];
                int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                GRBVar arrE = energyArr_bns[ebIdx][tripArrIdx][stationArrIdx];
                double dist = networkCfg.linkDistances[stationDep][stationArr];
                GRBLinExpr consumption = dist * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
                GRBVar linkCharge = charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
                model.addConstr(arrE == depE - consumption + linkCharge * chargerCfg.chargeEfficiency[2],
                                "e_InterStation" + subDep);
            }
        }
    }

    // overnight charging
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        const auto& trips = schedulingCfg.tripEBs[ebIdx];
        int endTripIdx = trips.size() - 1;
        int endTrip = trips[endTripIdx];
        int stationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
        int station = schedulingCfg.stationTrips[endTrip][stationIdx];
        string subEnd = "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
        GRBVar endDepE = energyDep_bns[ebIdx][endTripIdx][stationIdx];
        GRBVar endShortage = energyShortageArrEnd_b[ebIdx];
        GRBVar nightCharge = chargeNight_b[ebIdx];
        double dist = networkCfg.endDistanceEBs[ebIdx];
        GRBLinExpr consumption = dist * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        model.addConstr(nightCharge * chargerCfg.chargeEfficiency[0] == routeE * socMax - (endDepE + endShortage - consumption),
                        "e_Overnight_b" + to_string(eb));
    }

    model.setObjective(obj, GRB_MINIMIZE);
    model.update();
    return true;
}

static bool BuildWirelessOperationalSubproblem(const string& dataRootFolder, const WirelessScenarioConfig& scenario,
    size_t scenarioIdx,
    const vector<ProblemDataVar>& masterVars, GRBModel& model,
    IntegerLShapedSubProblemContext& context, const WirelessGurobiOptions& options,
    GRBLinExpr& aggregateObj, size_t& facilityRhsSerial,
    const unordered_map<string, GRBVar>* directMasterVarsByName = nullptr)
{
    ModelData data(dataRootFolder, scenario);

    OperationCfg operationCfg = data.GetOperationCfg();
    ChargerCfg chargerCfg = data.GetChargerCfg();
    SchedulingCfg schedulingCfg = data.GetSchedulingCfg();
    NetworkCfg networkCfg = data.GetNetworkCfg();

    unordered_map<string, size_t> masterIdxByName;
    masterIdxByName.reserve(masterVars.size());
    for (size_t idx = 0; idx < masterVars.size(); ++idx) {
        masterIdxByName[masterVars[idx].name] = idx;
    }

    auto& facilityRhsConstrs = context.EnsureConstrGroup(kWirelessFacilityRhsConstrGroup);
    const string namePrefix = "sc" + to_string(scenarioIdx) + "_";

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

    // Route-level continuous battery capacity decisions.
    vector<GRBVar> batteryCapacityByRoute;
    batteryCapacityByRoute.reserve(static_cast<size_t>(schedulingCfg.numRoutes));
    for (int routeId = 0; routeId < schedulingCfg.numRoutes; ++routeId) {
        batteryCapacityByRoute.push_back(
            model.addVar(operationCfg.capacityMin, operationCfg.capacityMax, 0.0, GRB_CONTINUOUS,
                namePrefix + MakeRouteBatteryCapacityVarName(routeId)));
    }

    auto batteryCapacityForEB = [&](int ebIdx) -> GRBVar {
        const int routeId = schedulingCfg.routeIdByEBs.at(static_cast<size_t>(ebIdx));
        return batteryCapacityByRoute.at(static_cast<size_t>(routeId));
    };

    vector<GRBVar> chargeNight_b; // [eb]
    vector<vector<unordered_map<int, GRBVar>>> charge_bns; // [eb][trip][station]
    vector<vector<unordered_map<int, unordered_map<int, GRBVar>>>> charge_bnl; // [eb][trip][start][end]
    vector<vector<vector<GRBVar>>> energyDep_bns; // [eb][trip][station]
    vector<vector<vector<GRBVar>>> energyArr_bns; // [eb][trip][station]
    vector<vector<vector<GRBVar>>> energyShortageArr_bns; // [eb][trip][station]
    vector<GRBVar> energyShortageArrEnd_b; // [eb]
    unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, GRBVar>>>> x_bns_t; // [eb][trip][station][time]

    size_t numEB = schedulingCfg.numEB;

    chargeNight_b.reserve(numEB);
    energyShortageArrEnd_b.reserve(numEB);
    for (int eb : schedulingCfg.EBs) {
        string varName = namePrefix + "c_Night_b" + to_string(eb);
        chargeNight_b.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, varName));
        string shortageName = namePrefix + "e_ShortageArrEnd_b" + to_string(eb);
        energyShortageArrEnd_b.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, shortageName));
    }

    unordered_map<int, vector<GRBLinExpr>> chargerOccupy;
    for (int chargerPhy : networkCfg.chargerPhys) {
        chargerOccupy[chargerPhy].resize(operationCfg.timeMax - operationCfg.timeMin);
    }

    charge_bns.reserve(numEB);
    energyDep_bns.reserve(numEB);
    energyArr_bns.reserve(numEB);
    energyShortageArr_bns.reserve(numEB);

    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        vector<unordered_map<int, GRBVar>> charge_b;
        vector<vector<GRBVar>> energyDep_b;
        vector<vector<GRBVar>> energyArr_b;
        vector<vector<GRBVar>> energyShortageArr_b;
        charge_b.reserve(numEBTrip);
        energyDep_b.reserve(numEBTrip);
        energyArr_b.reserve(numEBTrip);
        energyShortageArr_b.reserve(numEBTrip);

        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            unordered_map<int, GRBVar> charge_bn;
            vector<GRBVar> energyDep_bn;
            vector<GRBVar> energyArr_bn;
            vector<GRBVar> energyShortageArr_bn;
            energyDep_bn.reserve(numEBTripStations);
            energyArr_bn.reserve(numEBTripStations);
            energyShortageArr_bn.reserve(numEBTripStations);

            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                energyDep_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, namePrefix + "e_Dep" + sub));
                energyArr_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, namePrefix + "e_Arr" + sub));
                energyShortageArr_bn.push_back(model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, namePrefix + "e_ShortageArr" + sub));
                charge_bn[station] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, namePrefix + "c" + sub);
            }
            charge_b.push_back(charge_bn);
            energyDep_b.push_back(energyDep_bn);
            energyArr_b.push_back(energyArr_bn);
            energyShortageArr_b.push_back(energyShortageArr_bn);
        }
        charge_bns.push_back(charge_b);
        energyDep_bns.push_back(energyDep_b);
        energyArr_bns.push_back(energyArr_b);
        energyShortageArr_bns.push_back(energyShortageArr_b);
    }

    // stationOccupy
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                for (int time = arrTime; time < depTime; time++) {
                    string varName = namePrefix + "x_b" + to_string(eb) + "_n" + to_string(trip) +
                                     "_s" + to_string(station) + "_i" + to_string(stationIdx) + "_t" + to_string(time);
                    x_bns_t[eb][trip][station][time] = model.addVar(0, 1, 0, GRB_BINARY, varName);
                }
            }
        }
    }

    // dynamic wireless charging
    charge_bnl.reserve(numEB);
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        auto eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        vector<unordered_map<int, unordered_map<int, GRBVar>>> charge_b;
        charge_b.reserve(numEBTrip);
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            auto tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
            unordered_map<int, unordered_map<int, GRBVar>> charge_bn;
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArr;
                int stationArr;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArr = schedulingCfg.tripEBs[ebIdx][tripIdx + 1];
                    stationArrIdx = 0;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                } else {
                    tripArr = tripDep;
                    stationArrIdx = stationIdx + 1;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                }

                string sub = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                             "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                charge_bn[stationDep][stationArr] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, namePrefix + "c" + sub);
            }
            charge_b.push_back(charge_bn);
        }
        charge_bnl.push_back(charge_b);
    }

    model.update();

    // objective function
    GRBLinExpr obj = 0;

    GRBLinExpr sum_day = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            for (const auto& [_, chargeVar] : charge_bns[ebIdx][tripIdx]) {
                sum_day += chargeVar;
            }
        }
    }

    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[tripDep].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArr;
                int stationArr;
                if (stationIdx == numEBTripStations - 1) {
                    if (tripIdx == numEBTrip - 1) {
                        continue;
                    }
                    tripArr = schedulingCfg.tripEBs[ebIdx][tripIdx + 1];
                    stationArr = schedulingCfg.stationTrips[tripArr][0];
                } else {
                    tripArr = tripDep;
                    stationArr = schedulingCfg.stationTrips[tripArr][stationIdx + 1];
                }

                sum_day += charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
            }
        }
    }

    GRBLinExpr sum_night = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        sum_night += chargeNight_b[ebIdx];
    }

    GRBLinExpr sum_energy_shortage = 0;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                sum_energy_shortage += energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
            }
        }
        sum_energy_shortage += energyShortageArrEnd_b[ebIdx];
    }

    obj += operationCfg.priceOfPowerDay * sum_day + operationCfg.priceOfPowerNight * sum_night;
    obj += kWirelessEnergySlackPenalty * sum_energy_shortage;

    const vector<int> routeEBCounts = CountEBsByRoute(schedulingCfg);
    const double batteryCostPerUnitCapacityPerEB = CalcWirelessBatteryCostCoeffPerEB(data);
    for (int routeId = 0; routeId < schedulingCfg.numRoutes; ++routeId) {
        const double routeBatteryCostCoeff =
            static_cast<double>(routeEBCounts.at(static_cast<size_t>(routeId))) *
            batteryCostPerUnitCapacityPerEB;
        obj += routeBatteryCostCoeff * batteryCapacityByRoute.at(static_cast<size_t>(routeId));
    }

    // energy range
    const double socMax = operationCfg.socMax;
    const double socMin = operationCfg.socMin;
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar arrE = energyArr_bns[ebIdx][tripIdx][stationIdx];
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                GRBVar shortageArr = energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
                model.addConstr(arrE + shortageArr >= routeE * socMin, "constrArrMin" + sub);
                model.addConstr(arrE <= routeE * socMax, "constrArrMax" + sub);
                model.addConstr(depE >= routeE * socMin, "constrDepMin" + sub);
                model.addConstr(depE <= routeE * socMax, "constrDepMax" + sub);
            }
        }
    }

    // number of fast charger
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        size_t numEBTrip = schedulingCfg.tripEBs[ebIdx].size();
        for (int tripIdx = 0; tripIdx < numEBTrip; tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            size_t numEBTripStations = schedulingCfg.stationTrips[trip].size();
            for (int stationIdx = 0; stationIdx < numEBTripStations; stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                int arrTime = schedulingCfg.stationArrTimes[trip][stationIdx];
                int depTime = schedulingCfg.stationDepTimes[trip][stationIdx];
                for (int time = arrTime; time < depTime; time++) {
                    chargerOccupy[station][time] += x_bns_t[eb][trip][station][time];
                }
            }
        }
    }
    for (int chargerPhy : networkCfg.chargerPhys) {
        for (int time = operationCfg.timeMin; time < operationCfg.timeMax; time++) {
            if (chargerOccupy[chargerPhy][time].size() > 0) {
                string sub = "_t" + to_string(time) + "_c" + to_string(chargerPhy);
                model.addConstr(chargerOccupy[chargerPhy][time] <= networkCfg.chargerNums[chargerPhy], "constChargerNum" + sub);
            }
        }
    }

    // c_bns in fast charging stations
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (!networkCfg.chargerPhys.count(station)) {
                    continue;
                }
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                GRBLinExpr occupyTime;
                for (int time = schedulingCfg.stationArrTimes[trip][stationIdx];
                         time < schedulingCfg.stationDepTimes[trip][stationIdx]; time++) {
                    occupyTime += x_bns_t[eb][trip][station][time];
                }
                model.addConstr(charge <= chargerCfg.chargeRate[0] * occupyTime, "constrChargeFast" + sub);
            }
        }
    }

    // c_bns in static wireless charging stations.
    // Wireless facility availability is represented by RHS constraints tied to master z values.
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                if (networkCfg.chargerPhys.count(station)) {
                    continue;
                }

                const string masterVarName = "w_station_s" + to_string(station);
                const size_t masterIdx = masterIdxByName.at(masterVarName);

                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                const int stayTime = schedulingCfg.stationDepTimes[trip][stationIdx] - schedulingCfg.stationArrTimes[trip][stationIdx];
                const double chargeUpperBound = chargerCfg.chargeRate[1] * static_cast<double>(stayTime);
                if (chargeUpperBound <= 0.0) {
                    model.addConstr(charge <= 0.0, namePrefix + "constrChargeWirelessStationZero" + sub);
                    continue;
                }

                if (directMasterVarsByName != nullptr) {
                    model.addConstr(charge <= chargeUpperBound * directMasterVarsByName->at(masterVarName),
                        namePrefix + "constrChargeWirelessStation" + sub);
                } else {
                    GRBConstr rhsConstr = model.addConstr((1.0 / chargeUpperBound) * charge <= 0.0,
                        MakeWirelessFacilityRhsConstrName(masterIdx, facilityRhsSerial++));
                    facilityRhsConstrs.push_back(rhsConstr);
                }
            }
        }
    }

    // c_bnl in dynamic wireless charging links.
    // Link availability is represented by RHS constraints tied to master z values.
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                int tripArrIdx;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArrIdx = tripIdx + 1;
                    stationArrIdx = 0;
                } else {
                    tripArrIdx = tripIdx;
                    stationArrIdx = stationIdx + 1;
                }

                int tripArr = schedulingCfg.tripEBs[ebIdx][tripArrIdx];
                int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                const string masterVarName = "w_link_l" + to_string(stationDep) + "To" + to_string(stationArr);
                const size_t masterIdx = masterIdxByName.at(masterVarName);

                string subCharge = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) +
                                   "_i" + to_string(stationIdx) + "To_n" + to_string(tripArr) + "_s" + to_string(stationArr) + "_i" + to_string(stationArrIdx);
                GRBVar linkCharge = charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
                int timeDep = schedulingCfg.stationDepTimes[tripDep][stationIdx];
                int timeArr = schedulingCfg.stationArrTimes[tripArr][stationArrIdx];
                int travelTime = timeArr - timeDep;
                const double chargeUpperBound = chargerCfg.chargeRate[2] * static_cast<double>(travelTime);
                if (chargeUpperBound <= 0.0) {
                    model.addConstr(linkCharge <= 0.0, namePrefix + "constrChargeWirelessLinkZero" + subCharge);
                    continue;
                }

                if (directMasterVarsByName != nullptr) {
                    model.addConstr(linkCharge <= chargeUpperBound * directMasterVarsByName->at(masterVarName),
                        namePrefix + "constrChargeWirelessLink" + subCharge);
                } else {
                    GRBConstr rhsConstr = model.addConstr((1.0 / chargeUpperBound) * linkCharge <= 0.0,
                        MakeWirelessFacilityRhsConstrName(masterIdx, facilityRhsSerial++));
                    facilityRhsConstrs.push_back(rhsConstr);
                }
            }
        }
    }

    // energy of the first and the last trips
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        const auto& trips = schedulingCfg.tripEBs[ebIdx];
        int startTripIdx = 0;
        int startTrip = trips[startTripIdx];
        int startStationIdx = 0;
        int startStation = schedulingCfg.stationTrips[startTrip][startStationIdx];
        string subStart = "_b" + to_string(eb) + "_n" + to_string(startTrip) + "_s" + to_string(startStation) + "_i" + to_string(startStationIdx);
        double distStart = networkCfg.startDistanceEBs[ebIdx];
        GRBLinExpr consumptionStart = distStart * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        GRBVar startEnergy = energyArr_bns[ebIdx][startTripIdx][startStationIdx];
        model.addConstr(startEnergy == routeE * socMax - consumptionStart, "e_Start" + subStart);

        int endTripIdx = trips.size() - 1;
        int endTrip = trips[endTripIdx];
        int endStationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
        int endStation = schedulingCfg.stationTrips[endTrip][endStationIdx];
        string subEnd = "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(endStation) + "_i" + to_string(endStationIdx);
        double distEnd = networkCfg.endDistanceEBs[ebIdx];
        GRBLinExpr consumptionEnd = distEnd * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        GRBVar endEnergy = energyDep_bns[ebIdx][endTripIdx][endStationIdx];
        GRBVar endShortage = energyShortageArrEnd_b[ebIdx];
        model.addConstr(endEnergy + endShortage >= routeE * socMin + consumptionEnd, "e_End" + subEnd);
    }

    // inner station energy connection
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int trip = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[trip].size(); stationIdx++) {
                int station = schedulingCfg.stationTrips[trip][stationIdx];
                string sub = "_b" + to_string(eb) + "_n" + to_string(trip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
                GRBVar arrE = energyArr_bns[ebIdx][tripIdx][stationIdx];
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                GRBVar shortageArr = energyShortageArr_bns[ebIdx][tripIdx][stationIdx];
                if (networkCfg.chargerPhys.count(station)) {
                    GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                    model.addConstr(depE == arrE + shortageArr + charge * chargerCfg.chargeEfficiency[0],
                                    "e_InnerStation" + sub);
                } else {
                    GRBVar charge = charge_bns[ebIdx][tripIdx][station];
                    model.addConstr(depE == arrE + shortageArr + charge * chargerCfg.chargeEfficiency[1],
                                    "e_InnerStation" + sub);
                }
            }
        }
    }

    // inter station energy connection
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        for (int tripIdx = 0; tripIdx < schedulingCfg.tripEBs[ebIdx].size(); tripIdx++) {
            int tripDep = schedulingCfg.tripEBs[ebIdx][tripIdx];
            for (int stationIdx = 0; stationIdx < schedulingCfg.stationTrips[tripDep].size(); stationIdx++) {
                int stationDep = schedulingCfg.stationTrips[tripDep][stationIdx];
                string subDep = "_b" + to_string(eb) + "_n" + to_string(tripDep) + "_s" + to_string(stationDep) + "_i" + to_string(stationIdx);
                GRBVar depE = energyDep_bns[ebIdx][tripIdx][stationIdx];
                int tripArrIdx;
                int stationArrIdx;
                if (stationIdx == schedulingCfg.stationTrips[tripDep].size() - 1) {
                    if (tripIdx == schedulingCfg.tripEBs[ebIdx].size() - 1) {
                        continue;
                    }
                    tripArrIdx = tripIdx + 1;
                    stationArrIdx = 0;
                } else {
                    tripArrIdx = tripIdx;
                    stationArrIdx = stationIdx + 1;
                }

                int tripArr = schedulingCfg.tripEBs[ebIdx][tripArrIdx];
                int stationArr = schedulingCfg.stationTrips[tripArr][stationArrIdx];
                GRBVar arrE = energyArr_bns[ebIdx][tripArrIdx][stationArrIdx];
                double dist = networkCfg.linkDistances[stationDep][stationArr];
                GRBLinExpr consumption = dist * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
                GRBVar linkCharge = charge_bnl[ebIdx][tripIdx][stationDep][stationArr];
                model.addConstr(arrE == depE - consumption + linkCharge * chargerCfg.chargeEfficiency[2],
                                "e_InterStation" + subDep);
            }
        }
    }

    // overnight charging
    for (int ebIdx = 0; ebIdx < numEB; ebIdx++) {
        int eb = schedulingCfg.EBs[ebIdx];
        GRBVar routeE = batteryCapacityForEB(ebIdx);
        const auto& trips = schedulingCfg.tripEBs[ebIdx];
        int endTripIdx = trips.size() - 1;
        int endTrip = trips[endTripIdx];
        int stationIdx = schedulingCfg.stationTrips[endTrip].size() - 1;
        int station = schedulingCfg.stationTrips[endTrip][stationIdx];
        string subEnd = "_b" + to_string(eb) + "_n" + to_string(endTrip) + "_s" + to_string(station) + "_i" + to_string(stationIdx);
        GRBVar endDepE = energyDep_bns[ebIdx][endTripIdx][stationIdx];
        GRBVar endShortage = energyShortageArrEnd_b[ebIdx];
        GRBVar nightCharge = chargeNight_b[ebIdx];
        double dist = networkCfg.endDistanceEBs[ebIdx];
        GRBLinExpr consumption = dist * (operationCfg.curbWeight + routeE * operationCfg.weightPerEnergy) * operationCfg.mu * operationCfg.g / 3.6;
        model.addConstr(nightCharge * chargerCfg.chargeEfficiency[0] == routeE * socMax - (endDepE + endShortage - consumption),
                        "e_Overnight_b" + to_string(eb));
    }

    aggregateObj += scenario.probability * obj;
    model.update();
    return true;
}

static bool BuildWirelessChargingSolverModel(const string& dataFolder, GRBModel& model)
{
    WirelessGurobiOptions options;
    options.timeLimit = 18000.0;
    options.includeFirstStageObjective = true;
    options.configureSolverParameters = true;

    const vector<WirelessScenarioConfig> scenarios = LoadWirelessScenarioConfigs(dataFolder);
    ModelData masterData(dataFolder, scenarios.front());
    const vector<ProblemDataVar> masterVars = BuildWirelessMasterVars(masterData);

    if (options.configureSolverParameters) {
        model.set(GRB_DoubleParam_TimeLimit, options.timeLimit);
        model.set(GRB_IntParam_Presolve, 2);
        model.set(GRB_IntParam_MIPFocus, 1);
        model.set(GRB_DoubleParam_Heuristics, 0.5);
        model.set(GRB_IntParam_Threads, 14);
        model.set(GRB_IntParam_Method, 1);
        model.set(GRB_DoubleParam_MIPGap, 0.0);
    }

    GRBLinExpr obj = 0.0;
    unordered_map<string, GRBVar> masterVarsByName;
    masterVarsByName.reserve(masterVars.size());
    for (const auto& varData : masterVars) {
        GRBVar var = model.addVar(varData.lb, varData.ub, varData.obj, varData.type, varData.name);
        masterVarsByName.emplace(varData.name, var);
        obj += varData.obj * var;
    }

    IntegerLShapedSubProblemContext directContext;
    size_t facilityRhsSerial = 0;
    WirelessGurobiOptions scenarioOptions = options;
    scenarioOptions.includeFirstStageObjective = false;
    scenarioOptions.configureSolverParameters = false;
    for (size_t scenarioIdx = 0; scenarioIdx < scenarios.size(); ++scenarioIdx) {
        if (!BuildWirelessOperationalSubproblem(dataFolder, scenarios[scenarioIdx], scenarioIdx,
                masterVars, model, directContext, scenarioOptions, obj, facilityRhsSerial, &masterVarsByName)) {
            return false;
        }
    }

    model.setObjective(obj, GRB_MINIMIZE);
    model.update();
    return true;
}

static bool BuildWirelessChargingLShapedSubProblem(
    const string& dataRootFolder, const vector<WirelessScenarioConfig>& scenarios, size_t scenarioIdx,
    const vector<ProblemDataVar>& masterVars, GRBModel& subModel, IntegerLShapedSubProblemContext& context)
{
    context.Clear();
    context.EnsureConstrGroup(kWirelessFacilityRhsConstrGroup).clear();

    WirelessGurobiOptions options;
    options.mipGap = 0.0;
    options.timeLimit = 18000.0;
    options.includeFirstStageObjective = false;
    options.configureSolverParameters = false;

    GRBLinExpr obj = 0.0;
    size_t facilityRhsSerial = 0;
    if (scenarioIdx >= scenarios.size()) {
        return false;
    }
    if (!BuildWirelessOperationalSubproblem(dataRootFolder, scenarios[scenarioIdx], scenarioIdx,
            masterVars, subModel, context, options, obj, facilityRhsSerial)) {
        return false;
    }
    subModel.setObjective(obj, GRB_MINIMIZE);

    subModel.set(GRB_IntParam_InfUnbdInfo, 1);
    subModel.set(GRB_IntParam_DualReductions, 0);
    subModel.set(GRB_IntParam_OutputFlag, 0);
    subModel.set(GRB_DoubleParam_MIPGap, 0.0);
    subModel.set(GRB_DoubleParam_TimeLimit, 18000.0);
    subModel.set(GRB_IntParam_Presolve, 2);
    subModel.set(GRB_IntParam_Method, 1);
    subModel.update();
    return true;
}

WirelessChargingDataInitializationStrategy_Solver::WirelessChargingDataInitializationStrategy_Solver(
    std::string dataFolder)
    : dataFolder(std::move(dataFolder)) {}

Status WirelessChargingDataInitializationStrategy_Solver::DataInit(GRBModel& model)
{
    std::cout << "Loaded wireless charging data from file." << std::endl;
    PrintWirelessScenarioSummary(LoadWirelessScenarioConfigs(dataFolder));

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
    return std::vector<double>(masterVars.size(), 1.0);
}
bool WirelessChargingDataInitializationStrategy_LShaped::IsWarmStartMasterFeasible(const ProblemData& data,
    const std::vector<double>& zValues, double tolerance) const
{
    return true;
}

int WirelessChargingSubProblemStrategy_LShaped::ScenarioCount(const ProblemData& problemData) const
{
    return static_cast<int>(problemData.getData<std::vector<WirelessScenarioConfig>>("wireless_scenarios").size());
}

void WirelessChargingSubProblemStrategy_LShaped::InitSubProblem(const ProblemData& problemData, int scenarioIndex,
    GRBModel& subModel, IntegerLShapedSubProblemContext& context)
{
    const auto& dataRootFolder = problemData.getData<std::string>("wireless_data_root_folder");
    const auto& scenarios = problemData.getData<std::vector<WirelessScenarioConfig>>("wireless_scenarios");
    const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");

    bool built = BuildWirelessChargingLShapedSubProblem(dataRootFolder, scenarios,
        static_cast<size_t>(scenarioIndex), masterVars, subModel, context);
    if (!built) {
        throw std::runtime_error("Failed to build wireless charging L-shaped sub-problem.");
    }

    lowerBoundReady = false;
    qLowerBound = 0.0;
    lowerBoundCutAdded = false;

    auto& facilityRhsConstrs = context.EnsureConstrGroup(kWirelessFacilityRhsConstrGroup);
    SetWirelessFacilityRhsConstrs(facilityRhsConstrs, 1.0);
    subModel.update();
    subModel.optimize();
    int lbStatus = subModel.get(GRB_IntAttr_Status);
    if (lbStatus == GRB_OPTIMAL || (lbStatus == GRB_TIME_LIMIT && subModel.get(GRB_IntAttr_SolCount) > 0)) {
        qLowerBound = subModel.get(GRB_DoubleAttr_ObjVal);
        lowerBoundReady = true;
    } else {
        std::cerr << "Wireless L-shaped: failed to pre-compute recourse lower bound, status="
                  << lbStatus << ". fallback lower bound = 0." << std::endl;
    }

    SetWirelessFacilityRhsConstrs(facilityRhsConstrs, 0.0);
    subModel.update();

    InitRelaxedSubProblem(problemData, subModel);
}

void WirelessChargingSubProblemStrategy_LShaped::UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
    IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues)
{
    const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");
    if (zValues.size() != masterVars.size()) {
        throw std::runtime_error("Wireless L-shaped master value size mismatch.");
    }

    auto& facilityRhsConstrs = context.EnsureConstrGroup(kWirelessFacilityRhsConstrGroup);
    SetWirelessFacilityRhsConstrs(facilityRhsConstrs, zValues);

    subModel.update();
}

void WirelessChargingSubProblemStrategy_LShaped::InitRelaxedSubProblem(
    const ProblemData& problemData, GRBModel& subModel)
{
    (void)problemData;

    relaxedModel = std::make_unique<GRBModel>(subModel.relax());
    relaxedModel->set(GRB_IntParam_OutputFlag, 0);
    relaxedModel->set(GRB_IntParam_InfUnbdInfo, 1);
    relaxedModel->set(GRB_IntParam_DualReductions, 0);
    relaxedModel->set(GRB_IntParam_Method, 2);
    relaxedModel->set(GRB_IntParam_Crossover, 0);

    relaxedFixConstrs.clear();
    CollectWirelessFacilityRhsConstrs(*relaxedModel, relaxedFixConstrs);

    relaxedModel->update();
}

Status WirelessChargingSubProblemStrategy_LShaped::SolveRelaxedModel(
    const ProblemData& problemData, const std::vector<double>& zValues,
    IntegerLShapedCutInfo& cutInfo, double& subObj)
{
    try {
        const auto& masterVars = problemData.getData<std::vector<ProblemDataVar>>("masterVars");
        if (zValues.size() != masterVars.size() || !relaxedModel) {
            std::cerr << "Wireless relaxed sub-problem: relaxed model is not initialized" << std::endl;
            return ERROR;
        }

        SetWirelessFacilityRhsConstrs(relaxedFixConstrs, zValues);

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

            for (auto& constr : relaxedFixConstrs) {
                size_t masterIdx = 0;
                if (TryGetWirelessFacilityRhsMasterIndex(constr, zValues.size(), masterIdx)) {
                    const double pi = constr.get(GRB_DoubleAttr_Pi);
                    cutInfo.yCoeffs[masterIdx] += pi;
                    cutInfo.constant -= pi * zValues[masterIdx];
                }
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
