#include "BARP_S.hpp"
#include "tools.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr const char* kVarGroupX = "brs_x";
constexpr const char* kVarGroupY = "brs_y";
constexpr const char* kVarGroupAlpha = "brs_alpha";
constexpr const char* kConstrGroupAssign = "brs_assign_constr";

struct BRSRawInput {
    int stationCount = 0;
    int horizon = 0;
    std::vector<int> travelTimes;
    std::vector<int> existingTrains;
    std::vector<int> storageFlags;
    std::vector<double> brsCosts;
    int budget = 0;
    int trainCapacity = 0;
    int minHeadway = 0;
    double lambdaCost = 1.0;
    std::vector<BRSScenarioData> scenarios;
};

const std::string& RequireKey(
    const std::unordered_map<std::string, std::string>& kv,
    const std::string& key)
{
    auto it = kv.find(Tools::ToLower(key));
    if (it == kv.end()) {
        throw std::runtime_error("Missing key in BRS data file: " + key);
    }
    return it->second;
}

BRSRawInput LoadBRSInput(const std::string& dataFolder)
{
    const std::filesystem::path dataPath =
        std::filesystem::path(__FILE__).parent_path() / dataFolder / "brs_small_case.txt";

    std::ifstream fin(dataPath);
    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open BRS data file: " + dataPath.string());
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(fin, line)) {
        size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) {
            line = line.substr(0, commentPos);
        }
        line = Tools::Trim(line);
        if (line.empty()) {
            continue;
        }

        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) {
            continue;
        }

        std::string key = Tools::ToLower(Tools::Trim(line.substr(0, eqPos)));
        std::string value = Tools::Trim(line.substr(eqPos + 1));
        kv[key] = value;
    }

    BRSRawInput input;
    input.stationCount = std::stoi(RequireKey(kv, "station_count"));
    input.horizon = std::stoi(RequireKey(kv, "time_horizon"));
    input.travelTimes = Tools::ParseIntList(RequireKey(kv, "travel_times"));
    if (static_cast<int>(input.travelTimes.size()) != input.stationCount - 1) {
        throw std::runtime_error("travel_times size must equal station_count - 1");
    }

    input.existingTrains = Tools::ParseIntList(RequireKey(kv, "existing_trains"));
    input.storageFlags = Tools::ParseIntList(RequireKey(kv, "storage_flags"));
    input.brsCosts = Tools::ParseDoubleList(RequireKey(kv, "brs_costs"));
    input.budget = std::stoi(RequireKey(kv, "budget"));
    input.trainCapacity = std::stoi(RequireKey(kv, "train_capacity"));
    input.minHeadway = std::stoi(RequireKey(kv, "min_headway"));
    input.lambdaCost = std::stod(RequireKey(kv, "lambda_cost"));

    int scenarioCount = std::stoi(RequireKey(kv, "scenario_count"));
    std::vector<double> probabilities = Tools::ParseDoubleList(RequireKey(kv, "scenario_probabilities"));
    if (static_cast<int>(probabilities.size()) != scenarioCount) {
        throw std::runtime_error("scenario_probabilities size does not match scenario_count");
    }

    input.scenarios.assign(static_cast<size_t>(scenarioCount), BRSScenarioData{});
    for (int w = 0; w < scenarioCount; ++w) {
        input.scenarios[static_cast<size_t>(w)].probability = probabilities[static_cast<size_t>(w)];
        input.scenarios[static_cast<size_t>(w)].originDemand =
            Tools::ParseDoubleList(RequireKey(kv, "scenario_b_" + std::to_string(w)));
        input.scenarios[static_cast<size_t>(w)].destinationDemand =
            Tools::ParseDoubleList(RequireKey(kv, "scenario_l_" + std::to_string(w)));
    }

    if (input.stationCount < 2) {
        throw std::runtime_error("station_count must be at least 2");
    }
    if (input.horizon < 1) {
        throw std::runtime_error("time_horizon must be >= 1");
    }
    if (static_cast<int>(input.existingTrains.size()) != input.stationCount) {
        throw std::runtime_error("existing_trains size must equal station_count");
    }
    if (static_cast<int>(input.storageFlags.size()) != input.stationCount) {
        throw std::runtime_error("storage_flags size must equal station_count");
    }
    if (static_cast<int>(input.brsCosts.size()) != input.stationCount) {
        throw std::runtime_error("brs_costs size must equal station_count");
    }
    if (input.trainCapacity <= 0) {
        throw std::runtime_error("train_capacity must be positive");
    }
    if (input.minHeadway < 0) {
        throw std::runtime_error("min_headway must be >= 0");
    }

    for (int tt : input.travelTimes) {
        if (tt <= 0) {
            throw std::runtime_error("travel_time values must be positive");
        }
    }

    for (int i = 0; i < input.stationCount; ++i) {
        if (!(input.storageFlags[static_cast<size_t>(i)] == 0 ||
              input.storageFlags[static_cast<size_t>(i)] == 1)) {
            throw std::runtime_error("storage_flags must contain only 0/1");
        }
    }

    double totalProbability = 0.0;
    for (const auto& scenario : input.scenarios) {
        totalProbability += scenario.probability;
        if (static_cast<int>(scenario.originDemand.size()) != input.stationCount ||
            static_cast<int>(scenario.destinationDemand.size()) != input.stationCount) {
            throw std::runtime_error("Scenario demand vectors must match station_count");
        }

        double totalOrigin = std::accumulate(
            scenario.originDemand.begin(),
            scenario.originDemand.end(),
            0.0);
        double totalDestination = std::accumulate(
            scenario.destinationDemand.begin(),
            scenario.destinationDemand.end(),
            0.0);
        if (std::fabs(totalOrigin - totalDestination) > 1e-6) {
            throw std::runtime_error("Each scenario must satisfy sum(B) == sum(L)");
        }
    }

    if (totalProbability <= 0.0) {
        throw std::runtime_error("Sum of scenario probabilities must be positive");
    }

    for (auto& scenario : input.scenarios) {
        scenario.probability /= totalProbability;
    }

    int storageCount = 0;
    for (int flag : input.storageFlags) {
        if (flag == 1) {
            ++storageCount;
        }
    }
    if (input.budget < 0 || input.budget > storageCount) {
        throw std::runtime_error("budget must be within [0, number_of_storage_stations]");
    }

    return input;
}

void AppendTravelArcs(const BRSRawInput& input, std::vector<BRSArcData>& arcs)
{
    for (int i = 0; i < input.stationCount - 1; ++i) {
        int travelTime = input.travelTimes[static_cast<size_t>(i)];

        for (int t = 0; t + travelTime <= input.horizon; ++t) {
            BRSArcData travelArc;
            travelArc.fromStation = i;
            travelArc.fromTime = t;
            travelArc.toStation = i + 1;
            travelArc.toTime = t + travelTime;
            travelArc.type = 'T';
            arcs.push_back(travelArc);
        }
    }
}

} // namespace

BRSDataInitializationStrategy_Solver::BRSDataInitializationStrategy_Solver(std::string dataFolder)
    : dataFolder(dataFolder)
{}

BRSDataInitializationStrategy_Benders::BRSDataInitializationStrategy_Benders(std::string dataFolder)
    : dataFolder(dataFolder)
{}


void BRSDataInitializationStrategy_Solver::DataInit(ProblemData& problemData)
{
    BRSRawInput input = LoadBRSInput(dataFolder);

    std::vector<int> storageStations;
    std::vector<double> storageCosts;
    storageStations.reserve(static_cast<size_t>(input.stationCount));
    storageCosts.reserve(static_cast<size_t>(input.stationCount));

    for (int i = 0; i < input.stationCount; ++i) { // alternative station and rescheduling cost
        if (input.storageFlags[static_cast<size_t>(i)] == 1) {
            storageStations.push_back(i);
            storageCosts.push_back(input.brsCosts[static_cast<size_t>(i)]);
        }
    }

    const int storageCount = static_cast<int>(storageStations.size());

    const int nodePerStation = input.horizon + 1; // timestamps 0, 1, ..., horizon(T)
    auto nodeIndex = [nodePerStation](int i, int t) {
        return i * nodePerStation + t;
    };

    std::vector<BRSArcData> arcs;
    for (int i = 0; i < input.stationCount; ++i) {
        for (int t = 0; t < input.horizon; ++t) {
            BRSArcData arc;
            arc.fromStation = i;
            arc.fromTime = t;
            arc.toStation = i;
            arc.toTime = t + 1;
            arc.type = 'W'; // waiting arcs, non at the last time step (T = horizon)
            arcs.push_back(arc);
        }
    }

    AppendTravelArcs(input, arcs);

    for (int s = 0; s < storageCount; ++s) {
        int station = storageStations[static_cast<size_t>(s)];
        for (int t = 0; t < input.horizon; ++t) {
            BRSArcData arc;
            arc.fromStation = -1;
            arc.fromTime = -1;
            arc.toStation = station;
            arc.toTime = t;
            arc.type = 'I'; // assignment arcs, non at the last time step
            arc.storageIndex = s;
            arcs.push_back(arc);
        }
    }

    const int nodeCount = input.stationCount * nodePerStation;
    std::vector<std::vector<int>> nodeIncomingX(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeOutgoingX(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeIncomingY(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeOutgoingY(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeIncomingTravelX(static_cast<size_t>(nodeCount)); // Outgoing one is useless
    std::vector<std::vector<int>> assignmentArcsByStorage(static_cast<size_t>(storageCount));

    std::vector<int> xToYArc(static_cast<size_t>(arcs.size()), -1);
    std::vector<int> yToXArc;

    for (int a = 0; a < static_cast<int>(arcs.size()); ++a) { // all arcs (I,W,T)
        const auto& arc = arcs[static_cast<size_t>(a)];
        int toNode = nodeIndex(arc.toStation, arc.toTime);
        nodeIncomingX[static_cast<size_t>(toNode)].push_back(a);

        if (arc.type != 'I') { // W,T; xToYArc: 0, 1, 2, -1, 3, ...; yToXArc: 0, 1, 2, 3, ...
            int fromNode = nodeIndex(arc.fromStation, arc.fromTime);
            nodeOutgoingX[static_cast<size_t>(fromNode)].push_back(a);

            int yIdx = static_cast<int>(yToXArc.size());
            xToYArc[static_cast<size_t>(a)] = yIdx;
            yToXArc.push_back(a); // connecting nodeOutgoingX and Y
            nodeIncomingY[static_cast<size_t>(toNode)].push_back(yIdx);
            nodeOutgoingY[static_cast<size_t>(fromNode)].push_back(yIdx);
        }

        if (arc.type == 'T') {
            nodeIncomingTravelX[static_cast<size_t>(toNode)].push_back(a);
        }

        if (arc.type == 'I') {
            assignmentArcsByStorage[static_cast<size_t>(arc.storageIndex)].push_back(a);
        }
    }

    const int scenarioCount = static_cast<int>(input.scenarios.size());
    const int arcCount = static_cast<int>(arcs.size());
    const int yArcCount = static_cast<int>(yToXArc.size());
    const int activeAlphaHorizon = std::max(0, input.horizon - 1);

    const int zBase = 0;
    const int xBase = zBase + storageCount;
    const int yBase = xBase + scenarioCount * arcCount;
    const int alphaBase = yBase + scenarioCount * yArcCount;
    const int totalVars = alphaBase + scenarioCount * input.stationCount * activeAlphaHorizon + 1;

    auto zIndex = [zBase](int s) {
        return zBase + s;
    };
    auto xIndex = [xBase, arcCount](int w, int a) {
        return xBase + w * arcCount + a;
    };
    auto yIndex = [yBase, yArcCount](int w, int a) {
        return yBase + w * yArcCount + a;
    };
    auto alphaIndex = [alphaBase, activeAlphaHorizon, stationCount = input.stationCount](int w, int i, int t) {
        int local = i * activeAlphaHorizon + (t - 1);
        return alphaBase + w * stationCount * activeAlphaHorizon + local;
    };

    std::vector<ProblemDataVar> vars;
    vars.reserve(static_cast<size_t>(totalVars));

    for (int s = 0; s < storageCount; ++s) {
        ProblemDataVar var;
        var.lb = 0.0;
        var.ub = 1.0;
        var.obj = input.lambdaCost * storageCosts[static_cast<size_t>(s)]; // allocation
        var.type = 'B';
        var.name = "z_station_" + std::to_string(storageStations[static_cast<size_t>(s)]);
        vars.push_back(var);
    }

    for (int w = 0; w < scenarioCount; ++w) {
        for (int a = 0; a < arcCount; ++a) {
            ProblemDataVar var;
            var.lb = 0.0;
            var.ub = 1.0;
            var.obj = 0.0;
            var.type = 'B';
            var.name = "x_w" + std::to_string(w) + "_a" + std::to_string(a);
            vars.push_back(var);
        }
    }

    for (int w = 0; w < scenarioCount; ++w) {
        const double scenarioTotalDemand = std::accumulate(
            input.scenarios[static_cast<size_t>(w)].originDemand.begin(),
            input.scenarios[static_cast<size_t>(w)].originDemand.end(),
            0.0);
        for (int a = 0; a < yArcCount; ++a) {
            ProblemDataVar var;
            var.lb = 0.0;
            var.ub = scenarioTotalDemand;
            var.obj = 0.0;
            var.type = 'C';
            var.name = "y_w" + std::to_string(w) + "_a" + std::to_string(a);
            vars.push_back(var);
        }
    }

    for (int w = 0; w < scenarioCount; ++w) {
        for (int i = 0; i < input.stationCount; ++i) {
            for (int t = 1; t < input.horizon; ++t) {
                ProblemDataVar var;
                var.lb = 0.0;
                var.ub = GRB_INFINITY;
                var.obj = input.scenarios[static_cast<size_t>(w)].probability *
                          (static_cast<double>(t) - static_cast<double>(input.horizon)); // p(t-T)
                var.type = 'C';
                var.name = "alpha_w" + std::to_string(w) + "_i" + std::to_string(i) +
                           "_t" + std::to_string(t);
                vars.push_back(var);
            }
        }
    }

    double objectiveConstant = 0.0;
    for (int w = 0; w < scenarioCount; ++w) {
        const double totalDestination = std::accumulate(
            input.scenarios[static_cast<size_t>(w)].destinationDemand.begin(),
            input.scenarios[static_cast<size_t>(w)].destinationDemand.end(),
            0.0);
        objectiveConstant += input.scenarios[static_cast<size_t>(w)].probability *
                             static_cast<double>(input.horizon) * totalDestination;
    }
    {
        ProblemDataVar var;
        var.lb = 1.0;
        var.ub = 1.0;
        var.obj = objectiveConstant; // LT
        var.type = 'C';
        var.name = "obj_constant";
        vars.push_back(var);
    }

    if (static_cast<int>(vars.size()) != totalVars) {
        throw std::runtime_error("BARP-S solver variable size mismatch");
    }

    std::vector<ProblemDataConstr> constrs;
    constrs.reserve(
        static_cast<size_t>(1 + // budget
                            scenarioCount *
                                (input.stationCount * input.horizon + // train balance
                                 storageCount + // assignment
                                 input.stationCount * (input.horizon + 1) + // min headway
                                 input.stationCount * input.horizon + // passenger balance on t = 0..T-1
                                 input.stationCount + // passenger alighting constraints
                                 input.stationCount * activeAlphaHorizon + // passenger alighting link constraints
                                 yArcCount)));

    {
        ProblemDataConstr constr;
        constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
        for (int s = 0; s < storageCount; ++s) {
            constr.coeffs[static_cast<size_t>(zIndex(s))] = 1.0;
        }
        constr.sense = '<';
        constr.rhs = static_cast<double>(input.budget);
        constr.name = "brs_budget";
        constrs.push_back(constr);
    }

    for (int w = 0; w < scenarioCount; ++w) {
        for (int i = 0; i < input.stationCount; ++i) {
            for (int t = 0; t < input.horizon; ++t) {
                ProblemDataConstr constr;
                constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
                int node = nodeIndex(i, t);
                for (int arcIdx : nodeOutgoingX[static_cast<size_t>(node)]) {
                    constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] += 1.0;
                }
                for (int arcIdx : nodeIncomingX[static_cast<size_t>(node)]) {
                    constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] -= 1.0;
                }
                constr.sense = '=';
                constr.rhs = (t == 0) ? static_cast<double>(input.existingTrains[static_cast<size_t>(i)]) : 0.0;
                constr.name = "train_balance_w" + std::to_string(w) + "_i" + std::to_string(i) +
                              "_t" + std::to_string(t);
                constrs.push_back(constr);
            }
        }

        for (int s = 0; s < storageCount; ++s) {
            ProblemDataConstr constr;
            constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
            for (int arcIdx : assignmentArcsByStorage[static_cast<size_t>(s)]) {
                constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] += 1.0;
            }
            constr.coeffs[static_cast<size_t>(zIndex(s))] = -1.0;
            constr.sense = '<';
            constr.rhs = 0.0;
            constr.name = "assign_w" + std::to_string(w) + "_s" + std::to_string(s);
            constrs.push_back(constr);
        }

        for (int i = 0; i < input.stationCount; ++i) {
            for (int t = 0; t <= input.horizon; ++t) {
                ProblemDataConstr constr;
                constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
                int latest = std::min(input.horizon, t + input.minHeadway);
                for (int tau = t; tau <= latest; ++tau) {
                    int node = nodeIndex(i, tau);
                    for (int arcIdx : nodeIncomingTravelX[static_cast<size_t>(node)]) {
                        constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] += 1.0;
                    }
                }
                constr.sense = '<';
                constr.rhs = 1.0;
                constr.name = "headway_w" + std::to_string(w) + "_i" + std::to_string(i) +
                              "_t" + std::to_string(t);
                constrs.push_back(constr);
            }
        }

        for (int i = 0; i < input.stationCount; ++i) {
            for (int t = 0; t < input.horizon; ++t) {
                ProblemDataConstr constr;
                constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
                int node = nodeIndex(i, t);

                for (int yArcIdx : nodeIncomingY[static_cast<size_t>(node)]) {
                    constr.coeffs[static_cast<size_t>(yIndex(w, yArcIdx))] += 1.0;
                }
                for (int yArcIdx : nodeOutgoingY[static_cast<size_t>(node)]) {
                    constr.coeffs[static_cast<size_t>(yIndex(w, yArcIdx))] -= 1.0;
                }

                if (t == 0) {
                    constr.sense = '=';
                    constr.rhs = -input.scenarios[static_cast<size_t>(w)]
                                      .originDemand[static_cast<size_t>(i)];
                    constr.name = "passenger_balance0_w" + std::to_string(w) + "_i" +
                                  std::to_string(i);
                } else {
                    int aIdx = alphaIndex(w, i, t);
                    constr.coeffs[static_cast<size_t>(aIdx)] -= 1.0;
                    constr.sense = '=';
                    constr.rhs = 0.0;
                    constr.name = "passenger_balance_w" + std::to_string(w) + "_i" +
                                  std::to_string(i) + "_t" + std::to_string(t);
                }
                constrs.push_back(constr);
            }
        }

        for (int i = 0; i < input.stationCount; ++i) {
            ProblemDataConstr constr;
            constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
            for (int t = 1; t < input.horizon; ++t) {
                int aIdx = alphaIndex(w, i, t);
                constr.coeffs[static_cast<size_t>(aIdx)] += 1.0;
            }
            constr.sense = '<';
            constr.rhs = input.scenarios[static_cast<size_t>(w)].destinationDemand[static_cast<size_t>(i)];
            constr.name = "alight_total_w" + std::to_string(w) + "_i" + std::to_string(i);
            constrs.push_back(constr);
        }

        double bigM = std::accumulate(
            input.scenarios[static_cast<size_t>(w)].originDemand.begin(),
            input.scenarios[static_cast<size_t>(w)].originDemand.end(),
            0.0);

        for (int i = 0; i < input.stationCount; ++i) {
            for (int t = 1; t < input.horizon; ++t) {
                ProblemDataConstr constr;
                constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);

                int aIdx = alphaIndex(w, i, t);
                constr.coeffs[static_cast<size_t>(aIdx)] += 1.0;

                int node = nodeIndex(i, t);
                for (int arcIdx : nodeIncomingTravelX[static_cast<size_t>(node)]) {
                    constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] -= bigM;
                }

                constr.sense = '<';
                constr.rhs = 0.0;
                constr.name = "alight_link_w" + std::to_string(w) + "_i" + std::to_string(i) +
                              "_t" + std::to_string(t);
                constrs.push_back(constr);
            }
        }

        for (int yArcIdx = 0; yArcIdx < yArcCount; ++yArcIdx) {
            int arcIdx = yToXArc[static_cast<size_t>(yArcIdx)];
            if (arcs[static_cast<size_t>(arcIdx)].type != 'T') {
                continue;
            }

            ProblemDataConstr constr;
            constr.coeffs.assign(static_cast<size_t>(totalVars), 0.0);
            constr.coeffs[static_cast<size_t>(yIndex(w, yArcIdx))] += 1.0;
            constr.coeffs[static_cast<size_t>(xIndex(w, arcIdx))] -= static_cast<double>(input.trainCapacity);
            constr.sense = '<';
            constr.rhs = 0.0;
            constr.name = "travel_cap_w" + std::to_string(w) + "_a" + std::to_string(yArcIdx);
            constrs.push_back(constr);
        }
    }

    problemData.addData("vars", vars);
    problemData.addData("constrs", constrs);
    problemData.addData("obj", GRB_MINIMIZE);

    std::cout << "Loaded BRS BARP-S data for direct solver." << std::endl;
    std::cout << "Vars=" << vars.size() << ", Constrs=" << constrs.size() << std::endl;
}

void BRSDataInitializationStrategy_Benders::DataInit(ProblemData& problemData)
{
    BRSRawInput input = LoadBRSInput(dataFolder);

    std::vector<int> storageStations;
    std::vector<double> storageCosts;
    storageStations.reserve(static_cast<size_t>(input.stationCount));
    storageCosts.reserve(static_cast<size_t>(input.stationCount));

    for (int i = 0; i < input.stationCount; ++i) {
        if (input.storageFlags[static_cast<size_t>(i)] == 1) {
            storageStations.push_back(i);
            storageCosts.push_back(input.brsCosts[static_cast<size_t>(i)]);
        }
    }

    const int storageCount = static_cast<int>(storageStations.size());

    std::vector<ProblemDataVar> masterVars;
    masterVars.reserve(static_cast<size_t>(storageCount));
    for (int s = 0; s < storageCount; ++s) {
        ProblemDataVar var;
        var.lb = 0.0;
        var.ub = 1.0;
        var.obj = input.lambdaCost * storageCosts[static_cast<size_t>(s)];
        var.type = 'B';
        var.name = "z_station_" + std::to_string(storageStations[static_cast<size_t>(s)]);
        masterVars.push_back(var);
    } // finished master problem variable definition

    const int nodePerStation = input.horizon + 1;
    auto nodeIndex = [nodePerStation](int i, int t) {
        return i * nodePerStation + t;
    };

    std::vector<BRSArcData> arcs;
    for (int i = 0; i < input.stationCount; ++i) {
        for (int t = 0; t < input.horizon; ++t) {
            BRSArcData arc;
            arc.fromStation = i;
            arc.fromTime = t;
            arc.toStation = i;
            arc.toTime = t + 1;
            arc.type = 'W';
            arcs.push_back(arc);
        }
    }

    AppendTravelArcs(input, arcs);

    for (int s = 0; s < storageCount; ++s) {
        int station = storageStations[static_cast<size_t>(s)];
        for (int t = 0; t < input.horizon; ++t) {
            BRSArcData arc;
            arc.fromStation = -1;
            arc.fromTime = -1;
            arc.toStation = station;
            arc.toTime = t;
            arc.type = 'I';
            arc.storageIndex = s;
            arcs.push_back(arc);
        }
    }

    const int nodeCount = input.stationCount * nodePerStation;
    std::vector<std::vector<int>> nodeIncomingX(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeOutgoingX(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeIncomingY(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeOutgoingY(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> nodeIncomingTravelX(static_cast<size_t>(nodeCount));
    std::vector<std::vector<int>> assignmentArcsByStorage(static_cast<size_t>(storageCount));

    std::vector<int> xToYArc(static_cast<size_t>(arcs.size()), -1);
    std::vector<int> yToXArc;

    for (int a = 0; a < static_cast<int>(arcs.size()); ++a) {
        const auto& arc = arcs[static_cast<size_t>(a)];
        int toNode = nodeIndex(arc.toStation, arc.toTime);
        nodeIncomingX[static_cast<size_t>(toNode)].push_back(a);

        if (arc.fromStation >= 0) {
            int fromNode = nodeIndex(arc.fromStation, arc.fromTime);
            nodeOutgoingX[static_cast<size_t>(fromNode)].push_back(a);
        }

        if (arc.type != 'I') {
            int yIdx = static_cast<int>(yToXArc.size());
            xToYArc[static_cast<size_t>(a)] = yIdx;
            yToXArc.push_back(a);
            nodeIncomingY[static_cast<size_t>(toNode)].push_back(yIdx);
            if (arc.fromStation >= 0) {
                int fromNode = nodeIndex(arc.fromStation, arc.fromTime);
                nodeOutgoingY[static_cast<size_t>(fromNode)].push_back(yIdx);
            }
        }

        if (arc.type == 'T') {
            nodeIncomingTravelX[static_cast<size_t>(toNode)].push_back(a);
        }

        if (arc.type == 'I') {
            assignmentArcsByStorage[static_cast<size_t>(arc.storageIndex)].push_back(a);
        }
    }

    problemData.addData("masterVars", masterVars);
    problemData.addData("brsStationCount", input.stationCount);
    problemData.addData("brsHorizon", input.horizon);
    problemData.addData("brsTravelTimes", input.travelTimes);
    problemData.addData("brsExistingTrains", input.existingTrains);
    problemData.addData("brsStorageStations", storageStations);
    problemData.addData("brsBudget", input.budget);
    problemData.addData("brsTrainCapacity", input.trainCapacity);
    problemData.addData("brsMinHeadway", input.minHeadway);
    problemData.addData("brsStorageCount", storageCount);
    problemData.addData("brsScenarios", input.scenarios);
    problemData.addData("brsArcs", arcs);
    problemData.addData("brsXToYArc", xToYArc);
    problemData.addData("brsYToXArc", yToXArc);
    problemData.addData("brsNodeIncomingX", nodeIncomingX);
    problemData.addData("brsNodeOutgoingX", nodeOutgoingX);
    problemData.addData("brsNodeIncomingY", nodeIncomingY);
    problemData.addData("brsNodeOutgoingY", nodeOutgoingY);
    problemData.addData("brsNodeIncomingTravelX", nodeIncomingTravelX);
    problemData.addData("brsAssignArcsByStorage", assignmentArcsByStorage);

    std::cout << "Loaded BRS BARP-S data for Benders decomposition." << std::endl;
    std::cout << "Stations=" << input.stationCount
              << ", horizon=" << input.horizon
              << ", scenarios=" << input.scenarios.size()
              << ", master vars(z)=" << masterVars.size() << std::endl;
}

std::vector<ProblemDataConstr> BRSDataInitializationStrategy_Benders::ConstrInit(ProblemData& problemData)
{
    int storageCount = problemData.getData<int>("brsStorageCount");
    int budget = problemData.getData<int>("brsBudget");

    ProblemDataConstr budgetConstr;
    budgetConstr.coeffs.assign(static_cast<size_t>(storageCount), 1.0);
    budgetConstr.sense = '<';
    budgetConstr.rhs = static_cast<double>(budget);
    budgetConstr.name = "brs_budget";

    return {budgetConstr};
}

void BRSSubProblemStrategy_Benders::InitSubProblem(const ProblemData& problemData, GRBModel& subModel,
    BendersSubProblemContext& context)
{
    const int stationCount = problemData.getData<int>("brsStationCount");
    const int horizon = problemData.getData<int>("brsHorizon");
    const int minHeadway = problemData.getData<int>("brsMinHeadway");
    const int trainCapacity = problemData.getData<int>("brsTrainCapacity");
    const int storageCount = problemData.getData<int>("brsStorageCount");

    const auto& existingTrains = problemData.getData<std::vector<int>>("brsExistingTrains");
    const auto& scenarios = problemData.getData<std::vector<BRSScenarioData>>("brsScenarios");
    const auto& arcs = problemData.getData<std::vector<BRSArcData>>("brsArcs");
    const auto& yToXArc = problemData.getData<std::vector<int>>("brsYToXArc");
    const auto& nodeIncomingX = problemData.getData<std::vector<std::vector<int>>>("brsNodeIncomingX");
    const auto& nodeOutgoingX = problemData.getData<std::vector<std::vector<int>>>("brsNodeOutgoingX");
    const auto& nodeIncomingY = problemData.getData<std::vector<std::vector<int>>>("brsNodeIncomingY");
    const auto& nodeOutgoingY = problemData.getData<std::vector<std::vector<int>>>("brsNodeOutgoingY");
    const auto& nodeIncomingTravelX = problemData.getData<std::vector<std::vector<int>>>("brsNodeIncomingTravelX");
    const auto& assignmentArcsByStorage = problemData.getData<std::vector<std::vector<int>>>("brsAssignArcsByStorage");

    const int scenarioCount = static_cast<int>(scenarios.size());
    const int arcCount = static_cast<int>(arcs.size());
    const int yArcCount = static_cast<int>(yToXArc.size());
    const int activeAlphaHorizon = std::max(0, horizon - 1);
    const int alphaCountPerScenario = stationCount * activeAlphaHorizon;

    const int nodePerStation = horizon + 1;
    auto nodeIndex = [nodePerStation](int i, int t) {
        return i * nodePerStation + t;
    };

    context.Clear();
    auto& xVars = context.EnsureVarGroup(kVarGroupX);
    auto& yVars = context.EnsureVarGroup(kVarGroupY);
    auto& alphaVars = context.EnsureVarGroup(kVarGroupAlpha);
    auto& assignmentConstrs = context.EnsureConstrGroup(kConstrGroupAssign);

    xVars.reserve(static_cast<size_t>(scenarioCount * arcCount));
    yVars.reserve(static_cast<size_t>(scenarioCount * yArcCount));
    alphaVars.reserve(static_cast<size_t>(scenarioCount * alphaCountPerScenario));
    assignmentConstrs.reserve(static_cast<size_t>(scenarioCount * storageCount));

    subModel.set(GRB_IntParam_InfUnbdInfo, 1);
    subModel.set(GRB_IntParam_DualReductions, 0);

    auto xAt = [&xVars, arcCount](int w, int arcIdx) {
        return xVars[static_cast<size_t>(w * arcCount + arcIdx)];
    };
    auto yAt = [&yVars, yArcCount](int w, int yArcIdx) {
        return yVars[static_cast<size_t>(w * yArcCount + yArcIdx)];
    };
    auto alphaAt = [&alphaVars, activeAlphaHorizon, alphaCountPerScenario](int w, int i, int t) {
        int local = i * activeAlphaHorizon + (t - 1);
        return alphaVars[static_cast<size_t>(w * alphaCountPerScenario + local)];
    };

    for (int w = 0; w < scenarioCount; ++w) {
        for (int a = 0; a < arcCount; ++a) {
            xVars.push_back(subModel.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                "x_w" + std::to_string(w) + "_a" + std::to_string(a)));
        }

        const double scenarioTotalDemand = std::accumulate(
            scenarios[static_cast<size_t>(w)].originDemand.begin(),
            scenarios[static_cast<size_t>(w)].originDemand.end(), 0.0);
        for (int a = 0; a < yArcCount; ++a) {
            yVars.push_back(subModel.addVar(0.0, scenarioTotalDemand, 0.0, GRB_CONTINUOUS,
                "y_w" + std::to_string(w) + "_a" + std::to_string(a)));
        }

        for (int i = 0; i < stationCount; ++i) {
            for (int t = 1; t < horizon; ++t) {
                alphaVars.push_back(subModel.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                    "alpha_w" + std::to_string(w) + "_i" + std::to_string(i) + "_t" + std::to_string(t)));
            }
        }
    }

    for (int w = 0; w < scenarioCount; ++w) {
        for (int i = 0; i < stationCount; ++i) {
            for (int t = 0; t < horizon; ++t) {
                int node = nodeIndex(i, t);
                GRBLinExpr lhs = 0.0;
                for (int arcIdx : nodeOutgoingX[static_cast<size_t>(node)]) {
                    lhs += xAt(w, arcIdx);
                }
                for (int arcIdx : nodeIncomingX[static_cast<size_t>(node)]) {
                    lhs -= xAt(w, arcIdx);
                }
                double rhs = (t == 0) ? static_cast<double>(existingTrains[static_cast<size_t>(i)]) : 0.0;
                subModel.addConstr(lhs, '=', rhs,
                    "train_balance_w" + std::to_string(w) + "_i" + std::to_string(i) + "_t" + std::to_string(t));
            }
        }

        for (int s = 0; s < storageCount; ++s) {
            GRBLinExpr lhs = 0.0;
            for (int arcIdx : assignmentArcsByStorage[static_cast<size_t>(s)]) {
                lhs += xAt(w, arcIdx);
            }
            assignmentConstrs.push_back(subModel.addConstr(lhs, '<', 0.0, // rhs is provided by master problem
                "assign_w" + std::to_string(w) + "_s" + std::to_string(s)));
        }

        for (int i = 0; i < stationCount; ++i) {
            for (int t = 0; t <= horizon; ++t) {
                GRBLinExpr lhs = 0.0;
                int latest = std::min(horizon, t + minHeadway);
                for (int tau = t; tau <= latest; ++tau) {
                    int node = nodeIndex(i, tau);
                    for (int arcIdx : nodeIncomingTravelX[static_cast<size_t>(node)]) {
                        lhs += xAt(w, arcIdx);
                    }
                }
                subModel.addConstr(lhs, '<', 1.0,
                    "headway_w" + std::to_string(w) + "_i" + std::to_string(i) + "_t" + std::to_string(t));
            }
        }

        for (int i = 0; i < stationCount; ++i) {
            for (int t = 0; t < horizon; ++t) {
                int node = nodeIndex(i, t);
                GRBLinExpr lhs = 0.0;
                for (int yArcIdx : nodeIncomingY[static_cast<size_t>(node)]) {
                    lhs += yAt(w, yArcIdx);
                }
                for (int yArcIdx : nodeOutgoingY[static_cast<size_t>(node)]) {
                    lhs -= yAt(w, yArcIdx);
                }

                if (t == 0) {
                    subModel.addConstr(lhs, '=',
                        -scenarios[static_cast<size_t>(w)].originDemand[static_cast<size_t>(i)],
                        "passenger_balance0_w" + std::to_string(w) + "_i" + std::to_string(i));
                } else {
                    lhs -= alphaAt(w, i, t);
                    subModel.addConstr(lhs, '=', 0.0,
                        "passenger_balance_w" + std::to_string(w) + "_i" + std::to_string(i) +
                            "_t" + std::to_string(t));
                }
            }
        }

        for (int i = 0; i < stationCount; ++i) {
            GRBLinExpr lhs = 0.0;
            for (int t = 1; t < horizon; ++t) {
                lhs += alphaAt(w, i, t);
            }
            subModel.addConstr(lhs, '<', scenarios[static_cast<size_t>(w)].destinationDemand[static_cast<size_t>(i)],
                "alight_total_w" + std::to_string(w) + "_i" + std::to_string(i));
        }

        double bigM = std::accumulate( // TODO: 用数组表示，降低大M影响
            scenarios[static_cast<size_t>(w)].originDemand.begin(),
            scenarios[static_cast<size_t>(w)].originDemand.end(), 0.0);

        for (int i = 0; i < stationCount; ++i) {
            for (int t = 1; t < horizon; ++t) {
                int node = nodeIndex(i, t);
                GRBLinExpr arriveTrain = 0.0;
                for (int arcIdx : nodeIncomingTravelX[static_cast<size_t>(node)]) {
                    arriveTrain += xAt(w, arcIdx);
                }
                subModel.addConstr(alphaAt(w, i, t) <= bigM * arriveTrain,
                    "alight_link_w" + std::to_string(w) + "_i" + std::to_string(i) + "_t" + std::to_string(t));
            }
        }

        for (int yArcIdx = 0; yArcIdx < yArcCount; ++yArcIdx) {
            int arcIdx = yToXArc[static_cast<size_t>(yArcIdx)];
            if (arcs[static_cast<size_t>(arcIdx)].type == 'T') {
                subModel.addConstr(
                    yAt(w, yArcIdx) <= static_cast<double>(trainCapacity) * xAt(w, arcIdx),
                    "travel_cap_w" + std::to_string(w) + "_a" + std::to_string(yArcIdx));
            }
        }
    }

    GRBLinExpr objective = 0.0;
    for (int w = 0; w < scenarioCount; ++w) {
        const double probability = scenarios[static_cast<size_t>(w)].probability;
        const double totalDestination = std::accumulate(
            scenarios[static_cast<size_t>(w)].destinationDemand.begin(),
            scenarios[static_cast<size_t>(w)].destinationDemand.end(), 0.0);

        objective += probability * static_cast<double>(horizon) * totalDestination;
        for (int i = 0; i < stationCount; ++i) {
            for (int t = 1; t < horizon; ++t) {
                objective += probability * (static_cast<double>(t) - static_cast<double>(horizon)) *
                             alphaAt(w, i, t);
            }
        }
    }

    subModel.setObjective(objective, GRB_MINIMIZE);
    subModel.update();

    lowerBoundReady = false;
    for (auto& assignConstr : assignmentConstrs) {
        assignConstr.set(GRB_DoubleAttr_RHS, 1.0);
    }
    subModel.update();

    subModel.optimize();
    int status = subModel.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
        throw std::runtime_error(
            "Failed to compute BARP-S Benders lower bound, status: " + std::to_string(status));
    }

    qLowerBound = subModel.get(GRB_DoubleAttr_ObjVal);
    lowerBoundReady = true;

    subModel.update();
}

void BRSSubProblemStrategy_Benders::UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
    BendersSubProblemContext& context, const std::vector<double>& zValues)
{
    const int scenarioCount = static_cast<int>(problemData.getData<std::vector<BRSScenarioData>>("brsScenarios").size());
    const int storageCount = problemData.getData<int>("brsStorageCount");

    auto& assignConstrs = context.EnsureConstrGroup(kConstrGroupAssign);

    for (int w = 0; w < scenarioCount; ++w) {
        for (int s = 0; s < storageCount; ++s) {
            assignConstrs[static_cast<size_t>(w * storageCount + s)]
                .set(GRB_DoubleAttr_RHS, zValues[static_cast<size_t>(s)]);
        }
    }

    subModel.update();
}

Status BRSSubProblemStrategy_Benders::SolveSubProblem( const ProblemData& problemData, GRBModel& subModel,
    BendersSubProblemContext& context, const std::vector<double>& zValues, BendersCutInfo& cutInfo, double& subObj)
{
    const int storageCount = problemData.getData<int>("brsStorageCount");

    if (static_cast<int>(zValues.size()) != storageCount) {
        std::cerr << "BRS sub-problem: z value size mismatch" << std::endl;
        return ERROR;
    }

    subModel.optimize();
    int status = subModel.get(GRB_IntAttr_Status);
    if (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD || status == GRB_UNBOUNDED) {
        cutInfo.isOptimalityCut = false;
        cutInfo.sense = '>';
        cutInfo.yCoeffs.assign(static_cast<size_t>(storageCount), 0.0);

        int selectedCount = 0;
        for (int s = 0; s < storageCount; ++s) {
            bool selected = zValues[static_cast<size_t>(s)] > 0.5;
            if (selected) {
                ++selectedCount;
                cutInfo.yCoeffs[static_cast<size_t>(s)] = -1.0;
            } else {
                cutInfo.yCoeffs[static_cast<size_t>(s)] = 1.0;
            }
        }

        // No-good cut for binary z: sum_{s in S1}(1-z_s) + sum_{s in S0} z_s >= 1.
        cutInfo.rhs = 1.0 - static_cast<double>(selectedCount);
        cutInfo.constant = 0.0;
        subObj = GRB_INFINITY;
        return OK;
    }

    if (status != GRB_OPTIMAL) {
        std::cerr << "BRS sub-problem ended with unexpected status: " << status << std::endl;
        return ERROR;
    }

    subObj = subModel.get(GRB_DoubleAttr_ObjVal);

    double lowerBound = lowerBoundReady ? qLowerBound : 0.0;
    double delta = std::max(0.0, subObj - lowerBound);

    int selectedCount = 0;
    cutInfo.yCoeffs.assign(static_cast<size_t>(storageCount), 0.0);
    for (int s = 0; s < storageCount; ++s) {
        bool selected = zValues[static_cast<size_t>(s)] > 0.5;
        if (selected) {
            ++selectedCount;
            cutInfo.yCoeffs[static_cast<size_t>(s)] = delta;
        } else {
            cutInfo.yCoeffs[static_cast<size_t>(s)] = -delta;
        }
    }

    cutInfo.constant = lowerBound + delta * (1.0 - static_cast<double>(selectedCount));
    cutInfo.rhs = 0.0;
    cutInfo.isOptimalityCut = true;
    cutInfo.sense = '>';

    return OK;
}
