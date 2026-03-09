#pragma once

#include "using_solver.hpp"

#include <vector>

struct BRSScenarioData {
    double probability = 0.0;
    std::vector<double> originDemand;
    std::vector<double> destinationDemand;
    std::vector<double> dynamicArrivals;
    std::vector<double> cumulativeAlightLimits;
};

struct BRSArcData {
    int fromStation = -1;
    int fromTime = -1;
    int toStation = -1;
    int toTime = -1;
    char type = 'W';
    int storageIndex = -1;
};

class BRSDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    void DataInit(ProblemData& problemData) override;
};
