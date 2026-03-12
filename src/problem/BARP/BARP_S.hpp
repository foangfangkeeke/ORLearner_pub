#pragma once

#include "using_solver.hpp"
#include "benders_decomposition.hpp"

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

class BRSDataInitializationStrategy_Benders : public IDataInitializationStrategy_Benders {
public:
    void DataInit(ProblemData& problemData) override;
    std::vector<ProblemDataConstr> ConstrInit(ProblemData& problemData) override;
};

class BRSSubProblemStrategy_Benders : public ISubProblemStrategy_Benders {
public:
    void InitSubProblem(const ProblemData& problemData, GRBModel& subModel,
        BendersSubProblemContext& context) override;

    void UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
        BendersSubProblemContext& context, const std::vector<double>& yValues) override;

    Status SolveSubProblem(const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context,
        const std::vector<double>& yValues, BendersCutInfo& cutInfo, double& subObj) override;

    std::unique_ptr<ISubProblemStrategy_Benders> Clone() const override
    {
        return std::make_unique<BRSSubProblemStrategy_Benders>(*this);
    }

private:
    double qLowerBound = 0.0;
    bool lowerBoundReady = false;
};
