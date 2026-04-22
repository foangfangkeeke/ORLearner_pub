#pragma once

#include "using_solver.hpp"
#include "benders_decomposition.hpp"
#include "barp_s_integer_l_shaped.hpp"

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
    explicit BRSDataInitializationStrategy_Solver(std::string dataFolder = "test_data");
    void DataInit(ProblemData& problemData) override;

private:
    std::string dataFolder;
};

class BRSDataInitializationStrategy_Benders : public IDataInitializationStrategy_Benders {
public:
    explicit BRSDataInitializationStrategy_Benders(std::string dataFolder = "test_data");
    void DataInit(ProblemData& problemData) override;
    std::vector<ProblemDataConstr> ConstrInit(ProblemData& problemData) override;

private:
    std::string dataFolder;
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

class BRSDataInitializationStrategy_LShaped : public IDataInitializationStrategy_IntegerLShaped {
public:
    explicit BRSDataInitializationStrategy_LShaped(std::string dataFolder = "test_data");
    void DataInit(ProblemData& problemData) override;
    std::vector<ProblemDataConstr> ConstrInit(ProblemData& problemData) override;
    std::vector<double> BuildWarmStartMasterValues(const ProblemData& problemData) const override;

private:
    std::string dataFolder;
};

class BRSSubProblemStrategy_LShaped : public ISubProblemStrategy_IntegerLShaped {
public:
    void InitSubProblem(const ProblemData& problemData, GRBModel& subModel,
        IntegerLShapedSubProblemContext& context) override;

    void UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
        IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues) override;

    Status SolveSubProblem(const ProblemData& problemData, GRBModel& subModel,
        IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues,
        IntegerLShapedCutInfo& cutInfo, double& subObj) override;

    Status SolveRelaxedSubProblem(const ProblemData& problemData, GRBModel& subModel,
        IntegerLShapedSubProblemContext& context, const std::vector<double>& zValues,
        IntegerLShapedCutInfo& cutInfo, double& subObj) override;

    std::unique_ptr<ISubProblemStrategy_IntegerLShaped> Clone() const override
    {
        return std::make_unique<BRSSubProblemStrategy_LShaped>(*this);
    }

private:
    BRSSubProblemStrategy_Benders bendersImpl;
};
