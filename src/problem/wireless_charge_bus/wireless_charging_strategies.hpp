#include <string>
#include "using_solver.hpp"
#include "barp_s_integer_l_shaped.hpp"

class WirelessChargingDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
	WirelessChargingDataInitializationStrategy_Solver(std::string dataFolder);
	Status DataInit(GRBModel& model) override;

private:
	std::string dataFolder;
};

class WirelessChargingDataInitializationStrategy_LShaped : public IDataInitializationStrategy_IntegerLShaped {
public:
    explicit WirelessChargingDataInitializationStrategy_LShaped(std::string dataFolder = "test_data");
    void DataInit(ProblemData& data) override;
    std::vector<ProblemDataConstr> ConstrInit(ProblemData& data) override;
    std::vector<double> BuildWarmStartMasterValues(const ProblemData& data) const override;
    bool IsWarmStartMasterFeasible(const ProblemData& data, const std::vector<double>& zValues,
        double tolerance) const override;

private:
    std::string dataFolder;
};

class WirelessChargingSubProblemStrategy_LShaped : public ISubProblemStrategy_IntegerLShaped {
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
        return std::make_unique<WirelessChargingSubProblemStrategy_LShaped>();
    }

private:
    void InitRelaxedSubProblem(const ProblemData& problemData, GRBModel& subModel);
    Status SolveRelaxedModel(const ProblemData& problemData, const std::vector<double>& zValues,
        IntegerLShapedCutInfo& cutInfo, double& subObj);

    bool lowerBoundReady = false;
    double qLowerBound = 0.0;
    bool lowerBoundCutAdded = false;
    std::unique_ptr<GRBModel> relaxedModel;
    std::vector<GRBConstr> relaxedFixConstrs;
};
