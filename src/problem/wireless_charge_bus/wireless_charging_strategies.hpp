#include <string>
#include "using_solver.hpp"
#include "benders_decomposition.hpp"

class WirelessChargingDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
	WirelessChargingDataInitializationStrategy_Solver(std::string dataFolder);
	void DataInit(ProblemData& data) override;

private:
	std::string dataFolder;
};

class WirelessChargingDataInitializationStrategy_Benders : public IDataInitializationStrategy_Benders {
public:
	explicit WirelessChargingDataInitializationStrategy_Benders(std::string dataFolder = "test_data");
	void DataInit(ProblemData& data) override;
	std::vector<ProblemDataConstr> ConstrInit(ProblemData& data) override;

private:
	std::string dataFolder;
};

class WirelessChargingSubProblemStrategy_Benders : public ISubProblemStrategy_Benders {
public:
	void InitSubProblem(const ProblemData& problemData, GRBModel& subModel,
        BendersSubProblemContext& context) override;

	void UpdateSubProblem(const ProblemData& problemData, GRBModel& subModel,
        BendersSubProblemContext& context, const std::vector<double>& yValues) override;

	Status SolveSubProblem(const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context,
        const std::vector<double>& yValues, BendersCutInfo& cutInfo, double& subObj) override;

	std::unique_ptr<ISubProblemStrategy_Benders> Clone() const override
	{
		return std::make_unique<WirelessChargingSubProblemStrategy_Benders>(*this);
	}

private:
	bool lowerBoundReady = false;
	double qLowerBound = 0.0;
	bool lowerBoundCutAdded = false;
};