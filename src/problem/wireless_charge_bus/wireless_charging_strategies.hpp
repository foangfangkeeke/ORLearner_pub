#include <string>
#include "using_solver.hpp"
#include "benders_decomposition.hpp"
#include "barp_s_integer_l_shaped.hpp"

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
        auto clone = std::make_unique<WirelessChargingSubProblemStrategy_LShaped>();
        clone->bendersImpl = bendersImpl;
        return clone;
    }

private:
    void InitRelaxedSubProblem(const ProblemData& problemData, GRBModel& subModel);
    Status SolveRelaxedModel(const ProblemData& problemData, const std::vector<double>& zValues,
        IntegerLShapedCutInfo& cutInfo, double& subObj);

    WirelessChargingSubProblemStrategy_Benders bendersImpl;
    std::unique_ptr<GRBModel> relaxedModel;
    std::vector<GRBVar> relaxedMasterVars;
    std::vector<GRBConstr> relaxedFixConstrs;
};
