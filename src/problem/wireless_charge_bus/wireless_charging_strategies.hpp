#include <string>
#include "using_solver.hpp"

class WirelessChargingDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
	WirelessChargingDataInitializationStrategy_Solver(std::string dataFolder);
	void DataInit(ProblemData& data) override;
	bool UseCustomSolver() const override { return true; }
	Status SolveWithStandardInterface(const ProblemData& data) override;

private:
	std::string dataFolder;
};

bool SolveWithGurobiWirelessChargingStratgies(const std::string& modelType, const std::string& dataFolder);