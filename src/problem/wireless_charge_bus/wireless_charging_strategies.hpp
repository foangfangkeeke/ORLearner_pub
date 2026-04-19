#include <string>
#include "using_solver.hpp"

class WirelessChargingDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
	WirelessChargingDataInitializationStrategy_Solver(std::string dataFolder);
	void DataInit(ProblemData& data) override;

private:
	std::string dataFolder;
};