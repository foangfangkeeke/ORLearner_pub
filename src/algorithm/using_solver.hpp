#pragma once

#include "basic_solver.hpp"
#include "milp_solver.hpp"

#include <memory>
#include <string>

class IDataInitializationStrategy_Solver {
public:
    virtual Status DataInit(GRBModel& model) = 0;
    virtual ~IDataInitializationStrategy_Solver() = default;
};

class UsingSolver final : public IMILPAlgorithmStrategy {
public:
    UsingSolver(ProblemType problemType, std::string dataFolder = "test_data");
    Status Initialize();
    Status Solve();
    ~UsingSolver();

private:
    ProblemType problemType;
    std::string dataFolder;

    std::unique_ptr<GRBEnv> env;
    std::unique_ptr<GRBModel> model;
    std::unique_ptr<IDataInitializationStrategy_Solver> dataIniter;
};
