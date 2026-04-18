#pragma once

#include "basic_solver.hpp"
#include "milp_solver.hpp"

#include <vector>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <iostream>

class IDataInitializationStrategy_Solver {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual bool UseCustomSolver() const { return false; }
    virtual Status SolveWithStandardInterface(const ProblemData& data) {
        (void)data;
        return ERROR;
    }
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
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy_Solver> dataIniter;
    bool initialized = false;

    GRBLinExpr obj = 0;

};
