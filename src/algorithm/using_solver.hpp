#pragma once

#include "basic_solver.hpp"
#include "milp_solver.hpp"

#include <vector>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <iostream>

class ISolverDataInitializationStrategy {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual ~ISolverDataInitializationStrategy() = default;
};

class UsingSolver final : public IMILPAlgorithmStrategy {
public:
    UsingSolver(ProblemType problemType);
    Status Initialize();
    Status Run() override;
    ~UsingSolver();

private:
    ProblemType problemType;

    std::unique_ptr<GRBEnv> env;
    std::unique_ptr<GRBModel> model;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<ISolverDataInitializationStrategy> dataIniter;
    bool initialized = false;

    GRBLinExpr obj = 0;

    Status Solve();
};
