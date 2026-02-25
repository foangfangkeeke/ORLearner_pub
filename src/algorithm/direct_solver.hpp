#pragma once

#include "basic_solver.hpp"
#include "column_generation.hpp"

#include <memory>
#include <iostream>

class IDirectSolverStrategy {
public:
    virtual void BuildModel(GRBSolver& solver, const ProblemData& data) = 0;
    virtual void PrintSolution(GRBSolver& solver, const ProblemData& data) = 0;
    virtual ~IDirectSolverStrategy() = default;
};

class DirectSolver {
public:
    DirectSolver(ProblemType problemType);
    Status Run();
    ~DirectSolver();

private:
    ProblemType problemType;
    GRBEnv env;
    std::unique_ptr<GRBSolver> solver;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy> dataIniter;
    std::unique_ptr<IDirectSolverStrategy> strategy;

    Status Init();
    Status Solve();
};
