#pragma once

#include "basic_solver.hpp"
#include "gurobi_c++.h"
#include "solver_config.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class IMILPAlgorithmStrategy {
public:
    virtual ~IMILPAlgorithmStrategy() = default;
    virtual Status Initialize() = 0;
    virtual Status Solve() = 0;
    void SetSolverConfig(const SolverConfig& config) { solverConfig = config; }

    bool initialized = false;

protected:
    void ApplySolverConfig(GRBModel& model) const {
        model.set(GRB_DoubleParam_MIPGap, solverConfig.mipGap);
        model.set(GRB_DoubleParam_TimeLimit, solverConfig.timeLimit);
        model.set(GRB_IntParam_Presolve, solverConfig.presolve);
        model.set(GRB_IntParam_MIPFocus, solverConfig.mipFocus);
        model.set(GRB_DoubleParam_Heuristics, solverConfig.heuristics);
        model.set(GRB_IntParam_Threads, solverConfig.threads);
        model.set(GRB_IntParam_Method, solverConfig.method);
    }

    SolverConfig solverConfig;
};

class MILPSolver {
public:
    void SetSolverConfig(const SolverConfig& config) {
        solverConfig = config;
        if (algorithm) {
            algorithm->SetSolverConfig(solverConfig);
        }
    }

    void SetAlgorithm(std::unique_ptr<IMILPAlgorithmStrategy> newAlgorithm) {
        algorithm = std::move(newAlgorithm);
        algorithm->SetSolverConfig(solverConfig);
    }

    Status Run() {
        if (!algorithm->initialized) {
            Status status = algorithm->Initialize();
            if (status) {
                return status;
            }
        }

        return algorithm->Solve();
    }

private:
    SolverConfig solverConfig;
    std::unique_ptr<IMILPAlgorithmStrategy> algorithm;
};
