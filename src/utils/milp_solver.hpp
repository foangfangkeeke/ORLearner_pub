#pragma once

#include "basic_solver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class IMILPAlgorithmStrategy {
public:
    virtual ~IMILPAlgorithmStrategy() = default;
    virtual Status Initialize() = 0;
    virtual Status Solve() = 0;

    bool initialized = false;
};

class MILPSolver {
public:
    void SetAlgorithm(std::unique_ptr<IMILPAlgorithmStrategy> newAlgorithm) {
        algorithm = std::move(newAlgorithm);
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
    std::unique_ptr<IMILPAlgorithmStrategy> algorithm;
};
