#pragma once

#include "basic_solver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class IMILPAlgorithmStrategy {
public:
    virtual ~IMILPAlgorithmStrategy() = default;
    virtual Status Run() = 0;
};

class MILPSolver {
public:
    void SetAlgorithm(std::unique_ptr<IMILPAlgorithmStrategy> newAlgorithm) {
        algorithm = std::move(newAlgorithm);
    }

    Status Run() {
        return algorithm->Run();
    }

private:
    std::unique_ptr<IMILPAlgorithmStrategy> algorithm;
};
