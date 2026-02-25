#pragma once

#include "using_solver.hpp"

class TestDataInitializationStrategy : public ISolverDataInitializationStrategy {
public:
    void DataInit(ProblemData& data) override;
};