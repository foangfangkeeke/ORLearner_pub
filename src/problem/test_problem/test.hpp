#pragma once

#include "using_solver.hpp"

class TestDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    void DataInit(ProblemData& problemData) override;
};