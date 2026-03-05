#pragma once

#include "using_solver.hpp"

class FCTPDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    void DataInit(ProblemData& problemData) override;
};