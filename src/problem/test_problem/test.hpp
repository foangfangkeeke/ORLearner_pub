#pragma once

#include "using_solver.hpp"

class TestDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    explicit TestDataInitializationStrategy_Solver(std::string dataFolder = "test_data");
    void DataInit(ProblemData& problemData) override;

private:
    std::string dataFolder;
};