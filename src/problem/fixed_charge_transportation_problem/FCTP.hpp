#pragma once

#include "benders_decomposition.hpp"
#include "using_solver.hpp"

class FCTPDataInitializationStrategy_Benders : public IDataInitializationStrategy_Benders {
public:
    void DataInit(ProblemData& problemData) override;
    std::vector<ProblemDataConstr> ConstrInit(ProblemData& problemData) override;
};

class FCTPSubProblemStrategy_Benders : public ISubProblemStrategy_Benders {
public:
    void InitSubProblem(
        const ProblemData& problemData,
        GRBModel& subModel,
        BendersSubProblemContext& context) override;

    void UpdateSubProblem(
        const ProblemData& problemData,
        GRBModel& subModel,
        BendersSubProblemContext& context,
        const std::vector<double>& yValues) override;

    Status SolveSubProblem(
        const ProblemData& problemData,
        GRBModel& subModel,
        BendersSubProblemContext& context,
        const std::vector<double>& yValues,
        BendersCutInfo& cutInfo,
        double& subObj) override;

    std::unique_ptr<ISubProblemStrategy_Benders> Clone() const override {
        return std::make_unique<FCTPSubProblemStrategy_Benders>(*this);
    }
};

class FCTPDataInitializationStrategy_Solver : public IDataInitializationStrategy_Solver {
public:
    void DataInit(ProblemData& problemData) override;
};