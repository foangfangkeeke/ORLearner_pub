#pragma once

#include "milp_solver.hpp"
#include "benders_decomposition.hpp"

#include <memory>
#include <vector>
#include <string>

class ProblemData;
class GRBEnv;
class GRBModel;
class GRBVar;

using IntegerLShapedCutInfo = BendersCutInfo;
using IntegerLShapedSubProblemContext = BendersSubProblemContext;

class IDataInitializationStrategy_IntegerLShaped {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual std::vector<ProblemDataConstr> ConstrInit(ProblemData& data) = 0;
    virtual std::vector<double> BuildWarmStartMasterValues(const ProblemData& data) const = 0;
    virtual ~IDataInitializationStrategy_IntegerLShaped() = default;
};

class ISubProblemStrategy_IntegerLShaped {
public:
    virtual void InitSubProblem(
        const ProblemData& problemData, GRBModel& subModel, IntegerLShapedSubProblemContext& context) = 0;

    virtual void UpdateSubProblem(
        const ProblemData& problemData, GRBModel& subModel, IntegerLShapedSubProblemContext& context,
        const std::vector<double>& zValues) = 0;

    virtual Status SolveSubProblem(
        const ProblemData& problemData, GRBModel& subModel, IntegerLShapedSubProblemContext& context,
        const std::vector<double>& zValues, IntegerLShapedCutInfo& cutInfo, double& subObj) = 0;

    virtual Status SolveRelaxedSubProblem(
        const ProblemData& problemData, GRBModel& subModel, IntegerLShapedSubProblemContext& context,
        const std::vector<double>& zValues, IntegerLShapedCutInfo& cutInfo, double& subObj) = 0;

    virtual std::unique_ptr<ISubProblemStrategy_IntegerLShaped> Clone() const = 0;
    virtual ~ISubProblemStrategy_IntegerLShaped() = default;
};

class IntegerLShaped : public IMILPAlgorithmStrategy {
public:
    IntegerLShaped(ProblemType problemType, std::string dataFolder = "test_data", int maxIters = 100, double tol = 1e-6);
    Status Initialize() override;
    Status Solve() override;
    ~IntegerLShaped() override;

private:
    struct CutEval {
        GRBLinExpr expr = 0.0;
        double lhsAtCurrent = 0.0;
        std::string name;
    };

    CutEval BuildImprovedCut(const std::vector<double>& zValues, const std::vector<double>& zCurrent,
        double delta, double lowerBound, int iter) const;

    CutEval BuildPriorityCut(const std::vector<double>& zValues, const std::vector<double>& zCurrent,
        double delta, double lowerBound, int iter) const;

    CutEval BuildContinuousCut(const IntegerLShapedCutInfo& cutInfo, const std::vector<double>& zCurrent,
        int iter) const;

    void UpdateIncumbent(const std::vector<double>& zValues, double secondStageValue);

    void PrintBestSolution() const;

    ProblemType problemType;
    std::string dataFolder;
    int maxIters;
    double tolerance;

    std::unique_ptr<GRBEnv> env;
    std::shared_ptr<GRBModel> model;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy_IntegerLShaped> dataIniter;
    std::unique_ptr<ISubProblemStrategy_IntegerLShaped> subProblemStrategy;
    std::unique_ptr<GRBEnv> subEnv;
    std::unique_ptr<GRBModel> subModel;
    IntegerLShapedSubProblemContext subContext;
    std::vector<GRBVar> zVars;
    GRBVar theta;

    double globalLowerBound;
    double bestUpperBound;
    double bestLowerBound;
    std::vector<double> incumbentZValues;
    double incumbentSecondStageValue;
};