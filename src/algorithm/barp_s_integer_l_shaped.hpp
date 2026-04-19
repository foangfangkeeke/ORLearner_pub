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

class BARPSIntegerLShaped : public IMILPAlgorithmStrategy {
public:
    BARPSIntegerLShaped(ProblemType problemType, std::string dataFolder = "test_data", int maxIters = 100, double tol = 1e-6);
    Status Initialize() override;
    Status Solve() override;
    ~BARPSIntegerLShaped() override;

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

    CutEval BuildContinuousCut(const BendersCutInfo& cutInfo, const std::vector<double>& zCurrent, int iter) const;

    void UpdateIncumbent(const std::vector<double>& zValues, double secondStageValue);

    void PrintBestSolution() const;

    ProblemType problemType;
    std::string dataFolder;
    int maxIters;
    double tolerance;

    std::unique_ptr<GRBEnv> env;
    std::shared_ptr<GRBModel> model;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy_Benders> dataIniter;
    std::unique_ptr<ISubProblemStrategy_Benders> subProblemStrategy;
    std::unique_ptr<GRBEnv> subEnv;
    std::unique_ptr<GRBModel> subModel;
    BendersSubProblemContext subContext;
    std::vector<GRBVar> zVars;
    GRBVar theta;

    double globalLowerBound;
    double bestUpperBound;
    double bestLowerBound;
    std::vector<double> incumbentZValues;
    double incumbentSecondStageValue;
};