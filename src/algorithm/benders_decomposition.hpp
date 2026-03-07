#pragma once

#include "basic_solver.hpp"
#include "milp_solver.hpp"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

struct BendersCutInfo {
    std::vector<double> yCoeffs;
    double constant = 0.0;
    double rhs = 0.0;
    bool isOptimalityCut = true;
    char sense = '>';
};

struct BendersSubProblemContext {
    std::map<std::string, std::vector<GRBVar>> varGroups;
    std::map<std::string, std::vector<GRBConstr>> constrGroups;

    std::vector<GRBVar>& EnsureVarGroup(const std::string& groupName)
    {
        return varGroups[groupName];
    }

    std::vector<GRBConstr>& EnsureConstrGroup(const std::string& groupName)
    {
        return constrGroups[groupName];
    }

    const std::vector<GRBVar>& RequireVarGroup(const std::string& groupName, size_t expectedSize) const
    {
        auto it = varGroups.find(groupName);
        if (it == varGroups.end()) {
            throw std::runtime_error("Missing Benders sub-problem variable group: " + groupName);
        }
        if (it->second.size() != expectedSize) {
            throw std::runtime_error("Unexpected variable group size for: " + groupName);
        }
        return it->second;
    }

    const std::vector<GRBConstr>& RequireConstrGroup(const std::string& groupName, size_t expectedSize) const
    {
        auto it = constrGroups.find(groupName);
        if (it == constrGroups.end()) {
            throw std::runtime_error("Missing Benders sub-problem constraint group: " + groupName);
        }
        if (it->second.size() != expectedSize) {
            throw std::runtime_error("Unexpected constraint group size for: " + groupName);
        }
        return it->second;
    }

    void Clear()
    {
        varGroups.clear();
        constrGroups.clear();
    }
};

class IDataInitializationStrategy_Benders {
public:
    virtual void DataInit(ProblemData& data) = 0;
    virtual std::vector<ProblemDataConstr> ConstrInit(ProblemData& data) = 0;
    virtual ~IDataInitializationStrategy_Benders() = default;
};

class ISubProblemStrategy_Benders {
public:
    virtual void InitSubProblem(
        const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context) = 0;

    virtual void UpdateSubProblem(
        const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context,
        const std::vector<double>& yValues) = 0;

    virtual Status SolveSubProblem(
        const ProblemData& problemData, GRBModel& subModel, BendersSubProblemContext& context,
        const std::vector<double>& yValues, BendersCutInfo& cutInfo, double& subObj) = 0;

    virtual std::unique_ptr<ISubProblemStrategy_Benders> Clone() const = 0;
    virtual ~ISubProblemStrategy_Benders() = default;
};

class BendersSubSolver {
public:
    explicit BendersSubSolver(std::unique_ptr<ISubProblemStrategy_Benders> strategy);
    void Init(const ProblemData& problemData);
    Status Solve(const ProblemData& problemData, const std::vector<double>& yValues, BendersCutInfo& cutInfo, double& subObj);
    ~BendersSubSolver();

private:
    std::unique_ptr<ISubProblemStrategy_Benders> strategy;
    std::unique_ptr<GRBEnv> env;
    std::unique_ptr<GRBModel> model;
    BendersSubProblemContext context;
};

class BendersDecomposition : public IMILPAlgorithmStrategy {
public:
    BendersDecomposition(ProblemType problemType, int maxIters = 100, double tol = 1e-6);
    Status Initialize() override;
    Status Solve() override;
    ~BendersDecomposition();

private:
    ProblemType problemType;
    int maxIters;
    double tolerance;

    std::unique_ptr<GRBEnv> env;
    std::shared_ptr<GRBModel> model;
    std::unique_ptr<BendersSubSolver> sub;
    std::unique_ptr<ProblemData> problemData;
    std::unique_ptr<IDataInitializationStrategy_Benders> dataIniter;

    std::vector<GRBVar> yVars;
    GRBVar theta;

    double bestUpperBound;
    double bestLowerBound;
};
