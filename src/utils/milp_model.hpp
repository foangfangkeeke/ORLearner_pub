#pragma once

#include "gurobi_c++.h"
#include <string>
#include <vector>
#include <utility>

// ── Variable definition ──────────────────────────────────────────────────────
// type: GRB_CONTINUOUS ('C'), GRB_BINARY ('B'), GRB_INTEGER ('I')
struct VarDef {
    std::string name;
    double lb;
    double ub;
    char   type;
    double objCoeff;
};

// ── Constraint definition ────────────────────────────────────────────────────
// sense: GRB_LESS_EQUAL ('<'), GRB_GREATER_EQUAL ('>'), GRB_EQUAL ('=')
struct ConstrDef {
    std::string name;
    std::vector<std::pair<std::string, double>> terms;  // (varName, coefficient)
    char   sense;
    double rhs;
};

// ── Unified MILP model ───────────────────────────────────────────────────────
// A problem is fully described by its variables, constraints, and objective
// sense.  Any algorithm (DirectSolver, Benders, …) can consume this struct
// without knowing the problem domain.
struct MILPModel {
    std::vector<VarDef>   variables;
    std::vector<ConstrDef> constraints;
    int objSense;  // GRB_MINIMIZE or GRB_MAXIMIZE
};

// ── Problem interface ────────────────────────────────────────────────────────
// Each problem implements this interface once.  Algorithms receive the
// MILPModel returned by Formulate() and handle solving / decomposition
// themselves.
class IMILPFormulator {
public:
    // Build and return the complete MILP model for the problem.
    virtual MILPModel Formulate() = 0;

    // Pretty-print the optimal solution.
    // varValues[i] corresponds to MILPModel::variables[i].
    virtual void PrintSolution(const MILPModel& model,
                               const std::vector<double>& varValues) = 0;

    virtual ~IMILPFormulator() = default;
};
