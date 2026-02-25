#pragma once

#include "basic_solver.hpp"
#include "milp_model.hpp"

#include <memory>
#include <iostream>

// DirectSolver: solves any problem whose MILP formulation is provided via
// IMILPFormulator.  It is fully generic — no problem-specific logic lives here.
class DirectSolver {
public:
    DirectSolver(ProblemType problemType);
    Status Run();
    ~DirectSolver();

private:
    ProblemType problemType;
    GRBEnv env;
    std::unique_ptr<IMILPFormulator> formulator;

    Status Init();
    Status Solve(const MILPModel& model);
};
