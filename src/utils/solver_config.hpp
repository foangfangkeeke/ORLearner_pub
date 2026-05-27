#pragma once

struct SolverConfig {
    double mipGap = 0.0;
    double timeLimit = 18000.0;
    int presolve = -1;
    int mipFocus = 0;
    double heuristics = 0.05;
    int threads = 14;
    int method = -1;
};

SolverConfig LoadSolverConfig();