#include "direct_solver.hpp"
#include "cutting_stock_problem.hpp"

#include <iostream>
#include <map>
#include <functional>

static const std::map<ProblemType,
                      std::tuple<std::function<std::unique_ptr<IDataInitializationStrategy>()>,
                                 std::function<std::unique_ptr<IDirectSolverStrategy>()>>> directStrategyMap = {
    {
        CUTTINGSTOCK,
        std::make_tuple(
            [](){return std::make_unique<CuttingStockDataInitializationStrategy>();},
            [](){return std::make_unique<CuttingStockDirectSolverStrategy>();})
    }
};

DirectSolver::DirectSolver(ProblemType problemType) : problemType(problemType)
{
    std::cout << "===== build DirectSolver =====" << std::endl;
}

DirectSolver::~DirectSolver()
{
    std::cout << "===== destroy DirectSolver =====" << std::endl;
}

Status DirectSolver::Init()
{
    std::cout << "===== init DirectSolver =====" << std::endl;
    std::cout << "problemType=" << problemType << std::endl;

    problemData = std::make_unique<ProblemData>();
    solver = std::make_unique<GRBSolver>(env);

    auto it = directStrategyMap.find(problemType);
    if (it == directStrategyMap.end()) {
        throw std::invalid_argument("Unsupported problem type for DirectSolver");
    }
    const auto& strategies = it->second;

    dataIniter = std::get<0>(strategies)();
    strategy = std::get<1>(strategies)();

    dataIniter->DataInit(*problemData);

    return OK;
}

Status DirectSolver::Solve()
{
    std::cout << "===== start DirectSolver =====" << std::endl;

    strategy->BuildModel(*solver, *problemData);
    solver->Update();

    bool status = solver->Solve();
    if (!status) {
        std::cerr << "DirectSolver: no optimal solution found." << std::endl;
        return ERROR;
    }

    std::cout << "===== DirectSolver optimal solution =====" << std::endl;
    std::cout << "Objective value: " << solver->GetObjectiveValue() << std::endl;
    strategy->PrintSolution(*solver, *problemData);

    return OK;
}

Status DirectSolver::Run()
{
    Status status = Init();
    if (status != OK) {
        return status;
    }
    return Solve();
}
