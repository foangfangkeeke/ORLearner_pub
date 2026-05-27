#include "cutting_stock_problem.hpp"
#include "FCTP.hpp"
#include "BARP_S.hpp"
#include "wireless_charging_strategies.hpp"
#include "using_solver.hpp"
#include "test.hpp"
#include "tools.hpp"

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

static const std::map<
    ProblemType,
    std::function<std::unique_ptr<IDataInitializationStrategy_Solver>(const std::string&)>> dataInitMap = {
    {TEST, [](const std::string& dataFolder) {
        return std::make_unique<TestDataInitializationStrategy_Solver>(dataFolder); }},
    {CUTTINGSTOCK, [](const std::string& dataFolder) {
        return std::make_unique<CuttingStockDataInitializationStrategy_Solver>(dataFolder); }},
    {FCTP, [](const std::string& dataFolder) {
        return std::make_unique<FCTPDataInitializationStrategy_Solver>(dataFolder); }},
    {BARP_S, [](const std::string& dataFolder) {
        return std::make_unique<BRSDataInitializationStrategy_Solver>(dataFolder); }},
    {WIRELESS_CHARGING, [](const std::string& dataFolder){
        return std::make_unique<WirelessChargingDataInitializationStrategy_Solver>(dataFolder);
    }}
};

Status UsingSolver::Initialize()
{
    auto it = dataInitMap.find(problemType);
    if (it == dataInitMap.end()) {
        throw std::invalid_argument("Unsupported problem type");
    }
    dataIniter = it->second(dataFolder);

    env = std::make_unique<GRBEnv>(true);
    env->set("LogFile", "gurobi_log.txt");
    env->start();
    model = std::make_unique<GRBModel>(*env);
    model->set(GRB_IntParam_OutputFlag, 1);

    Status initStatus = dataIniter->DataInit(*model);
    if (initStatus != OK) {
        return initStatus;
    }
    ApplySolverConfig(*model);
    model->update();
    initialized = true;
    return OK;
}

Status UsingSolver::Solve()
{
    const auto solveStart = Tools::Clock::now();
    model->optimize();

    int solveStatus = model->get(GRB_IntAttr_Status);
    if (solveStatus == GRB_OPTIMAL || (solveStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount) > 0)) {
        std::cout << "Current objective: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
        const auto solveEnd = Tools::Clock::now();
        std::cout << "UsingSolver final solve time: " << Tools::ElapsedMs(solveStart, solveEnd) << " ms" << std::endl;
        return OK;
    }

    std::cerr << "Not solved to optimality, status: " << solveStatus << std::endl;
    return ERROR;
}

UsingSolver::UsingSolver(ProblemType problemType, std::string dataFolder)
    : problemType(problemType), dataFolder(dataFolder) {};

UsingSolver::~UsingSolver() {};
