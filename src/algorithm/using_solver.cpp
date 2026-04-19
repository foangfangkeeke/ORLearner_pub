#include "cutting_stock_problem.hpp"
#include "FCTP.hpp"
#include "BARP_S.hpp"
#include "wireless_charging_strategies.hpp"
#include "using_solver.hpp"
#include "test.hpp"
#include "basic_solver.hpp"

#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <map>

static const std::map<
    ProblemType,
    std::function<std::unique_ptr<IDataInitializationStrategy_Solver>(const std::string&)>> dataInitMap = {
    {TEST, [](const std::string&) {
        return std::make_unique<TestDataInitializationStrategy_Solver>(); }},
    {CUTTINGSTOCK, [](const std::string&) {
        return std::make_unique<CuttingStockDataInitializationStrategy_Solver>(); }},
    {FCTP, [](const std::string&) {
        return std::make_unique<FCTPDataInitializationStrategy_Solver>(); }},
    {BARP_S, [](const std::string&) {
        return std::make_unique<BRSDataInitializationStrategy_Solver>(); }},
    {WIRELESS_CHARGING, [](const std::string& dataFolder){
        return std::make_unique<WirelessChargingDataInitializationStrategy_Solver>(dataFolder);
    }}
};

Status UsingSolver::Initialize()
{
    problemData = std::make_unique<ProblemData>();

    auto it = dataInitMap.find(problemType);
    if (it == dataInitMap.end()) {
        throw std::invalid_argument("Unsupported problem type");
    }
    dataIniter = it->second(dataFolder);

    dataIniter->DataInit(*problemData);

    env = std::make_unique<GRBEnv>(true);
    env->set("LogFile", "gurobi_log.txt");
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    model = std::make_unique<GRBModel>(*env);

    auto vars = problemData->getData<std::vector<ProblemDataVar>>("vars");
    auto constr = problemData->getData<std::vector<ProblemDataConstr>>("constrs");
    auto objSense = problemData->getData<int>("obj");

    std::vector<GRBVar> varList;
    GRBLinExpr obj = 0;

    for (const auto& var : vars) {
        varList.push_back(model->addVar(var.lb, var.ub, var.obj, var.type, var.name));
        obj += var.obj * varList.back();
    }
    for (const auto& constr : constr) {
        GRBLinExpr lhs = 0;
        for (size_t i = 0; i < varList.size(); ++i) {
            lhs += varList[i] * constr.coeffs[i];
        }
        model->addConstr(lhs, constr.sense, constr.rhs, constr.name);
    }

    model->update();
    model->setObjective(obj, objSense);
    model->update();
    initialized = true;

    return OK;
}

Status UsingSolver::Solve()
{
    model->optimize();
    int solveStatus = model->get(GRB_IntAttr_Status);
    if (solveStatus == GRB_OPTIMAL || (solveStatus == GRB_TIME_LIMIT && model->get(GRB_IntAttr_SolCount)>0)) {
        std::cout << "Current objective: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
    } else {
        std::cerr << "Not solved to optimality, status: " << solveStatus << std::endl;
        return ERROR;
    }

    return OK;
}

UsingSolver::UsingSolver(ProblemType problemType, std::string dataFolder)
    : problemType(problemType), dataFolder(std::move(dataFolder)) {
    if (this->dataFolder.empty()) {
        this->dataFolder = "test_data";
    }
};

UsingSolver::~UsingSolver() {};