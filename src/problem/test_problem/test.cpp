#include <iostream>
#include "gurobi_c++.h"

using namespace std;

bool SolveWithGurobiTest() {
    try {
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "gurobi_log.txt");
        env.start();

        GRBModel model = GRBModel(env);
        model.set(GRB_StringAttr_ModelName, "TestModel");

        GRBVar x1 = model.addVar(0.0, 10.0, 1.0, GRB_INTEGER, "x1");
        GRBVar x2 = model.addVar(0.0, GRB_INFINITY, 2.0, GRB_CONTINUOUS, "x2");

        model.setObjective(x1 + 2 * x2, GRB_MINIMIZE);

        model.addConstr(x1 + x2 >= 5.0, "Constraint1");

        model.optimize();

        int solveStatus = model.get(GRB_IntAttr_Status);
        if (solveStatus == GRB_OPTIMAL) {
            cout << "================ 求解成功 ================" << endl;
            cout << "x1 = " << x1.get(GRB_DoubleAttr_X) << endl;  // 获取变量x1的最优值
            cout << "x2 = " << x2.get(GRB_DoubleAttr_X) << endl;  // 获取变量x2的最优值
            cout << "最优目标函数值 = " << model.get(GRB_DoubleAttr_ObjVal) << endl;
            return true;
        } else {
            cout << "求解失败，状态码：" << solveStatus << endl;
            return false;
        }

    } catch (GRBException& e) {  // 捕获Gurobi专属异常（如许可证、模型错误）
        cout << "Gurobi异常：错误码 " << e.getErrorCode() << "，信息：" << e.getMessage() << endl;
        return false;
    } catch (exception& e) {  // 捕获其他标准异常（如内存问题）
        cout << "其他异常：" << e.what() << endl;
        return false;
    }
}