#pragma once

#include "gurobi_c++.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <stdexcept>

typedef std::tuple<std::string, double, char> Constraint;

enum ProblemType {
    CUTTINGSTOCK,
    DEFAULT = 0x100,
};

// 与Gurobi状态对齐，避免冲突
enum Status {
    OK =        0,
    ERROR =     0x100,
};

// 基类：用于类型擦除
class DataBase {
public:
    virtual ~DataBase() = default;
};

// 派生类：存储具体类型的数据
template <typename T>
class DataWrapper : public DataBase {
public:
    explicit DataWrapper(const T& data) : data_(data) {}
    const T& getData() const { return data_; }
private:
    T data_;
};

// 数据容器：不依赖std::any，兼容C++11及以上
class ProblemData {
public:
    // 添加数据（支持任意类型）
    template <typename T>
    void addData(const std::string& key, const T& value) {
        dataMap_[key] = std::make_unique<DataWrapper<T>>(value);
    }

    // 获取数据（自动检查类型）
    template <typename T>
    const T& getData(const std::string& key) const {
        auto it = dataMap_.find(key);
        if (it == dataMap_.end()) {
            throw std::runtime_error("Data key not found: " + key);
        }

        // 尝试转换为具体类型的包装器
        DataWrapper<T>* wrapper = dynamic_cast<DataWrapper<T>*>(it->second.get());
        if (wrapper == nullptr) {
            throw std::runtime_error("Data type mismatch for key: " + key);
        }

        return wrapper->getData();
    }

private:
    // 用键值对存储数据，值为基类指针（实现类型擦除）
    std::unordered_map<std::string, std::unique_ptr<DataBase>> dataMap_;
};

class GRBSolver {
public:
    GRBSolver(GRBEnv& env) : model(env) {}

    virtual ~GRBSolver() = default;

    virtual bool Solve() {
        try {
            model.optimize();
            return model.get(GRB_IntAttr_Status) == GRB_OPTIMAL;
        } catch (GRBException& e) {
            std::cerr << "Gurobi error code = " << e.getErrorCode() << std::endl;
            std::cerr << "Error message: " << e.getMessage() << std::endl;
            return false;
        } catch (std::exception& e) {  // 补充通用异常捕获
            std::cerr << "Unexpected error: " << e.what() << std::endl;
            return false;
        }
    }

    // 添加变量：接口不变，内部依赖model.addVar兼容
    const GRBVar AddVariable(double lb, double ub, double obj, char type, const std::string& name = "") {
        return model.addVar(lb, ub, obj, type, name);
    }

    int GetConstraintsNum() {
        return model.get(GRB_IntAttr_NumConstrs);
    }

    // 添加模式约束：修正约束数量属性的引用
    void AddPattern(std::vector<int> pattern, GRBVar var) {
        GRBConstr* constrs = model.getConstrs();
        int numConstrs = GetConstraintsNum();
        
        if (pattern.size() != static_cast<size_t>(numConstrs)) {
            throw GRBException("Pattern size does not match number of constraints", ERROR);
        }

        for (int i = 0; i < numConstrs; ++i) {
            model.chgCoeff(constrs[i], var, pattern[i]);
        }
    }

    // 添加约束：接口不变，依赖model.addConstr兼容
    GRBConstr AddConstraint(const GRBLinExpr &lhsExpr, char sense, double rhs, const std::string& name = "") {
        return model.addConstr(lhsExpr, sense, rhs, name);
    }

    // 获取约束：接口不变
    GRBConstr GetConstraintByName(const std::string& name) {
        return model.getConstrByName(name);
    }

    void SetObjective(const GRBLinExpr& expr, int sense = GRB_MINIMIZE) {
        model.setObjective(expr, sense);
    }

    // 获取目标值：修正目标值属性的引用
    double GetObjectiveValue() const {
        // 目标值属性为GRB_DoubleAttr_ObjVal（前缀枚举）
        return model.get(GRB_DoubleAttr_ObjVal);
    }

    double GetDualByName(const std::string& name) {
        try {
            GRBConstr constr = model.getConstrByName(name);
        } catch (const GRBException& e) {
            if (e.getErrorCode() == 1001) {
                throw std::runtime_error("Constraint not found: " + name);
            } else {
                throw;
            }
        }
    }

    double GetDual(int idx) {
        GRBConstr* constrs = model.getConstrs();
        return constrs[idx].get(GRB_DoubleAttr_Pi);
    }

    void Update() {
        model.update();
    }

    void printObjective() {  // model 是 const 引用
        int numVars = model.get(GRB_IntAttr_NumVars);
        std::cout << "===" << std::endl;
        const GRBVar* vars = model.getVars();  // 用 const GRBVar* 接收
        std::cout << "numVars:" << numVars << std::endl;
        
        for (int i = 0; i < numVars; ++i) {
            const GRBVar& var = vars[i];  // 变量也为 const 引用（只读）
            std::cout << "Variable: " << var.get(GRB_StringAttr_VarName)
                    << ", objCoef: " << var.get(GRB_DoubleAttr_Obj) << std::endl;
        }
        std::cout << "===" << std::endl;
    }

protected:
    GRBModel model;  // 直接使用GRBModel对象（无需额外构造）
};