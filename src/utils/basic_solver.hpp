#pragma once

#include "gurobi_c++.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <stdexcept>

typedef std::tuple<std::string, double, char> Constraint;

enum ProblemType {
    TEST,
    CUTTINGSTOCK,
    DEFAULT = 0x100,
};

enum Status {
    OK =        0,
    ERROR =     0x100,
};

struct ProblemDataVar
{
    double lb;
    double ub;
    double obj;
    char type;
    std::string name;
};

struct ProblemDataConstr
{
    std::vector<double> coeffs;
    char sense;
    double rhs;
    std::string name;
};

class DataBase {
public:
    virtual ~DataBase() = default;
};

template <typename T>
class DataWrapper : public DataBase {
public:
    explicit DataWrapper(const T& data) : data_(data) {}
    const T& getData() const { return data_; }
private:
    T data_;
};

class ProblemData {
public:
    template <typename T>
    void addData(const std::string& key, const T& value) {
        dataMap_[key] = std::make_unique<DataWrapper<T>>(value);
    }

    template <typename T>
    const T& getData(const std::string& key) const {
        auto it = dataMap_.find(key);
        if (it == dataMap_.end()) {
            throw std::runtime_error("Data key not found: " + key);
        }

        DataWrapper<T>* wrapper = dynamic_cast<DataWrapper<T>*>(it->second.get());
        if (wrapper == nullptr) {
            throw std::runtime_error("Data type mismatch for key: " + key);
        }

        return wrapper->getData();
    }

private:
    std::unordered_map<std::string, std::unique_ptr<DataBase>> dataMap_;
};