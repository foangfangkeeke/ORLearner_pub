#pragma once

#include <string>
#include <vector>

inline std::string PatternToString(const std::vector<int>& pattern) {
    std::string res = "_";
    for (size_t i = 0; i < pattern.size() - 1; ++i) {
        res += std::to_string(pattern[i]) + "_";
    }
    res += std::to_string(pattern[pattern.size() - 1]);
    return res;
}

namespace Debug {
    inline void OutputModel(const std::shared_ptr<GRBModel>& model) {
        try {
            model->write("debug_model.lp");
            std::cerr << "Model written to debug_model.lp" << std::endl;
        } catch (GRBException& e) {
            std::cerr << "Failed to write model: " << e.getMessage() << std::endl;
        }
    }

    inline void OutputResult(const std::shared_ptr<GRBModel>& model) {
        size_t numVars = model->get(GRB_IntAttr_NumVars);
        auto vars = model->getVars();
        for (size_t i = 0; i < numVars; ++i) {
            GRBVar var = vars[i];
            std::cout << var.get(GRB_StringAttr_VarName) << ": " << var.get(GRB_DoubleAttr_X) << std::endl;
        }
    }
}
