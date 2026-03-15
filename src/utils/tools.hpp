#pragma once

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <istream>
#include <vector>

inline std::string PatternToString(const std::vector<int>& pattern) {
    std::string res = "_";
    for (size_t i = 0; i < pattern.size() - 1; ++i) {
        res += std::to_string(pattern[i]) + "_";
    }
    res += std::to_string(pattern[pattern.size() - 1]);
    return res;
}

namespace Tools {
    inline std::string Trim(const std::string& text) {
        size_t left = 0;
        while (left < text.size() && std::isspace(static_cast<unsigned char>(text[left]))) {
            ++left;
        }

        size_t right = text.size();
        while (right > left && std::isspace(static_cast<unsigned char>(text[right - 1]))) {
            --right;
        }

        return text.substr(left, right - left);
    }

    inline std::string ToLower(std::string text) {
        std::transform(
            text.begin(),
            text.end(),
            text.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return text;
    }

    inline std::string ToUpper(std::string text) {
        std::transform(
            text.begin(),
            text.end(),
            text.begin(),
            [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return text;
    }

    inline std::vector<std::string> SplitAndTrim(
        const std::string& text,
        char delimiter,
        bool skipEmpty = false) {
        std::vector<std::string> parts;
        std::istringstream stream(text);
        std::string token;
        while (std::getline(stream, token, delimiter)) {
            token = Trim(token);
            if (!skipEmpty || !token.empty()) {
                parts.push_back(token);
            }
        }
        return parts;
    }

    inline std::vector<int> ParseIntList(const std::string& text, char delimiter = ',') {
        std::vector<int> values;
        for (const auto& token : SplitAndTrim(text, delimiter, true)) {
            values.push_back(std::stoi(token));
        }
        return values;
    }

    inline std::vector<double> ParseDoubleList(const std::string& text, char delimiter = ',') {
        std::vector<double> values;
        for (const auto& token : SplitAndTrim(text, delimiter, true)) {
            values.push_back(std::stod(token));
        }
        return values;
    }

    inline bool ReadValidLine(std::istream& stream, std::string& line, char commentPrefix = '#') {
        while (std::getline(stream, line)) {
            line = Trim(line);
            if (!line.empty() && line[0] != commentPrefix) {
                return true;
            }
        }
        return false;
    }
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
