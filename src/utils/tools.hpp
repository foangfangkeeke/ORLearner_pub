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