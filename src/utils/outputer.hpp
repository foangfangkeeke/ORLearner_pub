#pragma once

#include <filesystem>
#include <fstream>
#include "useless.hpp" 

struct CoutRedirectGuard {
    std::streambuf* oldBuf{nullptr};
    std::ofstream file;

    explicit CoutRedirectGuard(const std::filesystem::path& path) {
        file.open(path);
        if (file.is_open()) {
            oldBuf = std::cout.rdbuf(file.rdbuf());
        }
    }

    ~CoutRedirectGuard() {
        if (oldBuf) {
            std::cout.rdbuf(oldBuf);
        }
    }
};

namespace print_utils {

template <typename T>
inline void printt(const T& t) {
    std::cout << t;
}

template <typename T>
inline void printt(const std::vector<T>& vec) {
    std::cout << "[";
    bool is_first = true;
    for (const auto& elem : vec) {
        if (!is_first) {
            std::cout << ", ";
        }
        printt(elem);
        is_first = false;
    }
    std::cout << "]";
}

template <typename K, typename V>
inline void printt(const std::unordered_map<K, V>& umap) {
    std::cout << "{";
    bool is_first = true;
    for (const auto& pair : umap) {
        if (!is_first) {
            std::cout << ", ";
        }
        printt(pair.first);
        std::cout << ":";
        printt(pair.second);
        is_first = false;
    }
    std::cout << "}";
}

template <typename T>
inline void print(const T& vec, const std::string& variableName = "") {
    std::cout << "================" + variableName + "================" << std::endl;
    printt(vec);
    std::cout << std::endl;
}

} // namespace print_utils