#pragma once

#include <string>
#include <vector>
#include <optional>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <type_traits>
#include <stdexcept>

typedef std::unordered_map<int, std::unordered_map<int, double>> linkTimeType;
typedef std::vector<std::vector<int>> stationTimeType;
typedef std::vector<std::vector<int>> stationType;
typedef std::vector<std::vector<std::string>> stringCfgType;

template <typename T>
std::optional<T> ParseFile(const std::string& file_path, char sep = ',', int startLine = 1);