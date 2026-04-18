#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <type_traits>
#include <stdexcept>

template <typename T> struct is_vector : false_type {};
template <typename T> struct is_vector<std::vector<T>> : true_type {};
template <typename T> constexpr bool is_vector_v = is_vector<T>::value;

template <typename T> struct is_unordered_map : false_type {};
template <typename K, typename V> struct is_unordered_map<std::unordered_map<K, V>> : true_type {};
template <typename T> constexpr bool is_unordered_map_v = is_unordered_map<T>::value;
template <typename T>
struct is_nested_map : std::false_type {};

template <typename K1, typename K2, typename V>
struct is_nested_map<std::unordered_map<K1, std::unordered_map<K2, V>>> : std::true_type {};

template <typename T>
constexpr bool is_nested_map_v = is_nested_map<T>::value;

template <typename T>
constexpr bool is_single_level_map_v = is_unordered_map_v<T> && !is_nested_map_v<T>;

inline std::string get_current_file_directory(const std::string& file_path) {
    size_t sep_pos = file_path.find_last_of("/\\");
    if (sep_pos == std::string::npos) {
        return ".";
    }
    return file_path.substr(0, sep_pos + 1);
}

inline std::string get_absolute_path(const std::string& file_path, const std::string& related_file_name) {
    std::string current_dir = get_current_file_directory(file_path);
    return current_dir + related_file_name;
}