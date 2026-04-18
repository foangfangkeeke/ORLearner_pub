#include "data_reader.hpp"
#include "useless.hpp"

using namespace std;

namespace {
    template <typename T>
    T StrToT(const std::string& s) {
        T val;
        std::istringstream iss(s);
        if (!(iss >> val)) {
            throw std::invalid_argument("Failed to convert string to target type: " + s);
        }
        return val;
    }

    template <>
    std::string StrToT<std::string>(const std::string& s) {
        return s;
    }

    template <typename T>
    void SafeClear(T& item) {
        if constexpr (is_same_v<T, std::vector<typename T::value_type>>) {
            item.clear();
        } else {
            item = T{};
        }
    }

    struct StructSSD {
        std::string fields0;
        std::string fields1;
        double fields2;
    };

    template <typename T>
    T StructCvt(const std::vector<std::string>& fields);

    template <>
    StructSSD StructCvt<StructSSD>(const std::vector<std::string>& fields) {
        StructSSD data;
        if (fields.size() >= 3) {
            data.fields0 = fields[0];
            data.fields1 = fields[1];
            data.fields2 = StrToT<double>(fields[2]);
        }
        return data;
    }
    template <typename T>
    bool ParseLine(const string& line, T& item, char sep) {
        if (line.empty() || line[0] == '#') {
            return false;
        }

        istringstream iss(line);
        string field_str;
        vector<string> fields;
        while (getline(iss, field_str, sep)) {
            fields.push_back(field_str);
        }
        if (fields.empty()) {
            return false;
        }

        try {
            if constexpr (is_vector_v<T>) {
                SafeClear(item);
                for (const auto& f : fields) {
                    item.push_back(StrToT<typename T::value_type>(f));
                }
            } else if constexpr (is_nested_map_v<T>) {
                using K1 = typename T::key_type;
                using K2 = typename T::mapped_type::key_type;
                using V = typename T::mapped_type::mapped_type;
                if (fields.size() != 3) {
                    cerr << "Warning: NestedMap need 3*N fields, got " << fields.size() << endl;
                    return false;
                }
                K1 outerKey = StrToT<K1>(fields[0]);
                K2 innerKey = StrToT<K2>(fields[1]);
                V value = StrToT<V>(fields[2]);
                item[outerKey][innerKey] = value;
            } else if constexpr (is_single_level_map_v<T>) {
                using K = typename T::key_type;
                using V = typename T::mapped_type;
                if (fields.size() != 2) {
                    cerr << "Warning: Single-level map need 2*N fields, got " << fields.size() << endl;
                    return false;
                }
                K key = StrToT<K>(fields[0]);
                V val = StrToT<V>(fields[1]);
                item[key] = val;
            } else {
                SafeClear(item);
                item = StructCvt<T>(fields);
            }
        } catch (const std::exception& e) {
            cerr << "Error parsing line: " << line << " - " << e.what() << endl;
            return false;
        }

        if constexpr (is_vector_v<T> || is_unordered_map_v<T>) {
            return !item.empty();
        } else {
            if constexpr (std::is_same_v<T, StructSSD>) {
                return !item.fields0.empty();
            } else {
                return true;
            }
        }
    }
} // namespace

template <typename T>
optional<T> ParseFile(const string& file_path, char sep, int startLine) {
    ifstream file(file_path);
    if (!file.is_open()) {
        cerr << "Error: Failed to open file: " << file_path << endl;
        return nullopt;
    }

    T result;
    string line;

    if constexpr (is_vector_v<T>) {
        using ItemType = T::value_type;
        ItemType item;
        while (getline(file, line)) {
            if (ParseLine<ItemType>(line, item, sep)) {
                result.push_back(item);
            }
        }
    } else if constexpr (is_unordered_map_v<T>) {
        while (getline(file, line)) {
            ParseLine<T>(line, result, sep);
        }
    }

    if (result.empty()) {
        cerr << "Warning: No valid data in file - " << file_path << endl;
        return nullopt;
    }

    return result;
}

template std::optional<linkTimeType> ParseFile<linkTimeType>(const std::string&, char, int);
template std::optional<stationTimeType> ParseFile<stationTimeType>(const std::string&, char, int);
template std::optional<stationType> ParseFile<stationType>(const std::string&, char, int);
template std::optional<stringCfgType> ParseFile<stringCfgType>(const std::string&, char, int);