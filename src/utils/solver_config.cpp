#include "solver_config.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {
const char* kSolverConfigPath = "solverCfg.txt";

std::string Trim(const std::string& text)
{
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) {
        ++begin;
    }

    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
        --end;
    }

    return text.substr(begin, end - begin);
}

std::string StripInlineComment(const std::string& line)
{
    const size_t commentPos = line.find('#');
    return Trim(commentPos == std::string::npos ? line : line.substr(0, commentPos));
}
} // namespace

SolverConfig LoadSolverConfig()
{
    std::ifstream fin(kSolverConfigPath);

    SolverConfig config;
    bool hasMipGap = false;
    bool hasTimeLimit = false;
    std::string line;
    int lineNo = 0;
    while (std::getline(fin, line)) {
        ++lineNo;
        line = StripInlineComment(line);
        if (line.empty()) {
            continue;
        }

        const size_t equalsPos = line.find('=');
        if (equalsPos == std::string::npos) {
            throw std::runtime_error("Invalid solver config line " + std::to_string(lineNo) + ": " + line);
        }

        const std::string key = Trim(line.substr(0, equalsPos));
        const std::string value = Trim(line.substr(equalsPos + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error("Invalid solver config line " + std::to_string(lineNo) + ": " + line);
        }

        if (key == "mipGap") {
            config.mipGap = std::stod(value);
            hasMipGap = true;
        } else if (key == "timeLimit") {
            config.timeLimit = std::stod(value);
            hasTimeLimit = true;
        } else if (key == "presolve") {
            config.presolve = std::stoi(value);
        } else if (key == "mipFocus") {
            config.mipFocus = std::stoi(value);
        } else if (key == "heuristics") {
            config.heuristics = std::stod(value);
        } else if (key == "threads") {
            config.threads = std::stoi(value);
        } else if (key == "method") {
            config.method = std::stoi(value);
        } else {
            throw std::runtime_error("Unknown solver config key: " + key);
        }
    }

    if (!hasMipGap || !hasTimeLimit) {
        throw std::runtime_error("solverCfg.txt must define mipGap and timeLimit");
    }
    return config;
}