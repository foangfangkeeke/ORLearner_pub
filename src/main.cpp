#include "test.hpp"
#include "cutting_stock_problem.hpp"
#include "direct_solver.hpp"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(936);
    SetConsoleCP(936);
#endif

    cout << fixed << setprecision(6);

    bool showHelp = false;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            showHelp = true;
            break;
        }
    }
    if (showHelp) {
        cout << "basic usage: " << endl;
        cout << "  " << argv[0] << " --alg solver --pb wirelesschargingstratgies --desc m1 --data test" << endl;
        cout << "  " << argv[0] << " --alg cg --pb cuttingstock" << endl;
        cout << "  " << argv[0] << " --alg direct --pb cuttingstock" << endl;
        return 0;
    }

    if (argc == 1) {
        cout << "No arguments provided, running test function...（无参数传入，运行测试函数）" << endl;
        SolveWithGurobiTest();
        return 0;
    }

    string alg = "unsigned";
    string pb = "unsigned";
    string desc = "unsigned";
    string dataType = "test";
    bool ifIter = true;
    bool sweepSubsidy = false;
    bool sweepSocRangeChange = false;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--alg" && i + 1 < argc) {
            alg = argv[++i];
        }
        else if (arg == "--pb" && i + 1 < argc) {
            pb = argv[++i];
        }
        else if (arg == "--desc" && i + 1 < argc) {
            desc = argv[++i];
        }
        else if (arg == "--data" && i + 1 < argc) {
            dataType = argv[++i];
        }
    }

    if (alg != "unsigned" && pb != "unsigned") {
        auto splitComma = [](const string& value) {
            vector<string> parts;
            string token;
            stringstream ss(value);
            while (getline(ss, token, ',')) {
                if (!token.empty()) {
                    parts.push_back(token);
                }
            }
            if (parts.empty()) {
                parts.push_back(value);
            }
            return parts;
        };

        vector<string> descList = splitComma(desc);
        string algLower = alg;
        string pbLower = pb;
        for (char& c : algLower) c = tolower(c);
        for (char& c : pbLower) c = tolower(c);
        for (const auto& descVal : descList) {
            string descUpper = descVal;
            for (char& c : descUpper) c = toupper(c);

            string coreText = "使用 " + alg + " 求解 " + pb + "_" + descVal;
            int totalWidth = 80;
            int InfoWidth = coreText.size();
            int padding = max(0, (totalWidth - InfoWidth) / 2);
            cout << "\n" << string(totalWidth, '=') << endl;
            cout << string(padding, ' ') << coreText;
            cout << "\n" << string(totalWidth, '=') << endl;

            bool err = false;

            if (algLower == "solver") {
                SolveWithGurobiTest();
            } else if (algLower == "cg") {
                if (pbLower == "cuttingstock") {
                    auto pbObj = ColumnGeneration(CUTTINGSTOCK);
                    pbObj.Run();
                } else {
                    err = true;
                }
            } else if (algLower == "direct") {
                if (pbLower == "cuttingstock") {
                    auto pbObj = DirectSolver(CUTTINGSTOCK);
                    pbObj.Run();
                } else {
                    err = true;
                }
            } else {
                err = true;
            }
            if (err) {
                cout << "invalid problem or algorithm combination: " << alg << " & " << pb << "（无效的问题或算法组合）" << endl;
            }
        }
    }

    return 0;
}