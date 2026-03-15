#include "using_solver.hpp"
#include "column_generation.hpp"
#include "branch_and_price.hpp"
#include "benders_decomposition.hpp"
#include "barp_s_integer_l_shaped.hpp"

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
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
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
        int InfoWidth = static_cast<int>(coreText.size());
        int padding = max(0, (totalWidth - InfoWidth) / 2);
        cout << "\n" << string(totalWidth, '=') << endl;
        cout << string(padding, ' ') << coreText;
        cout << "\n" << string(totalWidth, '=') << endl;

        bool err = false;

        ProblemType problemType;
        if (pbLower == "cuttingstock") {
            problemType = CUTTINGSTOCK;
        } else if (pbLower == "test") {
            problemType = TEST;
        } else if (pbLower == "fctp" || pbLower == "fixedchargetransportation" || pbLower == "fixed_charge_transportation") {
            problemType = FCTP;
        } else if (pbLower == "barp_s" || pbLower == "barp" || pbLower == "barps" || pbLower == "brs_allocation_and_rescheduling_train_timetables") {
            problemType = BARP_S;
        } else {
            err = true;
        }

        if (err) {
            cout << "invalid problem " << pb << endl;
            return 0;
        }

        MILPSolver solver;

        if (algLower == "cg") { // TODO: 直接算法名传进去更好
            solver.SetAlgorithm(std::make_unique<ColumnGeneration>(problemType));
        } else if (algLower == "bp") {
            solver.SetAlgorithm(std::make_unique<BranchAndPrice>(problemType));
        } else if (algLower == "lshape" || algLower == "lshaped") {
            solver.SetAlgorithm(std::make_unique<BARPSIntegerLShaped>(problemType));
        } else if (algLower == "benders" || algLower == "bd") {
            solver.SetAlgorithm(std::make_unique<BendersDecomposition>(problemType));
        } else {
            solver.SetAlgorithm(std::make_unique<UsingSolver>(problemType));
        }
        try {
            solver.Run();
        }
        catch (const std::exception& e) {
            cout << "Error: " << e.what() << endl;
        }
    }

    return 0;
}