#include "column_generation.hpp"
#include <queue>
#include <memory>
#include <cmath>
#include <map>

struct BranchNode {
    std::unique_ptr<ColumnGeneration> cgSolver;
    std::map<int, std::pair<double, double>> branchConstraints; 
    double lowerBound;
    double upperBound;
    bool isInteger;
    int nodeId;
    std::shared_ptr<BranchNode> parent;

    BranchNode(ProblemType type): 
        cgSolver(std::make_unique<ColumnGeneration>(type)),
        lowerBound(0.0), upperBound(INFINITY),
        isInteger(false), nodeId(0), parent(nullptr) {}

    BranchNode(const ColumnGeneration& other): 
        cgSolver(other.Clone()),
        lowerBound(0.0), upperBound(INFINITY),
        isInteger(false), nodeId(0), parent(nullptr) {}
};

class BranchAndPrice : public IMILPAlgorithmStrategy {
public:
    BranchAndPrice(ProblemType problemType);
    Status Initialize();
    Status Solve();
    ~BranchAndPrice();

private:
    ProblemType problemType;
    std::queue<std::shared_ptr<BranchNode>> nodeQueue;
    double globalUpperBound = INFINITY;
    int nextNodeId = 0;
    std::map<int, double> integerSolution;

    Status ProcessNode(std::shared_ptr<BranchNode> node);
    bool CheckIntegerSolution(const ColumnGeneration& cg, double tol = 1e-6);
    int SelectBranchingVariable(const ColumnGeneration& cg, double tol = 1e-6);
    std::pair<std::shared_ptr<BranchNode>, std::shared_ptr<BranchNode>> 
        CreateChildNodes(std::shared_ptr<BranchNode> parent, int branchVar, double branchVal);
    void ApplyBranchConstraints(ColumnGeneration& cg, const std::map<int, std::pair<double, double>>& constraints);
};