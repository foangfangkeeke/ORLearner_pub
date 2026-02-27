#include "branch_and_price.hpp"

Status BranchAndPrice::Initialize() {
    auto rootNode = std::make_shared<BranchNode>(problemType);
    rootNode->nodeId = nextNodeId++;
    if (rootNode->cgSolver->Initialize() != OK) {
        std::cerr << "Root node CG initialization failed!" << std::endl;
        return ERROR;
    }
    nodeQueue.push(rootNode);
    return OK;
}

bool BranchAndPrice::CheckIntegerSolution(const ColumnGeneration& cg, double tol) {
    auto model = cg.GetModel();
    int numVars = model->get(GRB_IntAttr_NumVars);
    GRBVar* vars = model->getVars();

    for (int i = 0; i < numVars; ++i) {
        double val = vars[i].get(GRB_DoubleAttr_X);
        if (fabs(val - round(val)) > tol) {
            return false;
        }
    }
    return true;
}

int BranchAndPrice::SelectBranchingVariable(const ColumnGeneration& cg, double tol) {
    auto model = cg.GetModel();
    int numVars = model->get(GRB_IntAttr_NumVars);
    GRBVar* vars = model->getVars();
    int branchVar = -1;
    double maxFraction = 0.0;

    for (int i = 0; i < numVars; ++i) {
        double val = vars[i].get(GRB_DoubleAttr_X);
        double fraction = fabs(val - round(val));
        if (fraction > tol && fraction > maxFraction) {
            maxFraction = fraction;
            branchVar = i;
        }
    }
    return branchVar;
}

std::pair<std::shared_ptr<BranchNode>, std::shared_ptr<BranchNode>>
BranchAndPrice::CreateChildNodes(std::shared_ptr<BranchNode> parent, int branchVar, double branchVal) {
    auto leftNode = std::make_shared<BranchNode>(*parent->cgSolver);
    leftNode->nodeId = nextNodeId++;
    leftNode->parent = parent;
    leftNode->branchConstraints = parent->branchConstraints;
    leftNode->branchConstraints[branchVar] = {0.0, floor(branchVal)};

    auto rightNode = std::make_shared<BranchNode>(*parent->cgSolver);
    rightNode->nodeId = nextNodeId++;
    rightNode->parent = parent;
    rightNode->branchConstraints = parent->branchConstraints;
    rightNode->branchConstraints[branchVar] = {ceil(branchVal), INFINITY};

    return {leftNode, rightNode};
}

void BranchAndPrice::ApplyBranchConstraints(ColumnGeneration& cg, const std::map<int, std::pair<double, double>>& constraints) {
    auto model = cg.GetModel();
    GRBVar* vars = model->getVars();

    for (const auto& [varIdx, bounds] : constraints) {
        double lb = bounds.first;
        double ub = bounds.second;
        vars[varIdx].set(GRB_DoubleAttr_LB, lb);
        vars[varIdx].set(GRB_DoubleAttr_UB, ub);
    }
    model->update();
}

Status BranchAndPrice::ProcessNode(std::shared_ptr<BranchNode> node) {
    ApplyBranchConstraints(*node->cgSolver, node->branchConstraints);

    Status cgStatus = node->cgSolver->Solve();
    if (cgStatus != OK) {
        std::cerr << "Node " << node->nodeId << " CG solve failed!" << std::endl;
        return ERROR;
    }

    auto model = node->cgSolver->GetModel();
    node->lowerBound = model->get(GRB_DoubleAttr_ObjVal);

    if (node->lowerBound >= globalUpperBound + 1e-6) {
        std::cout << "Node " << node->nodeId << " pruned (LB=" << node->lowerBound << " ≥ GUB=" << globalUpperBound << ")" << std::endl;
        return OK;
    }

    node->isInteger = CheckIntegerSolution(*node->cgSolver);
    if (node->isInteger) {
        if (node->lowerBound < globalUpperBound) {
            globalUpperBound = node->lowerBound;
            int numVars = model->get(GRB_IntAttr_NumVars);
            GRBVar* vars = model->getVars();
            integerSolution.clear();
            for (int i = 0; i < numVars; ++i) {
                integerSolution[i] = vars[i].get(GRB_DoubleAttr_X);
            }
            std::cout << "Found integer solution: obj=" << globalUpperBound << std::endl;
        }
        return OK;
    }

    int branchVar = SelectBranchingVariable(*node->cgSolver);
    if (branchVar == -1) {
        std::cerr << "No branching variable found!" << std::endl;
        return ERROR;
    }
    std::string branchVarName = model->getVars()[branchVar].get(GRB_StringAttr_VarName);
    double branchVal = model->getVar(branchVar).get(GRB_DoubleAttr_X);

    auto [leftNode, rightNode] = CreateChildNodes(node, branchVar, branchVal);

    nodeQueue.push(leftNode);
    nodeQueue.push(rightNode);

    std::cout << "Node " << node->nodeId << " branched on var " << branchVarName 
              << " (val=" << branchVal << "), left/right nodes: " 
              << leftNode->nodeId << "/" << rightNode->nodeId << "\n" << std::endl;

    return OK;
}


Status BranchAndPrice::Solve() {
    while (!nodeQueue.empty()) {
        auto currentNode = nodeQueue.front();
        nodeQueue.pop();

        std::cout << "Processing node " << currentNode->nodeId << std::endl;
        std::cout << "Current global upper bound: " << globalUpperBound << std::endl;
        Status nodeStatus = ProcessNode(currentNode);
        if (nodeStatus != OK) {
            std::cerr << "Node " << currentNode->nodeId << " processing failed!" << std::endl;
            return ERROR;
        }
    }

    std::cout << "Global upper bound (integer obj): " << globalUpperBound << std::endl;

    return OK;
}

BranchAndPrice::BranchAndPrice(ProblemType problemType): problemType(problemType) {};

BranchAndPrice::~BranchAndPrice() {};