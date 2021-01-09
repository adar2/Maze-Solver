

#include "Node.h"

Node &Node::operator=(const Node &node) {
    if (this != &node) {
        this->_evaluation_cost = node.getEvaluationCost();
        this->_actual_cost = node.getActualCost();
        this->_row = node.getRow();
        this->_col = node.getCol();
        this->setPathTilNow(node.getPathTilNow());
        this->_depth = node.getDepth();
    }
    return *this;
}

bool operator==(const Node &node1, const Node &node2) {
    return node1.getRow() == node2.getRow() && node1.getCol() == node2.getCol();
}

bool operator>(const Node &node1, const Node &node2) {
    // tie breaking in favor of the smallest h(n) value
    // f_cost = g(n) + h(n) -> h(n) = f_cost - g_cost
    if (node1.getEvaluationCost() == node2.getEvaluationCost())
        return (node1.getEvaluationCost() - node1.getActualCost()) > (node2.getEvaluationCost() - node2.getActualCost());
    return node1.getEvaluationCost() > node2.getEvaluationCost();
}

bool operator<(const Node &node1, const Node &node2) {
    // tie breaking in favor of the smallest h(n) value
    // f_cost = g(n) + h(n) -> h(n) = f_cost - g_cost
    if (node1.getEvaluationCost() == node2.getEvaluationCost())
        return (node1.getEvaluationCost() - node1.getActualCost()) < (node2.getEvaluationCost() - node2.getActualCost());
    return node1.getEvaluationCost() < node2.getEvaluationCost();
}

void Node::insertElementToPath(const pair<int, int> &p) {
    this->_path_til_now.push_back(p);
}

void Node::setPathTilNow(const deque<pair<int, int>> &pathTilNow) {
    this->_path_til_now = deque<pair<int, int>>(pathTilNow);
}

int Node::getDepth() const {
    return _depth;
}

void Node::setDepth(int depth) {
    _depth = depth;
}

bool operator!=(const Node &node1, const Node &node2) {
    return node1.getRow() != node2.getRow() || node1.getCol() != node2.getCol();
}

double Node::getEvaluationCost() const {
    return _evaluation_cost;
}

void Node::setEvaluationCost(double heuristicCost) {
    _evaluation_cost = heuristicCost;
}

double Node::getActualCost() const {
    return _actual_cost;
}

void Node::setActualCost(double actualCost) {
    _actual_cost = actualCost;
}

shared_ptr<vector<shared_ptr<Node>>> Node::successors(double **array,const int& dimension) {
    int row=0,col=0;
    double g_cost;
    Node& current_node = *this;
    shared_ptr<vector<shared_ptr<Node>>> successors (new vector<shared_ptr<Node>>());
    for (int i = 0; i < ACTIONS_SIZE; ++i) {
        switch (actions(i)) {
            case U:
                row = current_node.getRow() - 1;
                col = current_node.getCol();
                break;
            case RU:
                row = current_node.getRow() - 1;
                col = current_node.getCol() + 1;
                break;
            case R:
                row = current_node.getRow();
                col = current_node.getCol() + 1;
                break;
            case RD:
                row = current_node.getRow() + 1;
                col = current_node.getCol() + 1;
                break;
            case D:
                row = current_node.getRow() + 1;
                col = current_node.getCol();
                break;
            case LD:
                row = current_node.getRow() + 1;
                col = current_node.getCol() - 1;
                break;
            case L:
                row = current_node.getRow();
                col = current_node.getCol() - 1;
                break;
            case LU:
                row = current_node.getRow() - 1;
                col = current_node.getCol() - 1;
                break;
        }
        // if its index is out of range or its a wall, represented by -1 in the matrix continue.
        if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0)
            continue;
        g_cost = array[row][col] + current_node.getActualCost();
        shared_ptr<Node> successor(new Node(g_cost, g_cost, row, col, current_node.getDepth() + 1));
        // copy predecessor path
        successor->setPathTilNow(current_node.getPathTilNow());
        // insert new node coordinates to the path
        successor->insertElementToPath(pair<int, int>(row, col));
        successors->push_back(successor);
    }
    return successors;
}

