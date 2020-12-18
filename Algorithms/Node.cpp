//
// Created by adar on 12/8/2020.
//

#include "Node.h"

Node &Node::operator=(const Node &node) {
    if (this != &node) {
        this->_heuristic_cost = node.getHeuristicCost();
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
    // tie breaking in favor of the lowest actual cost of the path
    if (node1.getHeuristicCost() == node2.getHeuristicCost())
        return node1.getActualCost() > node2.getActualCost();
    return node1.getHeuristicCost() > node2.getHeuristicCost();
}

bool operator<(const Node &node1, const Node &node2) {
    if (node1.getHeuristicCost() == node2.getHeuristicCost())
        return node1.getActualCost() < node2.getActualCost();
    return node1.getHeuristicCost() < node2.getHeuristicCost();
}

void Node::insertElementToPath(const pair<int, int> &p) {
    this->_path_til_now.push(p);
}

void Node::setPathTilNow(const queue<pair<int, int>> &pathTilNow) {
    this->_path_til_now = queue<pair<int, int>>(pathTilNow);
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

int Node::getHeuristicCost() const {
    return _heuristic_cost;
}

void Node::setHeuristicCost(int heuristicCost) {
    _heuristic_cost = heuristicCost;
}

int Node::getActualCost() const {
    return _actual_cost;
}

void Node::setActualCost(int actualCost) {
    _actual_cost = actualCost;
}

