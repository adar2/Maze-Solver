//
// Created by adar on 12/8/2020.
//

#include "Node.h"

Node &Node::operator=(const Node &node) {
    if(this != &node){
        this->_path_cost = node.getPathCost();
        this->_row = node.getRow();
        this->_col = node.getCol();
        this->setPathTilNow(node.getPathTilNow());
    }
    return *this;
}

bool operator==(const Node &node1, const Node &node2) {
    return node1.getRow() == node2.getRow() && node1.getCol() == node2.getCol();
}

bool operator>(const Node &node1, const Node &node2) {
    return node1.getPathCost() > node2.getPathCost();
}

bool operator<(const Node &node1, const Node &node2) {
    return node1.getPathCost() < node2.getPathCost();
}

void Node::insertElementToPath(const pair<int, int> &p) {
    this->_path_til_now.push_back(p);
}

void Node::setPathTilNow(const vector<pair<int, int>> &pathTilNow) {
    this->_path_til_now.clear();
    for(auto &item : pathTilNow){
        this->_path_til_now.push_back(item);
    }
}

