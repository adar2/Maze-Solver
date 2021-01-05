//
// Created by adar on 12/8/2020.
//

#ifndef AI_PROJECT_NODE_H
#define AI_PROJECT_NODE_H

#include <utility>
#include <vector>
#include <queue>
#include <memory>

#define ACTIONS_SIZE 8

using std::shared_ptr;
using std::vector;
using std::deque;
using std::pair;

enum actions {
    RU, R, RD, D, LD, L, LU, U
};

class Node {
private:
    // value of heuristic function from current node to goal node.
    double _heuristic_cost;
    // actual cost for reaching this node from source node.
    double _actual_cost;
    // holds the coordinates of nodes on the path to this node.
    deque<pair<int, int>> _path_til_now;
    // row coordinate of this node in the cost matrix.
    int _row;
    // col coordinate of this node in the cost matrix.
    int _col;
    // depth of this node in the search tree, equal to the length of the path.
    int _depth;
public:
    Node() : _heuristic_cost(0), _actual_cost(0), _row(0), _col(0), _depth(0) {};

    Node(double heuristic_cost, double actual_cost, int row, int col, int depth) : _heuristic_cost(heuristic_cost),
                                                                             _actual_cost(actual_cost), _row(row),
                                                                             _col(col), _depth(depth) {};

    Node(const Node &node) : _heuristic_cost(node.getHeuristicCost()), _actual_cost(node.getActualCost()),
                             _row(node.getRow()), _col(node.getCol()),
                             _depth(node.getDepth()) {
        this->setPathTilNow(node.getPathTilNow());
    };


    const deque<pair<int, int>> &getPathTilNow() const {
        return _path_til_now;
    }

    int getRow() const {
        return _row;
    }

    int getCol() const {
        return _col;
    }

    void setPathTilNow(const deque<pair<int, int>> &pathTilNow);

    void insertElementToPath(const pair<int, int> &p);

    void setRow(int row) {
        _row = row;
    }

    void setCol(int col) {
        _col = col;
    }

    int getDepth() const;

    void setDepth(int depth);

    double getHeuristicCost() const;

    void setHeuristicCost(double heuristicCost);

    double getActualCost() const;

    void setActualCost(double actualCost);

    Node &operator=(const Node &node);

    // function that returns node successors as a vector of pointers to nodes.
    shared_ptr<vector<shared_ptr<Node>>> successors(double **array,const int& dimension);

    // friend functions, operator overloading for nodes comparison.
    friend bool operator==(const Node &node1, const Node &node2);

    friend bool operator!=(const Node &node1, const Node &node2);

    friend bool operator>(const Node &node1, const Node &node2);

    friend bool operator<(const Node &node1, const Node &node2);
};

// functor to use as comparator in multiset of Node pointers.
struct lessCompNodePointers {
    bool operator()(const shared_ptr<Node>& node1,const shared_ptr<Node>& node2) const {
        // tie breaking in favor of the smallest h(n) value
        // f_cost = g(n) + h(n) -> h(n) = f_cost - g_cost
        if (node1->getHeuristicCost() == node2->getHeuristicCost())
            return (node1->getHeuristicCost() - node1->getActualCost()) < (node2->getHeuristicCost() - node2->getActualCost());
        return node1->getHeuristicCost() < node2->getHeuristicCost();
    }
};


#endif //AI_PROJECT_NODE_H
