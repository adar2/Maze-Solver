//
// Created by adar on 12/8/2020.
//

#ifndef AI_PROJECT_NODE_H
#define AI_PROJECT_NODE_H

#include <utility>
#include <vector>
#include <queue>

#define ACTIONS_SIZE 7

enum actions {
    RU, R, RD, D, LD, L, LU, U
};

using std::vector;
using std::queue;
using std::pair;

class Node {
private:
    int _heuristic_cost;
    int _actual_cost;
    queue<pair<int, int>> _path_til_now;
    int _row;
    int _col;
    int _depth;
public:
    Node() : _heuristic_cost(0), _actual_cost(0), _row(0), _col(0), _depth(0) {};

    Node(int heuristic_cost, int actual_cost, int row, int col, int depth) : _heuristic_cost(heuristic_cost),
                                                                             _actual_cost(actual_cost), _row(row),
                                                                             _col(col), _depth(depth) {};

    Node(const Node &node) : _heuristic_cost(node.getHeuristicCost()), _actual_cost(node.getActualCost()),
                             _row(node.getRow()), _col(node.getCol()),
                             _depth(node.getDepth()) {
        this->setPathTilNow(node.getPathTilNow());
    };


    const queue<pair<int, int>> &getPathTilNow() const {
        return _path_til_now;
    }

    int getRow() const {
        return _row;
    }

    int getCol() const {
        return _col;
    }

    void setPathTilNow(const queue<pair<int, int>> &pathTilNow);

    void insertElementToPath(const pair<int, int> &p);

    void setRow(int row) {
        _row = row;
    }

    void setCol(int col) {
        _col = col;
    }

    int getDepth() const;

    void setDepth(int depth);

    int getHeuristicCost() const;

    void setHeuristicCost(int heuristicCost);

    int getActualCost() const;

    void setActualCost(int actualCost);

    Node &operator=(const Node &node);

    friend bool operator==(const Node &node1, const Node &node2);

    friend bool operator!=(const Node &node1, const Node &node2);

    friend bool operator>(const Node &node1, const Node &node2);

    friend bool operator<(const Node &node1, const Node &node2);
};


#endif //AI_PROJECT_NODE_H
