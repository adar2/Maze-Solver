//
// Created by adar on 12/8/2020.
//

#ifndef AI_PROJECT_NODE_H
#define AI_PROJECT_NODE_H

#include <utility>
#include <vector>

using std::vector;
using std::pair;

class Node {
private:
    int _path_cost;
    vector<pair<int, int>> _path_til_now;
    int _row;
    int _col;
    int _depth;
public:
    Node() : _path_cost(0), _row(0), _col(0), _depth(0) {};

    Node(int cost, int row, int col, int depth) : _path_cost(cost), _row(row), _col(col), _depth(depth) {};

    Node(const Node &node) : _path_cost(node.getPathCost()), _row(node.getRow()), _col(node.getCol()),
                             _depth(node.getDepth()) {
        this->setPathTilNow(node.getPathTilNow());
    };

    int getPathCost() const {
        return _path_cost;
    }

    const vector<pair<int, int>> &getPathTilNow() const {
        return _path_til_now;
    }

    int getRow() const {
        return _row;
    }

    int getCol() const {
        return _col;
    }

    void setPathCost(int pathCost) {
        _path_cost = pathCost;
    }

    void setPathTilNow(const vector<pair<int, int>> &pathTilNow);

    void setRow(int row) {
        _row = row;
    }

    void setCol(int col) {
        _col = col;
    }

    int getDepth() const;

    void setDepth(int depth);

    void insertElementToPath(const pair<int, int> &p);

    Node &operator=(const Node &node);

    static bool lessThenNodePtr(Node *node1, Node *node2) { return *node1 < *node2; }

    static bool equalToNodePtr(Node *node1, Node *node2) { return *node1 == *node2; }

    friend bool operator==(const Node &node1, const Node &node2);

    friend bool operator!=(const Node &node1, const Node &node2);

    friend bool operator>(const Node &node1, const Node &node2);

    friend bool operator<(const Node &node1, const Node &node2);
};


#endif //AI_PROJECT_NODE_H
