//
// Created by adar on 12/5/2020.
//

#include <iostream>
#include <queue>
#include <string>
#include <algorithm>

using namespace std;

struct Node {
private:
    int path_cost;
    vector<pair<int, int>> path_til_now;
    int row;
    int col;
public:
    Node() : path_cost(0), row(0), col(0) {};

    Node(const Node &node) : path_cost(node.path_cost), row(node.row), col(node.col) {this->setPathTilNow(node.getPathTilNow());};

    Node(int cost, int x, int y) : path_cost(cost), row(x), col(y) {};

    int getCost() { return path_cost; };

    int getRow() { return row; };

    int getCol() { return col; };

    Node &operator=(const Node &node) {
        if (this != &node) {
            this->path_cost = node.path_cost;
            this->row = node.row;
            this->col = node.col;
            this->setPathTilNow(node.getPathTilNow());
        }
        return *this;
    };

    friend bool operator==(const Node &node1, const Node &node2);

    friend bool operator>(const Node &node1, const Node &node2);

    friend bool operator<(const Node &node1, const Node &node2);

    int getPathCost() const;

    void setPathCost(int pathCost);

    const vector<pair<int, int>> &getPathTilNow() const;

    void setPathTilNow(const vector<pair<int, int>> &pathTilNow);

    void setRow(int row);

    void setCol(int col);

    void insertElementToPath(pair<int, int> p) { this->path_til_now.push_back(p); }
};

bool operator>(const Node &node1, const Node &node2) { return node1.path_cost > node2.path_cost; }

bool operator==(const Node &node1, const Node &node2) { return node1.row == node2.row && node1.col == node2.col; }

bool operator<(const Node &node1, const Node &node2) {
    return node1.path_cost < node2.path_cost;
}

int uniformCostSearch(int **array, int dimension, int *start, int *target) {
    vector<Node> explored = vector<Node>();
    vector<Node> openList = vector<Node>();
    Node sourceNode = Node(0, start[0], start[1]);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, target[0], target[1]);
    Node current_node;
    openList.push_back(sourceNode);
    while (!openList.empty()) {
        sort(openList.begin(), openList.end(),greater<Node>());
        current_node = openList.back();
        openList.pop_back();
        if (current_node == goalNode) {
            for (const auto & i : current_node.getPathTilNow()) {
                cout << i.first << ','<<i.second<<endl;
            }
            for(int i=0;i<dimension;++i){
                for(int j=0;j<dimension;++j){
                    pair<int,int> p = pair<int,int>(i,j);
                    if(count(current_node.getPathTilNow().begin(),current_node.getPathTilNow().end(),p))
                        cout << "\033[1;31m" << '(' << array[i][j] << ")," << "\033[0m";
                    else
                        cout << '(' << array[i][j] << "),";
                }
                cout << endl;
            }
            cout << "path cost: " << current_node.getCost() << endl;
            cout << "nodes explored: " <<explored.size() << endl;
            // reached to goal state
            return 0;
        }
        explored.push_back(current_node);
        //UP
        if (current_node.getRow() - 1 >= 0 && array[current_node.getRow() - 1][current_node.getCol()] >= 0) {
            int cost = array[current_node.getRow() - 1][current_node.getCol()] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() - 1, current_node.getCol());
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }

        //UP-RIGHT
        if (current_node.getRow() - 1 >= 0 && current_node.getCol() + 1 < dimension &&
            array[current_node.getRow() - 1][current_node.getCol() + 1] >= 0) {
            int cost = array[current_node.getRow() - 1][current_node.getCol() + 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() - 1,
                             current_node.getCol() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //RIGHT
        if (current_node.getCol() + 1 < dimension && array[current_node.getRow()][current_node.getCol() + 1] >= 0) {
            int cost = array[current_node.getRow()][current_node.getCol() + 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow(),
                             current_node.getCol() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //DOWN-RIGHT
        if (current_node.getRow() + 1 < dimension && current_node.getCol() + 1 < dimension &&
            array[current_node.getRow() + 1][current_node.getCol() + 1] >= 0) {
            int cost = array[current_node.getRow() + 1][current_node.getCol() + 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() + 1,
                             current_node.getCol() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //DOWN
        if (current_node.getRow() + 1 < dimension && array[current_node.getRow() + 1][current_node.getCol()] >= 0) {
            int cost = array[current_node.getRow() + 1][current_node.getCol()] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() + 1,
                             current_node.getCol());
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //DOWN-LEFT
        if (current_node.getCol() - 1 >= 0 && current_node.getRow() + 1 < dimension &&
            array[current_node.getRow() + 1][current_node.getCol() - 1] >= 0) {
            int cost = array[current_node.getRow() + 1][current_node.getCol() - 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() + 1,
                             current_node.getCol() - 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //LEFT
        if (current_node.getCol() - 1 >= 0 && array[current_node.getRow()][current_node.getCol() - 1] >= 0) {
            int cost = array[current_node.getRow()][current_node.getCol() - 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow(),
                             current_node.getCol() - 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
        //UP-LEFT
        if (current_node.getRow() - 1 >= 0 && current_node.getCol() - 1 >= 0 &&
            array[current_node.getRow() - 1][current_node.getCol() - 1] >= 0) {
            int cost = array[current_node.getRow() - 1][current_node.getCol() - 1] + current_node.getCost();
            Node node = Node(cost, current_node.getRow() - 1,
                             current_node.getCol() - 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(node.getRow(), node.getCol()));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)->getCost() > node.getCost())
                openList.erase(find(openList.begin(), openList.end(), node));
        }
    }
    return 1;
}

int Node::getPathCost() const {
    return path_cost;
}

void Node::setPathCost(int pathCost) {
    path_cost = pathCost;
}

const vector<pair<int, int>> &Node::getPathTilNow() const {
    return
            path_til_now;
}

void Node::setPathTilNow(const vector<pair<int, int>> &pathTilNow) {
    this->path_til_now.clear();
    for (pair<int, int> p :pathTilNow) {
        this->path_til_now.push_back(p);
    }
}

void Node::setRow(int row) {
    Node::row = row;
}

void Node::setCol(int col) {
    Node::col = col;
}
