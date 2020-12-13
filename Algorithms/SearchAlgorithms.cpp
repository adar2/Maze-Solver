//
// Created by adar on 12/5/2020.
//

#include <iostream>
#include <string>
#include <algorithm>
#include <stack>
#include "Node.h"

#define ACTIONS_SIZE 7
using namespace std;
enum actions {
    RU, R, RD, D, LD, L, LU, U
};


int uniformCostSearch(int **array, int dimension, int *start, int *target) {
    vector<Node> explored = vector<Node>();
    vector<Node> openList = vector<Node>();
    Node sourceNode = Node(0, start[0], start[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, target[0], target[1], 0);
    Node current_node;
    int x = 0, y = 0;
    openList.push_back(sourceNode);
    while (!openList.empty()) {
        sort(openList.begin(), openList.end(), greater<Node>());
        current_node = openList.back();
        openList.pop_back();
        if (current_node == goalNode) {
            for (const auto &i : current_node.getPathTilNow()) {
                cout << i.first << ',' << i.second << endl;
            }
            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    pair<int, int> p = pair<int, int>(i, j);
                    if (count(current_node.getPathTilNow().begin(), current_node.getPathTilNow().end(), p))
                        cout << "\033[1;31m" << '(' << array[i][j] << ")," << "\033[0m";
                    else
                        cout << '(' << array[i][j] << "),";
                }
                cout << endl;
            }
            cout << "path cost: " << current_node.getPathCost() << endl;
            cout << "nodes explored: " << explored.size() << endl;
            // reached to goal state
            return 0;
        }
        explored.push_back(current_node);

        for (int i = 0; i < ACTIONS_SIZE; ++i) {
            switch (actions(i)) {
                case U:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol();
                    break;
                case RU:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol() + 1;
                    break;
                case R:
                    x = current_node.getRow();
                    y = current_node.getCol() + 1;
                    break;
                case RD:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol() + 1;
                    break;
                case D:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol();
                    break;
                case LD:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol() - 1;
                    break;
                case L:
                    x = current_node.getRow();
                    y = current_node.getCol() - 1;
                    break;
                case LU:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol() - 1;
                    break;
            }
            if (x < 0 || x >= dimension || y < 0 || y >= dimension || array[x][y] < 0)
                continue;
            int cost = array[x][y] + current_node.getPathCost();
            Node node = Node(cost, x, y, current_node.getDepth() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(x, y));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node) != openList.end() &&
                     find(openList.begin(), openList.end(), node)->getPathCost() > node.getPathCost()) {
                openList.erase(find(openList.begin(), openList.end(), node));
                openList.push_back(node);
            }
        }
    }
    return 1;
}

//pair<Node *, bool> DLS(int **array, int dimension, Node *current_node, Node *goal, int depth) {
//    bool any_remaining = false, remaining;
//    Node *found, *node = nullptr;
//    int x = 0, y = 0;
//    if (depth == 0) {
//        if (*current_node == *goal) {
//            return {current_node, true};
//        } else {
//            return {nullptr, true};
//        }
//    } else if (depth > 0) {
//        for (int i = 0; i < ACTIONS_SIZE; ++i) {
//            switch (actions(i)) {
//                case U:
//                    x = current_node->getRow() - 1;
//                    y = current_node->getCol();
//                    break;
//                case RU:
//                    x = current_node->getRow() - 1;
//                    y = current_node->getCol() + 1;
//                    break;
//                case R:
//                    x = current_node->getRow();
//                    y = current_node->getCol() + 1;
//                    break;
//                case RD:
//                    x = current_node->getRow() + 1;
//                    y = current_node->getCol() + 1;
//                    break;
//                case D:
//                    x = current_node->getRow() + 1;
//                    y = current_node->getCol();
//                    break;
//                case LD:
//                    x = current_node->getRow() + 1;
//                    y = current_node->getCol() - 1;
//                    break;
//                case L:
//                    x = current_node->getRow();
//                    y = current_node->getCol() - 1;
//                    break;
//                case LU:
//                    x = current_node->getRow() - 1;
//                    y = current_node->getCol() - 1;
//                    break;
//            }
//            if (x < 0 || x >= dimension || y < 0 || y >= dimension || array[x][y] < 0)
//                continue;
//            // avoid going back to where you came from
//            if (current_node->getPathTilNow().size() >= 2) {
//                if (current_node->getPathTilNow().at(current_node->getPathTilNow().size() - 2) == pair<int, int>(x, y))
//                    continue;
//            }
//            int cost = array[x][y] + current_node->getPathCost();
//            node = new Node(cost, x, y);
//            node->setPathTilNow(current_node->getPathTilNow());
//            node->insertElementToPath(pair<int, int>(x, y));
//            auto temp = DLS(array, dimension, node, goal, depth - 1);
//            found = temp.first;
//            remaining = temp.second;
//            if (found != nullptr)
//                return {found, true};
//            if (remaining) {
//                any_remaining = true;
//            }
//            delete node;
//        }
//
//    }
//    return {nullptr, any_remaining};
//}

Node* DLS(int **array, int dimension, Node *root, Node *goal, int limit) {
    stack<Node *> frontier = stack<Node *>();
    Node *current_node, *node;
    int x = 0, y = 0;
    frontier.push(root);
    while (!frontier.empty()) {
        current_node = frontier.top();
        frontier.pop();
        if (*current_node == *goal) {
            while (!frontier.empty()) {
                node = frontier.top();
                frontier.pop();
                delete node;
            }
            return current_node;
        }

        if (current_node->getDepth() < limit) {
            for (int i = 0; i < ACTIONS_SIZE; ++i) {
                switch (actions(i)) {
                    case U:
                        x = current_node->getRow() - 1;
                        y = current_node->getCol();
                        break;
                    case RU:
                        x = current_node->getRow() - 1;
                        y = current_node->getCol() + 1;
                        break;
                    case R:
                        x = current_node->getRow();
                        y = current_node->getCol() + 1;
                        break;
                    case RD:
                        x = current_node->getRow() + 1;
                        y = current_node->getCol() + 1;
                        break;
                    case D:
                        x = current_node->getRow() + 1;
                        y = current_node->getCol();
                        break;
                    case LD:
                        x = current_node->getRow() + 1;
                        y = current_node->getCol() - 1;
                        break;
                    case L:
                        x = current_node->getRow();
                        y = current_node->getCol() - 1;
                        break;
                    case LU:
                        x = current_node->getRow() - 1;
                        y = current_node->getCol() - 1;
                        break;
                }
                if (x < 0 || x >= dimension || y < 0 || y >= dimension || array[x][y] < 0)
                    continue;
                if (current_node->getPathTilNow().size() >= 2) {
                    if (current_node->getPathTilNow().at(current_node->getPathTilNow().size() - 2) ==
                        pair<int, int>(x, y))
                        continue;
                }
                int cost = array[x][y] + current_node->getPathCost();
                node = new Node(cost, x, y, current_node->getDepth() + 1);
                node->setPathTilNow(current_node->getPathTilNow());
                node->insertElementToPath(pair<int, int>(x, y));
                frontier.push(node);
            }
        }
        // compare pointers by addresses.
        if (current_node != root) {
            delete current_node;
        }
    }
    return nullptr;
}


int IDS(int **array, int dimension, int *source, int *goal) {
    Node *found;
    Node *root = new Node(0, source[0], source[1], 0);
    root->insertElementToPath(pair<int, int>(source[0], source[1]));
    Node *target = new Node(0, goal[0], goal[1], 0);
    for (int i = 0; i < dimension; ++i) {
        cout << "current penetration level: " << i << endl;
        found = DLS(array, dimension, root, target, i);
        if (found != nullptr) {
            // found the node
            delete root;
            delete target;
            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    pair<int, int> p = pair<int, int>(i, j);
                    if (count(found->getPathTilNow().begin(), found->getPathTilNow().end(), p))
                        cout << "\033[1;31m" << '(' << array[i][j] << ")," << "\033[0m";
                    else
                        cout << '(' << array[i][j] << "),";
                }
                cout << endl;
            }
            delete found;
            return 0;// return success.
        }
    }
    delete root;
    delete target;
    return 1;//return failure.
}