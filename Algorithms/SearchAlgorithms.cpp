//
// Created by adar on 12/5/2020.
//

#include <iostream>
#include <string>
#include <algorithm>
#include "Node.h"

#define ACTIONS_SIZE 7
using namespace std;
enum actions {
    u, ur, r, dr, d, dl, l, ul
};


int uniformCostSearch(int **array, int dimension, int *start, int *target) {
    vector<Node> explored = vector<Node>();
    vector<Node> openList = vector<Node>();
    Node sourceNode = Node(0, start[0], start[1]);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, target[0], target[1]);
    Node current_node;
    int x, y;
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
                case u:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol();
                    break;
                case ur:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol() + 1;
                    break;
                case r:
                    x = current_node.getRow();
                    y = current_node.getCol() + 1;
                    break;
                case dr:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol() + 1;
                    break;
                case d:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol();
                    break;
                case dl:
                    x = current_node.getRow() + 1;
                    y = current_node.getCol() - 1;
                    break;
                case l:
                    x = current_node.getRow();
                    y = current_node.getCol() - 1;
                    break;
                case ul:
                    x = current_node.getRow() - 1;
                    y = current_node.getCol() - 1;
                    break;
            }
            if (x < 0 || x >= dimension || y < 0 || y >= dimension || array[x][y] < 0)
                continue;
            int cost = array[x][y] + current_node.getPathCost();
            Node node = Node(cost, x, y);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(x, y));
            if (!count(explored.begin(), explored.end(), node) && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node)!=openList.end() && find(openList.begin(), openList.end(), node)->getPathCost() > node.getPathCost())
            {
                openList.erase(find(openList.begin(), openList.end(), node));
                openList.push_back(node);
            }
        }
    }
    return 1;
}
