//
// Created by r00t on 12/15/20.
//

#include "BiDirectionalAStar.h"
#include <set>
#include <unordered_map>

using std::multiset;
using std::unordered_map;

int BiDirectionalAStar::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    multiset<Node> frontier_front = multiset<Node>();
    multiset<Node> frontier_back = multiset<Node>();
    unordered_map<pair<int, int>, Node, pair_hash> visited = unordered_map<pair<int, int>, Node, pair_hash>();
    Node current_node;
    int row = 0, col = 0, expand_counter, f_cost = _heuristic_function(pair<int, int>(source[0], source[1]),
                                                                       pair<int, int>(goal[0], goal[1])), g_cost = 0;
    Node sourceNode = Node(f_cost, g_cost, source[0], source[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, 0, goal[0], goal[1], 0);
    frontier_front.insert(sourceNode);
    frontier_back.insert(goalNode);
    while (!frontier_front.empty() && !frontier_back.empty()) {
        if (!frontier_front.empty()) {
            current_node = *frontier_front.begin();
            frontier_front.erase(current_node);
            if (current_node == goalNode || frontier_back.count(current_node)){
                print_path(array,dimension,current_node);
                return 0;
            }
            expand_counter = 0;
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
                if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0)
                    continue;
                expand_counter++;
                g_cost = array[row][col] + current_node.getActualCost();
                f_cost = g_cost + _heuristic_function(pair<int, int>(row, col),
                                                      pair<int, int>(goalNode.getRow(), goalNode.getCol()));
                Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
                node.setPathTilNow(current_node.getPathTilNow());
                node.insertElementToPath(pair<int, int>(row, col));
                // use std::find as it uses operator== for comparison
                auto open_list_iterator = std::find(frontier_front.begin(), frontier_front.end(), node);
                auto explored_iterator = visited.find(pair<int, int>(row, col));
                if (explored_iterator == visited.end() && open_list_iterator == frontier_front.end())
                    frontier_front.insert(node);
                else if (open_list_iterator != frontier_front.end() &&
                         open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                    frontier_front.erase(open_list_iterator);
                    frontier_front.insert(node);
                } else if (explored_iterator != visited.end() &&
                           explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                    visited.erase(explored_iterator);
                    frontier_front.insert(node);
                }
            }
        }
        if(!frontier_back.empty()){
            current_node = *frontier_back.begin();
            frontier_back.erase(current_node);
            if (current_node == sourceNode || frontier_front.count(current_node)){
                print_path(array,dimension,current_node);
                return 0;
            }
            expand_counter = 0;
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
                if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0)
                    continue;
                expand_counter++;
                g_cost = array[row][col] + current_node.getActualCost();
                f_cost = g_cost + _heuristic_function(pair<int, int>(row, col),
                                                      pair<int, int>(goalNode.getRow(), goalNode.getCol()));
                Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
                node.setPathTilNow(current_node.getPathTilNow());
                node.insertElementToPath(pair<int, int>(row, col));
                // use std::find as it uses operator== for comparison
                auto open_list_iterator = std::find(frontier_back.begin(), frontier_back.end(), node);
                auto explored_iterator = visited.find(pair<int, int>(row, col));
                if (explored_iterator == visited.end() && open_list_iterator == frontier_back.end())
                    frontier_back.insert(node);
                else if (open_list_iterator != frontier_front.end() &&
                         open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                    frontier_back.erase(open_list_iterator);
                    frontier_back.insert(node);
                } else if (explored_iterator != visited.end() &&
                           explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                    visited.erase(explored_iterator);
                    frontier_back.insert(node);
                }
            }
        }
    }
    return 1;
}

BiDirectionalAStar &BiDirectionalAStar::getInstance() {
    static BiDirectionalAStar instance;
    return instance;
}


