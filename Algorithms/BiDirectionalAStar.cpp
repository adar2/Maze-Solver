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
    unordered_map<pair<int, int>, Node, pair_hash> visited_front = unordered_map<pair<int, int>, Node, pair_hash>();
    unordered_map<pair<int, int>, Node, pair_hash> visited_back = unordered_map<pair<int, int>, Node, pair_hash>();
    Node current_node;
    int row = 0, col = 0, expand_counter, f_cost = _heuristic_function(pair<int, int>(source[0], source[1]),
                                                                       pair<int, int>(goal[0], goal[1])), g_cost = 0;
    Node sourceNode = Node(f_cost, g_cost, source[0], source[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    // the heuristic function is symmetric so source to goal is the same as goal to source
    Node goalNode = Node(f_cost, g_cost, goal[0], goal[1], 0);
    goalNode.insertElementToPath(pair<int, int>(goalNode.getRow(), goalNode.getCol()));
    frontier_front.insert(sourceNode);
    frontier_back.insert(goalNode);
    while (!frontier_front.empty() && !frontier_back.empty()) {
        if (*frontier_front.begin() == *frontier_back.begin()) {
            // node v is at the top of both frontier_front and frontier_back
            break;
        }
        if (!frontier_front.empty()) {
            current_node = *frontier_front.begin();
            frontier_front.erase(frontier_front.begin());
            visited_front[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
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
                // use std::find as it uses operator== for comparison while set find method uses operator<
                auto open_list_iterator = std::find(frontier_front.begin(), frontier_front.end(), node);
                auto explored_iterator = visited_front.find(pair<int, int>(row, col));
                if (explored_iterator == visited_front.end() && open_list_iterator == frontier_front.end())
                    frontier_front.insert(node);
                else if (open_list_iterator != frontier_front.end() &&
                         open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                    frontier_front.erase(open_list_iterator);
                    frontier_front.insert(node);
                } else if (explored_iterator != visited_front.end() &&
                           explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                    visited_front.erase(explored_iterator);
                    frontier_front.insert(node);
                }
            }
            if (!expand_counter) {
                // cut off occurrence
                if (getInstance().getMin() == 0 || getInstance().getMin() > current_node.getDepth())
                    getInstance().setMin(current_node.getDepth());
                if (getInstance().getMax() == 0 || getInstance().getMax() < current_node.getDepth())
                    getInstance().setMax(current_node.getDepth());
                getInstance().addCutoffToSum(current_node.getDepth());
            }
        }
        if (!frontier_back.empty()) {
            current_node = *frontier_back.begin();
            frontier_back.erase(frontier_back.begin());
            visited_back[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
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
                                                      pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
                Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
                node.setPathTilNow(current_node.getPathTilNow());
                node.insertElementToPath(pair<int, int>(row, col));
                // use std::find as it uses operator== for comparison
                auto open_list_iterator = std::find(frontier_back.begin(), frontier_back.end(), node);
                auto explored_iterator = visited_back.find(pair<int, int>(row, col));
                if (explored_iterator == visited_back.end() && open_list_iterator == frontier_back.end())
                    frontier_back.insert(node);
                else if (open_list_iterator != frontier_back.end() &&
                         open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                    frontier_back.erase(open_list_iterator);
                    frontier_back.insert(node);
                } else if (explored_iterator != visited_back.end() &&
                           explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                    visited_back.erase(explored_iterator);
                    frontier_back.insert(node);
                }
            }
            if (!expand_counter) {
                // cut off occurrence
                if (getInstance().getMin() == 0 || getInstance().getMin() > current_node.getDepth())
                    getInstance().setMin(current_node.getDepth());
                if (getInstance().getMax() == 0 || getInstance().getMax() < current_node.getDepth())
                    getInstance().setMax(current_node.getDepth());
                getInstance().addCutoffToSum(current_node.getDepth());
            }
        }
    }
    // post phase find the optimum cost
    int min = 0, sum = 0;
    Node sol1,sol2;
    for (const auto &element : frontier_front) {
        auto found = std::find(frontier_back.begin(),frontier_back.end(),element);
        if(found != frontier_back.end() ){
            sum = element.getActualCost() + found->getActualCost();
            if (!min || sum < min) {
                min = sum;
                sol1 = element;
                sol2 = (*found);
            }
        }

    }
    std::cout << min << std::endl;
    std::cout << sol1.getDepth() + sol2.getDepth() -1 << std::endl;
    setExplored(visited_front.size());
    print_path(array, dimension, sol1);
    std::cout << std::endl;
    print_path(array, dimension, sol2);
    generate_stats();

    return 1;
}

BiDirectionalAStar &BiDirectionalAStar::getInstance() {
    static BiDirectionalAStar instance;
    return instance;
}


