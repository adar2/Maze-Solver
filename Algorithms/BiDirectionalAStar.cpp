//
// Created by r00t on 12/15/20.
//

#include "BiDirectionalAStar.h"
#include <set>
#include <unordered_map>

using std::multiset;
using std::unordered_map;

int BiDirectionalAStar::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    //open list front search
    multiset<Node> frontier_front = multiset<Node>();
    //open list backward search
    multiset<Node> frontier_back = multiset<Node>();
    //closed list front search
    unordered_map<pair<int, int>, Node, pair_hash> visited_front = unordered_map<pair<int, int>, Node, pair_hash>();
    //closed list backward search
    unordered_map<pair<int, int>, Node, pair_hash> visited_back = unordered_map<pair<int, int>, Node, pair_hash>();
    // two nodes to hold the solution, one for each direction.
    Node current_node, sol1, sol2;
    // row and col hold the matrix direction according to node actions.
    // g_cost is the actual weight of the path
    int row = 0, col = 0, expand_counter, g_cost = 0, sum = 0, min = 0;
    // f_cost is the heuristic cost of the node to goal node
    int f_cost = _heuristic_function(pair<int, int>(source[0], source[1]),pair<int, int>(goal[0],goal[1]));
    // source node to start a search from it to goal node, and from goal node to source node.
    Node sourceNode = Node(f_cost, g_cost, source[0], source[1], 0);
    // inserts source node coordinates to its path
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    // the heuristic function is symmetric so source to goal is the same as goal to source
    // init goal node with its cost as we are actually searching for the goal node and we need to consider its wight
    Node goalNode = Node(f_cost, array[goal[0]][goal[1]], goal[0], goal[1], 0);
    // inserts goal node coordinates to its path
    goalNode.insertElementToPath(pair<int, int>(goalNode.getRow(), goalNode.getCol()));
    // init front and back priority queues with source and goal nodes.
    frontier_front.insert(sourceNode);
    frontier_back.insert(goalNode);
    while (!frontier_front.empty() && !frontier_back.empty()) {
        //check whether node v is at the top of both frontier_front and frontier_back
        if (*frontier_front.begin() == *frontier_back.begin()) {
            break;
        }
        if (!frontier_front.empty()) {
            // get the node with the smallest f_cost, tie breaking is done by minimal h(n)
            current_node = *frontier_front.begin();
            frontier_front.erase(frontier_front.begin());
            // mark that node as visited in the front search
            visited_front[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
            // expand counter is used to detect cutoffs and recored the for stats.
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
        //check again as it may happen between the forward step to the backward step.
        if (*frontier_front.begin() == *frontier_back.begin()) {
            // node v is at the top of both frontier_front and frontier_back
            break;
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

    for (const auto &element : frontier_front) {
        auto found = std::find(frontier_back.begin(), frontier_back.end(), element);
        if (found != frontier_back.end()) {
            sum = element.getActualCost() + found->getActualCost();
            if (!min || sum < min) {
                min = sum;
                sol1 = element;
                sol2 = (*found);
            }
        }

    }
    for (const auto &item: visited_front) {
        auto found = std::find(visited_back.begin(), visited_back.end(), item);
        if (found != visited_back.end()) {
            sum = item.second.getActualCost() + found->second.getActualCost();
            if (!min || sum < min) {
                min = sum;
                sol1 = item.second;
                sol2 = (*found).second;
            }
        }

    }
    for (auto node = sol2.getPathTilNow().rbegin() + 1; node != sol2.getPathTilNow().rend(); ++node) {
        sol1.insertElementToPath(*node);
        sol1.setDepth(sol1.getDepth() + 1);
    }
    sol1.setActualCost(sol1.getActualCost() + sol2.getActualCost() - array[sol1.getRow()][sol1.getCol()]);
    std::cout << sol1.getActualCost() << std::endl;
    std::cout << sol1.getDepth() << std::endl;
    setExplored(visited_front.size() + visited_back.size());
    print_path(array, dimension, sol1);
    std::cout << std::endl;

    return 1;
}

BiDirectionalAStar &BiDirectionalAStar::getInstance() {
    static BiDirectionalAStar instance;
    return instance;
}


