//
// Created by r00t on 12/15/20.
//

#include "BiDirectionalAStar.h"
#include <set>

using std::multiset;
using std::unordered_map;

int BiDirectionalAStar::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    //open list front search
    multiset<shared_ptr<Node>,lessCompNodePointers> frontier_front = multiset<shared_ptr<Node>,lessCompNodePointers>();
    //open list backward search
    multiset<shared_ptr<Node>,lessCompNodePointers> frontier_back = multiset<shared_ptr<Node>,lessCompNodePointers>();
    //closed list front search
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash>  visited_front = unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> ();
    //closed list backward search
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash>  visited_back = unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> ();
    // two nodes to hold the solution, one for each direction.
    shared_ptr<Node> current_node, sol1, sol2;
    shared_ptr<vector<shared_ptr<Node>>> successors;
    // row and col hold the matrix direction according to node actions.
    // g_cost is the actual weight of the path
    int expand_counter, g_cost = 0, sum, min = 0;
    // h_cost is the heuristic cost of the node to goal node
    int h_cost = _heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1]));
    // source node to start a search from it to goal node, and from goal node to source node.
    shared_ptr<Node> sourceNode (new Node(h_cost, g_cost, source[0], source[1], 0));
    // inserts source node coordinates to its path
    sourceNode->insertElementToPath(pair<int, int>(sourceNode->getRow(), sourceNode->getCol()));
    // the heuristic function is symmetric so source to goal is the same as goal to source
    // init goal node with its cost as we are actually searching for the goal node and we need to consider its weight.
    shared_ptr<Node> goalNode (new Node(h_cost, array[goal[0]][goal[1]], goal[0], goal[1], 0));
    // inserts goal node coordinates to its path
    goalNode->insertElementToPath(pair<int, int>(goalNode->getRow(), goalNode->getCol()));
    // init front and back priority queues with source and goal nodes.
    frontier_front.insert(sourceNode);
    frontier_back.insert(goalNode);
    while (!frontier_front.empty() && !frontier_back.empty()) {
        setCurrentTime(clock());
        //check whether node v is at the top of both frontier_front and frontier_back
        if (*(*frontier_front.begin()) == *(*frontier_back.begin())) {
            setEndStatus(true);
            break;
        }
        // time out
        if(diff_clock(getCurrentTime(),getStartTime()) >= time_limit){
            setExplored(frontier_front.size() + frontier_back.size());
            generate_stats(*current_node);
            return 1;
        }
        if (!frontier_front.empty()) {
            // get the node with the smallest h_cost, tie breaking is done by minimal h(n)
            current_node = *frontier_front.begin();
            frontier_front.erase(frontier_front.begin());
            // mark that node as visited in the front search
            visited_front[pair<int, int>(current_node->getRow(), current_node->getCol())] = current_node;
            // expand counter is used to detect cutoffs and record the for stats.
            expand_counter = 0;
            // loop over node actions
            successors = current_node->successors(array,dimension);
            for(const auto& successor : *successors){
                expand_counter++;
                h_cost = _heuristic_function(pair<int, int>(successor->getRow(), successor->getCol()), pair<int, int>(goalNode->getRow(), goalNode->getCol()));
                successor->setHeuristicCost(successor->getActualCost() + h_cost);
                sumNodeHeuristic(h_cost);
                // use std::find_if with lambada function as node pointers comparator.
                auto open_list_iterator = std::find_if(frontier_front.begin(), frontier_front.end(), [&successor](const shared_ptr<Node>& node){return *node == *successor;});
                auto explored_iterator = visited_front.find(pair<int, int>(successor->getRow(), successor->getCol()));
                // if the node is not visited and not in the open list, add it to open list.
                if (explored_iterator == visited_front.end() && open_list_iterator == frontier_front.end())
                    frontier_front.insert(successor);
                // if the node is present in the open list but have a higher h_cost , remove it and insert the better one.
                else if (open_list_iterator != frontier_front.end() &&
                         (*open_list_iterator)->getHeuristicCost() > successor->getHeuristicCost()) {
                    frontier_front.erase(open_list_iterator);
                    frontier_front.insert(successor);
                // if the node has been visited with higher h_cost, remove it from visited and insert it to open list with better h_cost
                } else if (explored_iterator != visited_front.end() &&
                           explored_iterator->second->getHeuristicCost() > successor->getHeuristicCost()) {
                    visited_front.erase(explored_iterator);
                    frontier_front.insert(successor);
                // if we didnt get into one of the above cond, we didnt expended this node so decrement expend_counter.
                }else expand_counter--;
            }
            if (!expand_counter) {
                // cut off occurrence
                update_cutoffs(current_node->getDepth());
            }
        }
        //check again as it may happen between the forward step to the backward step.
        //check whether node v is at the top of both frontier_front and frontier_back
        if (*(*frontier_front.begin()) == *(*frontier_back.begin())) {
            setEndStatus(true);
            break;
        }
        // time out
        if(diff_clock(getCurrentTime(),getStartTime()) >= time_limit){
            setExplored(frontier_front.size() + frontier_back.size());
            generate_stats(*current_node);
            return 1;
        }
        if (!frontier_back.empty()) {
            // get the node with the smallest h_cost, tie breaking is done by minimal h(n)
            current_node = *frontier_back.begin();
            frontier_back.erase(frontier_back.begin());
            // mark that node as visited in the front search
            visited_back[pair<int, int>(current_node->getRow(), current_node->getCol())] = current_node;
            // expand counter is used to detect cutoffs and record the for stats.
            expand_counter = 0;
            // loop over node actions
            successors = current_node->successors(array,dimension);
            for(const auto& successor: *successors ){
                expand_counter++;
                h_cost = _heuristic_function(pair<int, int>(successor->getRow(), successor->getCol()), pair<int, int>(sourceNode->getRow(), sourceNode->getCol()));
                successor->setHeuristicCost(successor->getActualCost() + h_cost);
                sumNodeHeuristic(h_cost);
                // use std::find_if with lambada function as node pointers comparator.
                auto open_list_iterator = std::find_if(frontier_back.begin(), frontier_back.end(), [&successor](const shared_ptr<Node>& node){return *node == *successor;});
                auto explored_iterator = visited_back.find(pair<int, int>(successor->getRow(), successor->getCol()));
                // if the node is not visited and not in the open list, add it to open list.
                if (explored_iterator == visited_back.end() && open_list_iterator == frontier_back.end())
                    frontier_back.insert(successor);
                // if the node is present in the open list but have a higher h_cost , remove it and insert the better one.
                else if (open_list_iterator != frontier_back.end() &&
                         (*open_list_iterator)->getHeuristicCost() > successor->getHeuristicCost()) {
                    frontier_back.erase(open_list_iterator);
                    frontier_back.insert(successor);

                }
                // if the node has been visited with higher h_cost, remove it from visited and insert it to open list with better h_cost
                else if (explored_iterator != visited_back.end() &&
                           explored_iterator->second->getHeuristicCost() > successor->getHeuristicCost()) {
                    visited_back.erase(explored_iterator);
                    frontier_back.insert(successor);

                }
                // if we didnt get into one of the above cond, we didnt expended this node so decrement expend_counter.
                else expand_counter--;
            }
            if (!expand_counter) {
                // cut off occurrence
                update_cutoffs(current_node->getDepth());
            }
        }
    }
    // post search phase find the optimal cost
    setExplored(visited_front.size() + visited_back.size());
    if(getEndStatus()) {
        for (const auto &element : frontier_front) {
            auto found = std::find_if(frontier_back.begin(), frontier_back.end(),
                                      [&element](const shared_ptr<Node> &node) { return *node == *element; });
            if (found != frontier_back.end()) {
                sum = element->getActualCost() + (*found)->getActualCost();
                if (!min || sum < min) {
                    min = sum;
                    sol1 = element;
                    sol2 = (*found);
                }
            }

        }
        for (const auto &element: visited_front) {
            auto found = std::find_if(visited_back.begin(), visited_back.end(),
                                      [&element](const pair<pair<int, int>, shared_ptr<Node>> &pair) {
                                          return *element.second == *pair.second;
                                      });
            if (found != visited_back.end()) {
                sum = element.second->getActualCost() + found->second->getActualCost();
                if (!min || sum < min) {
                    min = sum;
                    sol1 = element.second;
                    sol2 = (*found).second;
                }
            }

        }
        for (auto node = sol2->getPathTilNow().rbegin() + 1; node != sol2->getPathTilNow().rend(); ++node) {
            sol1->insertElementToPath(*node);
            sol1->setDepth(sol1->getDepth() + 1);
        }
        sol1->setActualCost(sol1->getActualCost() + sol2->getActualCost() - array[sol1->getRow()][sol1->getCol()]);
        double sol_depth = sol1->getPathTilNow().size();
        setDN(sol_depth/getExplored());
        calcEBF(sol_depth);
        print_path(array, dimension, *sol1);
        std::cout << std::endl;
        generate_stats(*sol1);
        return 0;
    }
    generate_stats(*sourceNode);

    return 1;
}

BiDirectionalAStar &BiDirectionalAStar::getInstance() {
    static BiDirectionalAStar instance;
    return instance;
}

void BiDirectionalAStar::generate_stats(const Node &current_node) {
    AlgorithmStatistics::generate_stats(current_node);
    HeuristicSearch::generate_heuristic_stats();
}


