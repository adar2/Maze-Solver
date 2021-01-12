
#include "AStarSearch.h"
#include <vector>
#include <set>

using std::multiset;
using std::vector;
using std::unordered_map;
using std::cout;
using std::endl;


int AStarSearch::run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) {
    // closed list
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> visited = unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash>();
    // open list
    multiset<shared_ptr<Node>, lessCompNodePointers> frontier = multiset<shared_ptr<Node>, lessCompNodePointers>();
    shared_ptr<vector<shared_ptr<Node>>> successors;
    shared_ptr<Node> current_node;
    // time out indicator
    bool time_out;
    int expand_counter;
    // calc heuristic value of the source node.
    double h_cost = _heuristic_function(pair<int, int>(source[0], source[1]),
                                                     pair<int, int>(goal[0], goal[1]));
    sumNodeHeuristic(h_cost);
    // init the source node
    shared_ptr<Node> sourceNode(new Node(h_cost, 0, source[0], source[1], 0));
    // insert source node coordinates to the path
    sourceNode->insertElementToPath(pair<int, int>(sourceNode->getRow(), sourceNode->getCol()));
    // init goal node for goal reached comparison
    shared_ptr<Node> goalNode(new Node(0, 0, goal[0], goal[1], 0));
    // insert the source node to the open list
    frontier.insert(sourceNode);
    while (!frontier.empty()) {
        // update current time for time out checking
        setCurrentTime(clock());
        // get the node with the smallest h_cost value
        current_node = *frontier.begin();
        frontier.erase(frontier.begin());
        time_out = (diff_clock(getCurrentTime(), getStartTime()) >= time_limit);
        ++getInstance()._expanded;
        // if goal node reached or time out stop the search
        if (*current_node == *goalNode || time_out) {
            update_cutoffs(current_node->getDepth());
            if (!time_out) {
                setEndStatus(true);
            }
            generate_stats(*current_node);
            return 0;
        }
        // mark current node as visited
        visited[pair<int, int>(current_node->getRow(), current_node->getCol())] = current_node;
        // zero the expand counter for the current node in order to check cutoffs
        expand_counter = 0;
        // loop current node successors.
        successors = current_node->successors(array, dimension);
        for (const auto &successor :*successors) {
            expand_counter++;
            h_cost = _heuristic_function(pair<int, int>(successor->getRow(), successor->getCol()), pair<int, int>(goalNode->getRow(), goalNode->getCol()));
            successor->setEvaluationCost(successor->getActualCost() + h_cost);
            // sum successor h value for heuristics statistics.
            sumNodeHeuristic(h_cost);
            // use std::find_if with lambada function as node pointers comparator.
            auto open_list_iterator = std::find_if(frontier.begin(), frontier.end(),
                                                   [&successor](const shared_ptr<Node> &node) {
                                                       return *node == *successor;
                                                   });
            auto visited_iterator = visited.find(pair<int, int>(successor->getRow(), successor->getCol()));
            // if the node is not visited and not in the open list, add it to open list.
            if (visited_iterator == visited.end() && open_list_iterator == frontier.end())
                frontier.insert(successor);
                // if the node is present in the open list but have a higher h_cost , remove it and insert the better one.
            else if (open_list_iterator != frontier.end() &&
                     (*open_list_iterator)->getEvaluationCost() > successor->getEvaluationCost()) {
                frontier.erase(open_list_iterator);
                frontier.insert(successor);
            }
                // if the node has been visited with higher h_cost, remove it from visited and insert it to open list with better h_cost
            else if (visited_iterator != visited.end() &&
                     visited_iterator->second->getEvaluationCost() > successor->getEvaluationCost()) {
                visited.erase(visited_iterator);
                frontier.insert(successor);
            }
                // if we didnt got into one of the above cond, we didnt expended this node so decrement expend_counter.
            else expand_counter--;

        }
        if (!expand_counter) {
            // cut off occurrence
            update_cutoffs(current_node->getDepth());
        }
    }
    generate_stats(*current_node);
    return 1;
}


AStarSearch &AStarSearch::getInstance() {
    static AStarSearch instance;
    return instance;
}

void AStarSearch::generate_stats(const Node &current_node) {
    AbstractSearchAlgorithm::generate_stats(current_node);
    HeuristicSearch::generate_heuristic_stats();
}
