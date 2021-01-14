

#include "UniformCostSearch.h"
#include "PairHashing.h"
#include <set>

using std::multiset;
using std::vector;
using std::unordered_map;

UniformCostSearch &UniformCostSearch::getInstance() {
    static UniformCostSearch instance;
    return instance;
}

int UniformCostSearch::run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) {
    // closed list
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> visited = unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash>();
    // open list
    multiset<shared_ptr<Node>, lessCompNodePointers> open_list = multiset<shared_ptr<Node>, lessCompNodePointers>();
    shared_ptr<vector<shared_ptr<Node>>> successors;
    shared_ptr<Node> current_node;
    // time out indicator
    bool time_out;
    int expand_counter;
    // init the source node
    shared_ptr<Node> sourceNode(new Node(0, 0, source[0], source[1], 0));
    // insert source node coordinates to the path
    sourceNode->insertElementToPath(pair<int, int>(sourceNode->getRow(), sourceNode->getCol()));
    // init goal node for goal reached comparison
    shared_ptr<Node> goalNode(new Node(0, 0, goal[0], goal[1], 0));
    // insert the source node to the open list
    open_list.insert(sourceNode);
    while (!open_list.empty()) {
        setCurrentTime(clock());
        current_node = *open_list.begin();
        open_list.erase(open_list.begin());
        if (visited.find(pair<int, int>(current_node->getRow(), current_node->getCol())) != visited.end())
            continue;
        time_out = (diff_clock(getCurrentTime(), getStartTime()) >= time_limit);
        ++getInstance()._expanded;
        if (*current_node == *goalNode || time_out) {
            update_cutoffs(current_node->getDepth());
            if (!time_out) {
                // reached to goal state
                setEndStatus(true);
            }
            generate_stats(*current_node);
            return 0;
        }
        visited[pair<int, int>(current_node->getRow(), current_node->getCol())] = current_node;
        expand_counter = 0;
        successors = current_node->successors(array, dimension);
        for (const auto &successor:*successors) {
            expand_counter++;
            auto visited_iterator = visited.find(pair<int, int>(successor->getRow(), successor->getCol()));
            // if the node is not visited and not in the open list, add it to open list.
            if (visited_iterator == visited.end())
                open_list.insert(successor);
            else if (visited_iterator != visited.end() &&
                     visited_iterator->second->getActualCost() > successor->getActualCost()) {
                visited.erase(visited_iterator);
                open_list.insert(successor);
            } else expand_counter--;
        }
        if (!expand_counter) {
            update_cutoffs(current_node->getDepth());
        }
    }
    generate_stats(*sourceNode);
    return 1;
}
