//
// Created by r00t on 1/2/21.
//

#include "UniformCostSearch.h"
#include <set>
#include "utils.h"

using std::multiset;
using std::vector;
using std::unordered_map;

UniformCostSearch &UniformCostSearch::getInstance() {
    static UniformCostSearch instance;
    return instance;
}

int UniformCostSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    // closed list
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> visited = unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash>();
    // open list
    multiset<shared_ptr<Node>,lessCompNodePointers> openList = multiset<shared_ptr<Node>,lessCompNodePointers>();
    shared_ptr<vector<shared_ptr<Node>>> successors;
    shared_ptr<Node> current_node;
    // time out indicator
    bool time_out;
    int expand_counter;
    // init the source node
    shared_ptr<Node> sourceNode (new Node(0, 0, source[0], source[1], 0));
    // insert source node coordinates to the path
    sourceNode->insertElementToPath(pair<int, int>(sourceNode->getRow(), sourceNode->getCol()));
    // init goal node for goal reached comparison
    shared_ptr<Node> goalNode (new Node(0, 0, goal[0], goal[1], 0));
    // insert the source node to the open list
    openList.insert(sourceNode);
    while (!openList.empty()) {
        setCurrentTime(clock());
        current_node = *openList.begin();
        openList.erase(openList.begin());
        time_out = (diff_clock(getCurrentTime(), getStartTime()) >= time_limit);
        if (*current_node == *goalNode || time_out) {
            setExplored(visited.size());
            if(!time_out){
                setEndStatus(true);
                int depth = current_node->getPathTilNow().size();
                setDN(double(depth) / getExplored());
                calcEBF(depth);
                // reached to goal state
            }
            generate_stats(*current_node);
            return 0;
        }
        visited[pair<int, int>(current_node->getRow(), current_node->getCol())] = current_node;
        expand_counter = 0;
        successors = current_node->successors(array,dimension);
        for(const auto& successor:*successors){
            expand_counter++;
            auto open_list_iterator = std::find_if(openList.begin(), openList.end(), [&successor](const shared_ptr<Node>& node){return *node == *successor;});
            auto explored_iterator = visited.find(pair<int, int>(successor->getRow(), successor->getCol()));
            // if the node is not visited and not in the open list, add it to open list.
            if (explored_iterator == visited.end() && open_list_iterator == openList.end())
                openList.insert(successor);
            else if (open_list_iterator != openList.end() &&
                     (*open_list_iterator)->getActualCost() > successor->getActualCost()) {
                openList.erase(open_list_iterator);
                openList.insert(successor);
            } else expand_counter--;
        }
        if(!expand_counter){
            update_cutoffs(current_node->getDepth());
        }
    }
    return 1;
}
