//
// Created by r00t on 12/14/20.
//

#include "AStarSearch.h"
#include <vector>
#include <set>

using std::multiset;
using std::vector;
using std::unordered_map;
using std::cout;
using std::endl;


int AStarSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    // closed list
    unordered_map<pair<int, int>, Node, pair_hash> visited = unordered_map<pair<int, int>, Node, pair_hash>();
    // open list
    multiset<Node> openList = multiset<Node>();
    Node current_node;
    // time out indicator
    bool time_out = false;
    // row and col hold the matrix direction according to node actions.
    // g_cost is the actual weight of the path
    int row = 0, col = 0, expand_counter, f_cost = _heuristic_function(pair<int, int>(source[0], source[1]),
                                                                       pair<int, int>(goal[0], goal[1])), g_cost = 0;
    // init the source node
    Node sourceNode = Node(f_cost, g_cost, source[0], source[1], 0);
    // insert source node coordinates to the path
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    // init goal node for goal reached comparison
    Node goalNode = Node(0, 0, goal[0], goal[1], 0);
    // insert the source node to the open list
    openList.insert(sourceNode);
    while (!openList.empty()) {
        // update current time for time out checking
        setCurrentTime(clock());
        // get the node with the smallest f_cost value
        current_node = *openList.begin();
        openList.erase(openList.begin());
        time_out = diff_clock(getCurrentTime(), getStartTime()) >= time_limit;
        // if goal node reached or time out stop the search
        if (current_node == goalNode || time_out) {
            setExplored(visited.size());
            if (!time_out) {
                setEndStatus(true);
                double depth = current_node.getPathTilNow().size();
                setDN(depth / getExplored());
                setEbf(pow(getExplored(), pow(depth, -1)));
            }
//            print_path(array, dimension, current_node);
            generate_stats(current_node);
            return 0;
        }
        // mark current node as visited
        visited[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
        // zero the expand counter for the current node in order to check cutoffs
        expand_counter = 0;
        // loop over node actions
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
            // if its index is out of range or its a wall, represented by -1 in the matrix continue.
            if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0)
                continue;
            expand_counter++;
            g_cost = array[row][col] + current_node.getActualCost();
            f_cost = g_cost + _heuristic_function(pair<int, int>(row, col),
                                                  pair<int, int>(goalNode.getRow(), goalNode.getCol()));
            Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
            // copy predecessor path
            node.setPathTilNow(current_node.getPathTilNow());
            // insert new node coordinates to the path
            node.insertElementToPath(pair<int, int>(row, col));
            // use std::find as it uses operator== for comparison
            auto open_list_iterator = std::find(openList.begin(), openList.end(), node);
            auto explored_iterator = visited.find(pair<int, int>(row, col));
            // if the node is not visited and not in the open list, add it to open list.
            if (explored_iterator == visited.end() && open_list_iterator == openList.end())
                openList.insert(node);
            // if the node is present in the open list but have a higher f_cost , remove it and insert the better one.
            else if (open_list_iterator != openList.end() &&
                     open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                openList.erase(open_list_iterator);
                openList.insert(node);
            }
            // if the node has been visited with higher f_cost, remove it from visited and insert it to open list with better f_cost
            else if (explored_iterator != visited.end() &&
                       explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                visited.erase(explored_iterator);
                openList.insert(node);
            }
            // if we didnt get into one of the above cond, we didnt expended this node so decrement expend_counter.
            else expand_counter--;

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
    // no solution found
    setExplored(visited.size());
    generate_stats(current_node);
    return 1;
}


AStarSearch &AStarSearch::getInstance() {
    static AStarSearch instance;
    return instance;
}
