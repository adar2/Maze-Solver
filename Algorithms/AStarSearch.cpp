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
    unordered_map<pair<int, int>, Node, pair_hash> visited = unordered_map<pair<int, int>, Node, pair_hash>();
    multiset<Node> openList = multiset<Node>();
    Node current_node;
    bool time_out = false;
    int row = 0, col = 0, expand_counter, f_cost = _heuristic_function(pair<int, int>(source[0], source[1]),
                                                                       pair<int, int>(goal[0], goal[1])), g_cost = 0;
    Node sourceNode = Node(f_cost, g_cost, source[0], source[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, 0, goal[0], goal[1], 0);
    openList.insert(sourceNode);
    while (!openList.empty()) {
        setCurrentTime(clock());
        current_node = *openList.begin();
        openList.erase(openList.begin());
        time_out = diff_clock(getCurrentTime(), getStartTime()) >= time_limit;
        if (current_node == goalNode || time_out) {
            setExplored(visited.size());
            if (time_out) {
                setEndStatus(false);
            } else {
                setEndStatus(true);
                double depth = current_node.getPathTilNow().size();
                setDN(depth / getExplored());
                setEbf(pow(getExplored(), pow(depth, -1)));
            }
            print_path(array, dimension, current_node);
            generate_stats(current_node);
            std::cout << "solution cost: " << current_node.getActualCost() << std::endl;
            std::cout << "solution depth: " << current_node.getDepth() << std::endl;
            return 0;
        }
        visited[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
        // zero the expand counter for the current node in order to check cutoffs
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
            auto open_list_iterator = std::find(openList.begin(), openList.end(), node);
            auto explored_iterator = visited.find(pair<int, int>(row, col));
            if (explored_iterator == visited.end() && open_list_iterator == openList.end())
                openList.insert(node);
            else if (open_list_iterator != openList.end() &&
                     open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                openList.erase(open_list_iterator);
                openList.insert(node);
            } else if (explored_iterator != visited.end() &&
                       explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                visited.erase(explored_iterator);
                openList.insert(node);
            } else expand_counter--;

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
    setEndStatus(false);
    return 1;
}


AStarSearch &AStarSearch::getInstance() {
    static AStarSearch instance;
    return instance;
}
