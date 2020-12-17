//
// Created by r00t on 12/14/20.
//

#include "AStarSearch.h"
#include <vector>
#include <set>

using std::vector;
using std::unordered_map;
using std::cout;
using std::endl;

multiset<Node>::iterator set_find(multiset<Node>::iterator start,multiset<Node>::iterator end,Node node){
    while(start!=end){
        if(*start == node)
            return start;
        ++start;
    }
    return end;
}


int AStarSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    unordered_map<pair<int, int>, Node, pair_hash> explored = unordered_map<pair<int, int>, Node, pair_hash>();
    vector<Node> openList = vector<Node>();
    Node sourceNode = Node(_heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1])),
                           0, source[0], source[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, 0, goal[0], goal[1], 0);
    Node current_node;
    int row = 0, col = 0, expand_counter;
    openList.push_back(sourceNode);
    while (!openList.empty()) {
        setCurrentTime(time(nullptr));
//        if (difftime(getCurrentTime(), getStartTime()) >= time_limit)
//            return 3;//time out
        std::sort(openList.begin(),openList.end(),std::greater<Node>());
        current_node = openList.back();
        openList.pop_back();
        if (current_node == goalNode) {
            setCurrentTime(time(nullptr));
            double depth = current_node.getPathTilNow().size();
            setExplored(explored.size());
            setDN(depth / getExplored());
            setEbf(pow(getExplored(), pow(depth, -1)));
            generate_stats(current_node);
            return 0;
        }
        explored[pair<int, int>(current_node.getRow(), current_node.getCol())] = current_node;
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
            int g_cost = array[row][col] + current_node.getActualCost();
            int f_cost = g_cost + _heuristic_function(pair<int, int>(row, col),
                                                      pair<int, int>(goalNode.getRow(), goalNode.getCol()));
            Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(row, col));
            auto open_list_iterator = std::find(openList.begin(),openList.end(),node);
            auto explored_iterator = explored.find(pair<int, int>(row, col));
            if (explored_iterator == explored.end() && open_list_iterator == openList.end())
                openList.push_back(node);
            else if (open_list_iterator != openList.end() &&
                     open_list_iterator->getHeuristicCost() > node.getHeuristicCost()) {
                openList.erase(open_list_iterator);
                openList.push_back(node);
            } else if (explored_iterator != explored.end() &&
                       explored_iterator->second.getHeuristicCost() > node.getHeuristicCost()) {
                explored.erase(explored_iterator);
                openList.push_back(node);
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
    return 1;
}


AStarSearch &AStarSearch::getInstance() {
    static AStarSearch instance;
    return instance;
}
