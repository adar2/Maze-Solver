//
// Created by r00t on 12/14/20.
//

#include "AStarSearch.h"


using std::unordered_map;
using std::cout;
using std::endl;


int AStarSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    unordered_map<pair<int, int>, bool, pair_hash> explored = unordered_map<pair<int, int>, bool, pair_hash>();
    vector<Node> openList = vector<Node>();
    Node sourceNode = Node(heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1])),
                           0, source[0], source[1], 0);
    sourceNode.insertElementToPath(pair<int, int>(sourceNode.getRow(), sourceNode.getCol()));
    Node goalNode = Node(0, 0, goal[0], goal[1], 0);
    Node current_node;
    int row = 0, col = 0;
    openList.push_back(sourceNode);
    while (!openList.empty()) {
        setCurrentTime(time(nullptr));
        if (difftime(getCurrentTime(), getStartTime()) >= time_limit)
            return 3;//time out
        sort(openList.begin(), openList.end(), std::greater<Node>());
        current_node = openList.back();
        openList.pop_back();
        if (current_node == goalNode) {
            double depth = current_node.getPathTilNow().size();
            setExplored(explored.size());
            setDN(depth / getExplored());
            setEbf(pow(getExplored(), pow(depth, -1)));
            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    pair<int, int> p = pair<int, int>(i, j);
                    if (count(current_node.getPathTilNow().begin(), current_node.getPathTilNow().end(), p))
                        cout << "\033[1;31m" << '(' << array[i][j] << ")," << "\033[0m";
                    else
                        cout << '(' << array[i][j] << "),";
                }
                cout << endl;
            }
            generate_stats();
            return 0;
        }
        explored[pair<int, int>(current_node.getRow(), current_node.getCol())] = true;
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
            int g_cost = array[row][col] + current_node.getActualCost();
            int f_cost = g_cost + heuristic_function(pair<int, int>(row, col),
                                                     pair<int, int>(goalNode.getRow(), goalNode.getCol()));
            Node node = Node(f_cost, g_cost, row, col, current_node.getDepth() + 1);
            node.setPathTilNow(current_node.getPathTilNow());
            node.insertElementToPath(pair<int, int>(row, col));
            if (!explored[pair<int, int>(row, col)] && !count(openList.begin(), openList.end(), node))
                openList.push_back(node);
            else if (find(openList.begin(), openList.end(), node) != openList.end() &&
                     find(openList.begin(), openList.end(), node)->getHeuristicCost() > node.getHeuristicCost()) {
                openList.erase(find(openList.begin(), openList.end(), node));
                openList.push_back(node);
            }
        }
    }
    return 1;
}


AStarSearch &AStarSearch::getInstance() {
    static AStarSearch instance;
    return instance;
}
