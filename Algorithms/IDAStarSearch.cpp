//
// Created by r00t on 12/15/20.
//

#include "IDAStarSearch.h"
#include <limits>

IDAStarSearch &IDAStarSearch::getInstance() {
    static IDAStarSearch instance;
    return instance;
}

int IDAStarSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    Node *found;
    std::tuple<Node *, int> results_tuple;
    // initialize f_limit to the heuristic value of root
    int f_limit = _heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1]));
    Node *root = new Node(f_limit, 0, source[0], source[1], 0);
    root->insertElementToPath(pair<int, int>(source[0], source[1]));
    Node *target = new Node(0, 0, goal[0], goal[1], 0);
    while (true) {
        results_tuple = DFS_CONTOUR(array, dimension, root, target, f_limit);
        found = std::get<0>(results_tuple);
        f_limit = std::get<1>(results_tuple);
        if (found != nullptr) {
            generate_stats(*found);
            delete found;
            delete root;
            delete target;
            return 0;
        }
        if (f_limit == std::numeric_limits<int>::max()) {

            //failed
            delete root;
            delete target;
            return 1;
        }
    }
}

std::tuple<Node *, int>
IDAStarSearch::DFS_CONTOUR(int **array, int dimension, Node *current_node, Node *goal, int f_limit) {
    Node *found, *node;
    std::tuple<Node *, int> results_tuple;
    int row = 0, col = 0, new_f = 0, expand_counter = 0;
    int next_f = std::numeric_limits<int>::max();
    int current_node_f = current_node->getHeuristicCost();
    if (current_node_f > f_limit) return {nullptr, current_node_f};
    if (*current_node == *goal) return {current_node, current_node_f};
    for (int i = 0; i < ACTIONS_SIZE; ++i) {
        switch (actions(i)) {
            case U:
                row = current_node->getRow() - 1;
                col = current_node->getCol();
                break;
            case RU:
                row = current_node->getRow() - 1;
                col = current_node->getCol() + 1;
                break;
            case R:
                row = current_node->getRow();
                col = current_node->getCol() + 1;
                break;
            case RD:
                row = current_node->getRow() + 1;
                col = current_node->getCol() + 1;
                break;
            case D:
                row = current_node->getRow() + 1;
                col = current_node->getCol();
                break;
            case LD:
                row = current_node->getRow() + 1;
                col = current_node->getCol() - 1;
                break;
            case L:
                row = current_node->getRow();
                col = current_node->getCol() - 1;
                break;
            case LU:
                row = current_node->getRow() - 1;
                col = current_node->getCol() - 1;
                break;
        }
        if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0)
            continue;
        expand_counter++;
        int g_cost = array[row][col] + current_node->getActualCost();
        int f_cost =
                g_cost + _heuristic_function(pair<int, int>(row, col), pair<int, int>(goal->getRow(), goal->getCol()));
        node = new Node(f_cost, g_cost, row, col, current_node->getDepth() + 1);
        node->setPathTilNow(current_node->getPathTilNow());
        node->insertElementToPath(pair<int, int>(row, col));
        results_tuple = DFS_CONTOUR(array, dimension, node, goal, f_limit);
        found = std::get<0>(results_tuple);
        new_f = std::get<1>(results_tuple);
        if (found != nullptr) {
            if (node != found)
                // delete the node created in this iteration except for the solution node.
                delete node;
            return {found, f_limit};
        }
        next_f = next_f < new_f ? next_f : new_f;
        delete node;
        if (!expand_counter) {
            // cut off occurrence
            if (getInstance().getMin() == 0 || getInstance().getMin() > current_node->getDepth())
                getInstance().setMin(current_node->getDepth());
            if (getInstance().getMax() == 0 || getInstance().getMax() < current_node->getDepth())
                getInstance().setMax(current_node->getDepth());
            getInstance().addCutoffToSum(current_node->getDepth());
        }
    }

    return {nullptr, next_f};
}
