//
// Created by r00t on 12/13/20.
//

#include "IterativeDeepeningSearch.h"


using std::unordered_map;
using std::stack;
using std::cout;
using std::endl;

Node *IterativeDeepeningSearch::DLS(int **array, int dimension, Node *root, Node *goal, int limit) {
    stack<Node *> frontier = stack<Node *>();
    unordered_map<pair<int, int>, bool, pair_hash> explored = unordered_map<pair<int, int>, bool, pair_hash>();
    Node *current_node, *node;
    int row = 0, col = 0;
    frontier.push(root);
    while (!frontier.empty()) {
        current_node = frontier.top();
        frontier.pop();
        if (*current_node == *goal) {
            while (!frontier.empty()) {
                node = frontier.top();
                frontier.pop();
                delete node;
            }
            return current_node;
        }
        if (current_node->getDepth() < limit) {
            explored[pair<int, int>(current_node->getRow(), current_node->getCol())] = true;
            for (int i = ACTIONS_SIZE - 1; i >= 0; --i) {
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
                if (explored[pair<int, int>(row, col)])
                    continue;
                int cost = array[row][col] + current_node->getActualCost();
                node = new Node(cost, cost, row, col, current_node->getDepth() + 1);
                node->setPathTilNow(current_node->getPathTilNow());
                node->insertElementToPath(pair<int, int>(row, col));
                frontier.push(node);
            }
        }

        // compare pointers by addresses.
        if (current_node != root) {
            delete current_node;
        }
    }
    return nullptr;
}


int IterativeDeepeningSearch::run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) {
    Node *found;
    Node *root = new Node(0, 0, source[0], source[1], 0);
    root->insertElementToPath(pair<int, int>(source[0], source[1]));
    Node *target = new Node(0, 0, goal[0], goal[1], 0);
    for (int i = 0; i < pow(dimension, 2); ++i) {
        cout << "current penetration level: " << i << endl;
        found = DLS(array, dimension, root, target, i);
        if (found != nullptr) {
            // found the node
            delete root;
            delete target;
            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    pair<int, int> p = pair<int, int>(i, j);
                    if (count(found->getPathTilNow().begin(), found->getPathTilNow().end(), p))
                        cout << "\033[1;31m" << '(' << array[i][j] << ")," << "\033[0m";
                    else
                        cout << '(' << array[i][j] << "),";
                }
                cout << endl;
            }
            delete found;
            return 0;// return success.
        }
    }
    delete root;
    delete target;
    return 1;//return failure.
}

IterativeDeepeningSearch &IterativeDeepeningSearch::getInstance() {
    static IterativeDeepeningSearch instance;
    return instance;
}




