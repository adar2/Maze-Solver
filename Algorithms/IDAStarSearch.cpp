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
    shared_ptr<Node> found;
    pair<shared_ptr<Node>, int> results_pair;
    // initialize f_limit to the heuristic value of root
    int f_limit = _heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1]));
    // try to improve performance by increase f_limit by at least some constant
    sumNodeHeuristic(f_limit);
    shared_ptr<Node> root  (new Node(f_limit, 0, source[0], source[1], 0));
    root->insertElementToPath(pair<int, int>(source[0], source[1]));
    shared_ptr<Node> target  (new Node(0, 0, goal[0], goal[1], 0));
    while (true) {
        results_pair = DFS_CONTOUR(array, dimension, root, target, f_limit, time_limit);
        found = results_pair.first;
        f_limit = results_pair.second;
        if (found != nullptr) {
            setEndStatus(true);
            int depth = found->getPathTilNow().size();
            setDN(double(depth) / getExplored());
            calcEbf(depth);
//            print_path(array, dimension, *found);
            generate_stats(*found);
            return 0;
        }
        if (f_limit == std::numeric_limits<int>::max() || diff_clock(getCurrentTime(), getStartTime()) >= time_limit) {
            generate_stats(*root);
            //failed or timeout
            return 1;
        }
    }
}

pair<shared_ptr<Node>, int>
IDAStarSearch::DFS_CONTOUR(int **array, int dimension, const shared_ptr<Node>&current_node, const shared_ptr<Node>&goal, int f_limit, float time_limit) {
    shared_ptr<Node>found;
    pair<shared_ptr<Node>, int> results_pair;
    shared_ptr<vector<shared_ptr<Node>>> successors;
    int new_f,h_cost;
    int next_f = std::numeric_limits<int>::max();
    int current_node_f = current_node->getHeuristicCost();
    setCurrentTime(clock());
    if (current_node_f > f_limit || diff_clock(getCurrentTime(), getStartTime()) >= time_limit){
        update_cutoffs(current_node->getDepth());
        return {nullptr, current_node_f};
    }
    if (*current_node == *goal){
        update_cutoffs(current_node->getDepth());
        return {current_node, current_node_f};
    }
    getInstance()._explored++;
    successors = current_node->successors(array, dimension);
    for (const auto& node : *successors) {
        if (std::find(current_node->getPathTilNow().begin(), current_node->getPathTilNow().end(),pair<int, int>(node->getRow(), node->getCol())) != current_node->getPathTilNow().end()){
            continue;
        }
        h_cost = _heuristic_function(pair<int, int>(node->getRow(), node->getCol()), pair<int, int>(goal->getRow(), goal->getCol()));
        node->setHeuristicCost(node->getActualCost() + h_cost);
        sumNodeHeuristic(h_cost);
        results_pair = DFS_CONTOUR(array, dimension, node, goal, f_limit, time_limit);
        found = results_pair.first;
        new_f = results_pair.second;
        if (found != nullptr) {
            return {found, f_limit};
        }
        next_f = next_f < new_f ? next_f : new_f;
    }

    return {nullptr, next_f};
}

void IDAStarSearch::generate_stats(const Node &current_node) {
    AlgorithmStatistics::generate_stats(current_node);
    HeuristicSearch::generate_heuristic_stats();
}
