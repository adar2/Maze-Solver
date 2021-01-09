
#include "IDAStarSearch.h"
#include <limits>

using std::pair;

IDAStarSearch &IDAStarSearch::getInstance() {
    static IDAStarSearch instance;
    return instance;
}

int IDAStarSearch::run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) {
    shared_ptr<Node> found;
    pair<shared_ptr<Node>, double> results;
    // initialize f_limit to the heuristic value of sourceNode
    double f_limit = _heuristic_function(pair<int, int>(source[0], source[1]), pair<int, int>(goal[0], goal[1]));
    double old_f_limit;
    double  min_jump = 2;
    // try to improve performance by increase f_limit by at least some constant
    sumNodeHeuristic(f_limit);
    shared_ptr<Node> sourceNode  (new Node(f_limit, 0, source[0], source[1], 0));
    sourceNode->insertElementToPath(pair<int, int>(source[0], source[1]));
    shared_ptr<Node> goalNode  (new Node(0, 0, goal[0], goal[1], 0));
    while (true) {
        old_f_limit = f_limit;
        results = DFS_CONTOUR(array, dimension, sourceNode, goalNode, f_limit, time_limit);
        found = results.first;
        f_limit = results.second;
        if (found != nullptr) {
            setEndStatus(true);
//            print_path(array, dimension, *found);
            generate_stats(*found);
            return 0;
        }
        if (f_limit == std::numeric_limits<double>::infinity() || diff_clock(getCurrentTime(), getStartTime()) >= time_limit) {
            //failed or timeout
            generate_stats(*sourceNode);
            return 1;
        }
        // try to improve performance by limit f_limit steps by at least 2.
        if(f_limit - old_f_limit < min_jump)
            f_limit = old_f_limit + min_jump;
    }
}

pair<shared_ptr<Node>, double>
IDAStarSearch::DFS_CONTOUR(double **array, int dimension, const shared_ptr<Node>&current_node, const shared_ptr<Node>&goal, double f_limit, float time_limit) {
    shared_ptr<Node>found;
    pair<shared_ptr<Node>, double> results_pair;
    shared_ptr<vector<shared_ptr<Node>>> successors;
    double new_f,h_cost;
    double next_f = std::numeric_limits<double>::infinity();
    double current_node_f = current_node->getEvaluationCost();
    setCurrentTime(clock());
    if (current_node_f > f_limit || diff_clock(getCurrentTime(), getStartTime()) >= time_limit){
        update_cutoffs(current_node->getDepth());
        return {nullptr, current_node_f};
    }
    if (*current_node == *goal){
        update_cutoffs(current_node->getDepth());
        return {current_node, current_node_f};
    }
    getInstance()._expanded++;
    successors = current_node->successors(array, dimension);
    for (const auto& node : *successors) {
        // avoid expanding nodes that are on current node path
        // optimization to avoid circles and going backward on path
        if (std::find(current_node->getPathTilNow().begin(), current_node->getPathTilNow().end(),pair<int, int>(node->getRow(), node->getCol())) != current_node->getPathTilNow().end()){
            continue;
        }
        h_cost = _heuristic_function(pair<int, int>(node->getRow(), node->getCol()), pair<int, int>(goal->getRow(), goal->getCol()));
        node->setEvaluationCost(node->getActualCost() + h_cost);
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
