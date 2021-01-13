

#include "HeuristicSearch.h"
#include <iostream>

void HeuristicSearch::setHeuristicFunction(double (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &,double)) {
    _heuristic_function = heuristicFunction;
}

void HeuristicSearch::sumNodeHeuristic(double h) {
    _nodes_heuristic_sum += h;
    ++_no_of_nodes;
}

double HeuristicSearch::getAvg() const {
    if (_no_of_nodes == 0)
        return 0;
    return (_nodes_heuristic_sum / _no_of_nodes);
}

void HeuristicSearch::generate_heuristic_stats() {
    std::cout << "Avg H Value: " << getAvg() << std::endl;
}

double HeuristicSearch::getMinOfCostMatrix() const {
    return _min_of_cost_matrix;
}

void HeuristicSearch::setMinOfCostMatrix(double minOfCostMatrix) {
    _min_of_cost_matrix = minOfCostMatrix;
}

