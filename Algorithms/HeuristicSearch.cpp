

#include "HeuristicSearch.h"
#include <iostream>

void HeuristicSearch::setHeuristicFunction(double (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &)) {
    _heuristic_function = heuristicFunction;
}

void HeuristicSearch::sumNodeHeuristic(double h) {
    _nodes_heuristic_sum += h;
    ++_no_of_nodes;
}

double HeuristicSearch::getAvg() const {
    if(_no_of_nodes == 0)
        return 0;
    return (_nodes_heuristic_sum / _no_of_nodes);
}

void HeuristicSearch::generate_heuristic_stats() {
    std::cout<< "Avg H Value: " << getAvg() << std::endl;
}

