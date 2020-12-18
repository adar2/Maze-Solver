//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_HEURISTICSEARCH_H
#define AI_PROJECT_HEURISTICSEARCH_H

#include <utility>

using std::pair;

class HeuristicSearch {

protected:
    int (*_heuristic_function)(const pair<int, int> &, const pair<int, int> &);

    double _nodes_heuristic_sum;
    int _no_of_nodes;

public:
    void setHeuristicFunction(
            int (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &));

    void sumNodeHeuristic(int h);

    double getAvg() const;

};

#endif //AI_PROJECT_HEURISTICSEARCH_H
