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

    HeuristicSearch():_heuristic_function(nullptr),_nodes_heuristic_sum(0),_no_of_nodes(0){};

    HeuristicSearch(int(*f)(const pair<int, int> &, const pair<int, int> &)):_heuristic_function(f),
    _nodes_heuristic_sum(0),_no_of_nodes(0){};

public:
    void setHeuristicFunction(int (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &));

    void sumNodeHeuristic(int h);

    double getAvg() const;

    virtual void generate_heuristic_stats();

};

#endif //AI_PROJECT_HEURISTICSEARCH_H
