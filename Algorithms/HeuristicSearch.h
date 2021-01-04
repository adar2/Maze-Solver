//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_HEURISTICSEARCH_H
#define AI_PROJECT_HEURISTICSEARCH_H

#include <utility>

using std::pair;

class HeuristicSearch {

protected:
    // pointer to heuristic function
    int (*_heuristic_function)(const pair<int, int> &, const pair<int, int> &);
    // sum of heuristic value of all nodes explored
    double _nodes_heuristic_sum;
    // number of nodes explored, for the use of calculating the average h value.
    int _no_of_nodes;

    HeuristicSearch():_heuristic_function(nullptr),_nodes_heuristic_sum(0),_no_of_nodes(0){};

public:
    // method for setting the heuristic function.
    void setHeuristicFunction(int (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &));
    // add to sum node h value
    void sumNodeHeuristic(int h);
    // return avg h value of the search
    double getAvg() const;
    // method for generate heuristic related stats.
    virtual void generate_heuristic_stats();

};

#endif //AI_PROJECT_HEURISTICSEARCH_H
