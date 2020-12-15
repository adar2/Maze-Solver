//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_HEURISTICSEARCH_H
#define AI_PROJECT_HEURISTICSEARCH_H

#include <utility>

using std::pair;

class HeuristicSearch {

protected:
    int (*heuristic_function)(pair<int, int>, pair<int, int>);

public:
    void setHeuristicFunction(
            int (*heuristicFunction)(pair<int, int>, pair<int, int>)) { heuristic_function = heuristicFunction; };

};

#endif //AI_PROJECT_HEURISTICSEARCH_H
