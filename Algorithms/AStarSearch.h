//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_ASTARSEARCH_H
#define AI_PROJECT_ASTARSEARCH_H

#include "SearchAlgorithmBaseClass.h"
#include "HeuristicSearch.h"

class AStarSearch : public SearchAlgorithmBaseClass, public HeuristicSearch {
private:


    AStarSearch() : SearchAlgorithmBaseClass(), HeuristicSearch() {};

    AStarSearch(const AStarSearch &);

    void operator=(const AStarSearch &);

public:
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;

    static AStarSearch &getInstance();


};


#endif //AI_PROJECT_ASTARSEARCH_H
