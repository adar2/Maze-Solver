//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_ASTARSEARCH_H
#define AI_PROJECT_ASTARSEARCH_H

#include "AbstractSearchAlgorithm.h"
#include "HeuristicSearch.h"

class AStarSearch : public AbstractSearchAlgorithm, public HeuristicSearch {
private:


    AStarSearch() : AbstractSearchAlgorithm(), HeuristicSearch() {};

    AStarSearch(const AStarSearch &);

    void operator=(const AStarSearch &);

public:
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;

    static AStarSearch &getInstance();


};


#endif //AI_PROJECT_ASTARSEARCH_H
