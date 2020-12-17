//
// Created by r00t on 12/15/20.
//

#ifndef AI_PROJECT_BIDIRECTIONALASTAR_H
#define AI_PROJECT_BIDIRECTIONALASTAR_H

#include "SearchAlgorithmBaseClass.h"
#include "HeuristicSearch.h"

class BiDirectionalAStar : public SearchAlgorithmBaseClass, public HeuristicSearch {
public:
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;
};


#endif //AI_PROJECT_BIDIRECTIONALASTAR_H
