//
// Created by r00t on 12/15/20.
//

#ifndef AI_PROJECT_BIDIRECTIONALASTAR_H
#define AI_PROJECT_BIDIRECTIONALASTAR_H

#include "AbstractSearchAlgorithm.h"
#include "HeuristicSearch.h"

class BiDirectionalAStar : public AbstractSearchAlgorithm, public HeuristicSearch {
private:
    BiDirectionalAStar() : AbstractSearchAlgorithm(), HeuristicSearch() {};

    BiDirectionalAStar(const BiDirectionalAStar &);

    void operator=(const BiDirectionalAStar &);

public:
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;

    static BiDirectionalAStar &getInstance();
};


#endif //AI_PROJECT_BIDIRECTIONALASTAR_H
