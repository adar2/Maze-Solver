//
// Created by r00t on 12/13/20.
//

#ifndef AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H
#define AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H

#include "SearchAlgorithmBaseClass.h"

class IterativeDeepeningSearch : public SearchAlgorithmBaseClass {
private:
    pair<Node *, bool> DLS(int **array, int dimension, Node *root, Node *goal, int limit);

    IterativeDeepeningSearch() : SearchAlgorithmBaseClass() {};

    IterativeDeepeningSearch(const IterativeDeepeningSearch &);

    void operator=(const IterativeDeepeningSearch &);

public:
    static IterativeDeepeningSearch &getInstance();

    int run_algorithm(int **array, int dimension, int *start, int *target, float time_limit) override;
};


#endif //AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H