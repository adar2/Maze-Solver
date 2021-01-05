//
// Created by r00t on 12/13/20.
//

#ifndef AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H
#define AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H

#include "AlgorithmStatistics.h"
#include "ISearchAlgorithm.h"
#include "utils.h"

class IterativeDeepeningSearch : public ISearchAlgorithm, public AlgorithmStatistics {
private:
    pair<shared_ptr<Node>, bool> DLS(double **array, int dimension,const shared_ptr<Node>& root,const shared_ptr<Node>& goal, int limit, float time_limit);

    IterativeDeepeningSearch() : AlgorithmStatistics() {};

    IterativeDeepeningSearch(const IterativeDeepeningSearch &);

    void operator=(const IterativeDeepeningSearch &);

public:
    static IterativeDeepeningSearch &getInstance();

    int run_algorithm(double **array, int dimension, int *start, int *target, float time_limit) override;
};


#endif //AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H
