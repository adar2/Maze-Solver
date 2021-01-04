//
// Created by r00t on 1/2/21.
//

#ifndef AI_PROJECT_UNIFORMCOSTSEARCH_H
#define AI_PROJECT_UNIFORMCOSTSEARCH_H

#include "ISearchAlgorithm.h"
#include "AlgorithmStatistics.h"

class UniformCostSearch : public ISearchAlgorithm , public AlgorithmStatistics {
private:
    UniformCostSearch():AlgorithmStatistics(){};
    UniformCostSearch(const UniformCostSearch &);
    void operator=(const UniformCostSearch &);

public:
    static UniformCostSearch& getInstance();
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;
};


#endif //AI_PROJECT_UNIFORMCOSTSEARCH_H
