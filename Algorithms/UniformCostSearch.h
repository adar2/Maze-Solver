

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
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;
};


#endif //AI_PROJECT_UNIFORMCOSTSEARCH_H
