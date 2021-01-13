

#ifndef AI_PROJECT_UNIFORMCOSTSEARCH_H
#define AI_PROJECT_UNIFORMCOSTSEARCH_H

#include "AbstractSearchAlgorithm.h"

class UniformCostSearch : public AbstractSearchAlgorithm {
private:
    UniformCostSearch() : AbstractSearchAlgorithm() {};

    UniformCostSearch(const UniformCostSearch &);

    void operator=(const UniformCostSearch &);

public:
    static UniformCostSearch &getInstance();

    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;
};


#endif //AI_PROJECT_UNIFORMCOSTSEARCH_H
