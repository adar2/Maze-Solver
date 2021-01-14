

#ifndef AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H
#define AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H

#include "AbstractSearchAlgorithm.h"
#include "Utils.h"

class IterativeDeepeningSearch : public AbstractSearchAlgorithm {
private:
    pair<shared_ptr<Node>, bool>
    DLS(double **array, int dimension, const shared_ptr<Node> &root, const shared_ptr<Node> &goal, int limit,
        float time_limit);

    IterativeDeepeningSearch() : AbstractSearchAlgorithm() {};

    IterativeDeepeningSearch(const IterativeDeepeningSearch &);

    void operator=(const IterativeDeepeningSearch &);

public:
    static IterativeDeepeningSearch &getInstance();

    int run_algorithm(double **array, int dimension, int *start, int *target, float time_limit) override;
};


#endif //AI_PROJECT_ITERATIVEDEEPENINGSEARCH_H
