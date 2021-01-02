//
// Created by r00t on 12/31/20.
//

#ifndef AI_PROJECT_ISEARCHALGORITHM_H
#define AI_PROJECT_ISEARCHALGORITHM_H
class ISearchAlgorithm{
    virtual int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) = 0;
};
#endif //AI_PROJECT_ISEARCHALGORITHM_H
