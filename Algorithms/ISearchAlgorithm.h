
#ifndef AI_PROJECT_ISEARCHALGORITHM_H
#define AI_PROJECT_ISEARCHALGORITHM_H
class ISearchAlgorithm{
    virtual int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) = 0;
};
#endif //AI_PROJECT_ISEARCHALGORITHM_H
