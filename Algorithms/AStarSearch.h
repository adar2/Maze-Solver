
#ifndef AI_PROJECT_ASTARSEARCH_H
#define AI_PROJECT_ASTARSEARCH_H

#include "AbstractSearchAlgorithm.h"
#include "HeuristicSearch.h"
#include "utils.h"

class AStarSearch : public AbstractSearchAlgorithm, public HeuristicSearch {
private:


    AStarSearch() : AbstractSearchAlgorithm(), HeuristicSearch() {};

    AStarSearch(const AStarSearch &);

    void operator=(const AStarSearch &);

public:
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;

    static AStarSearch &getInstance();

    void generate_stats(const Node &current_node) override;


};


#endif //AI_PROJECT_ASTARSEARCH_H
