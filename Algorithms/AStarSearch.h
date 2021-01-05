//
// Created by r00t on 12/14/20.
//

#ifndef AI_PROJECT_ASTARSEARCH_H
#define AI_PROJECT_ASTARSEARCH_H

#include "AlgorithmStatistics.h"
#include "HeuristicSearch.h"
#include "ISearchAlgorithm.h"
#include "utils.h"

class AStarSearch : public ISearchAlgorithm, public AlgorithmStatistics, public HeuristicSearch {
private:


    AStarSearch() : AlgorithmStatistics(), HeuristicSearch() {};

    AStarSearch(const AStarSearch &);

    void operator=(const AStarSearch &);

public:
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override ;

    static AStarSearch &getInstance();

    void generate_stats(const Node &current_node) override;


};


#endif //AI_PROJECT_ASTARSEARCH_H
