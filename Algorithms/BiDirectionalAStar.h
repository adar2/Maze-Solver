//
// Created by r00t on 12/15/20.
//

#ifndef AI_PROJECT_BIDIRECTIONALASTAR_H
#define AI_PROJECT_BIDIRECTIONALASTAR_H

#include "AlgorithmStatistics.h"
#include "HeuristicSearch.h"
#include "ISearchAlgorithm.h"
#include "utils.h"

class BiDirectionalAStar : public ISearchAlgorithm , public AlgorithmStatistics, public HeuristicSearch {
private:
    BiDirectionalAStar() : AlgorithmStatistics(), HeuristicSearch() {};

    BiDirectionalAStar(const BiDirectionalAStar &);

    void operator=(const BiDirectionalAStar &);

public:
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;

    static BiDirectionalAStar &getInstance();

    void generate_stats(const Node &current_node) override;
};


#endif //AI_PROJECT_BIDIRECTIONALASTAR_H
