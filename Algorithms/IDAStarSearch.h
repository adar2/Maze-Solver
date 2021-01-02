//
// Created by r00t on 12/15/20.
//

#ifndef AI_PROJECT_IDASTARSEARCH_H
#define AI_PROJECT_IDASTARSEARCH_H

#include "AlgorithmStatistics.h"
#include "HeuristicSearch.h"
#include "ISearchAlgorithm.h"
#include <tuple>

class IDAStarSearch : public ISearchAlgorithm , public AlgorithmStatistics, public HeuristicSearch {
private:
    IDAStarSearch() : AlgorithmStatistics(), HeuristicSearch() {};

    IDAStarSearch(const IDAStarSearch &);

    void operator=(const IDAStarSearch &);
public:
    int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) override;

    pair<shared_ptr<Node>, int>
    DFS_CONTOUR(int **array, int dimension, const shared_ptr<Node>& node, const shared_ptr<Node>& goal, int f_limit, float time_limit);

    static IDAStarSearch &getInstance();

    void generate_stats(const Node &current_node) override;
};


#endif //AI_PROJECT_IDASTARSEARCH_H
