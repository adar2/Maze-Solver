
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
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;

    pair<shared_ptr<Node>, double>
    DFS_CONTOUR(double **array, int dimension, const shared_ptr<Node>& node, const shared_ptr<Node>& goal, double f_limit, float time_limit);

    static IDAStarSearch &getInstance();

    void generate_stats(const Node &current_node) override;
};


#endif //AI_PROJECT_IDASTARSEARCH_H
