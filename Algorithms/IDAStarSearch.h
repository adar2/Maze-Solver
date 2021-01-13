
#ifndef AI_PROJECT_IDASTARSEARCH_H
#define AI_PROJECT_IDASTARSEARCH_H

#include "AbstractSearchAlgorithm.h"
#include "HeuristicSearch.h"
#include <tuple>

class IDAStarSearch : public AbstractSearchAlgorithm, public HeuristicSearch {
private:
    IDAStarSearch() : AbstractSearchAlgorithm(), HeuristicSearch() {};

    IDAStarSearch(const IDAStarSearch &);

    void operator=(const IDAStarSearch &);

public:
    int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) override;

    pair<shared_ptr<Node>, double>
    DFS_CONTOUR(double **array, int dimension, const shared_ptr<Node> &node, const shared_ptr<Node> &goal,
                double f_limit, float time_limit);

    static IDAStarSearch &getInstance();

};


#endif //AI_PROJECT_IDASTARSEARCH_H
