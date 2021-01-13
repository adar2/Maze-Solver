

#ifndef AI_PROJECT_HEURISTICSEARCH_H
#define AI_PROJECT_HEURISTICSEARCH_H

#include <utility>

using std::pair;

class HeuristicSearch {

protected:
    // pointer to heuristic function
    double (*_heuristic_function)(const pair<int, int> &, const pair<int, int> &, double);

    // sum of heuristic value of all nodes explored
    double _nodes_heuristic_sum;
    // number of nodes explored, for the use of calculating the average h value.
    int _no_of_nodes;
    // minimum value of the costs matrix
    double _min_of_cost_matrix;

    HeuristicSearch() : _heuristic_function(nullptr), _nodes_heuristic_sum(0), _no_of_nodes(0),
                        _min_of_cost_matrix(0) {};

public:
    // method for setting the heuristic function.
    void setHeuristicFunction(double (*heuristicFunction)(const pair<int, int> &, const pair<int, int> &, double));

    // add to sum node h value
    void sumNodeHeuristic(double h);

    // return avg h value of the search
    double getAvgHeuristicValue() const;

    double getMinOfCostMatrix() const;

    void setMinOfCostMatrix(double minOfCostMatrix);

};

#endif //AI_PROJECT_HEURISTICSEARCH_H
