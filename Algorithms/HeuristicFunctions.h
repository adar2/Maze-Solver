//
// Created by r00t on 1/14/21.
//

#ifndef AI_PROJECT_HEURISTICFUNCTIONS_H
#define AI_PROJECT_HEURISTICFUNCTIONS_H
double chebyshev_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val);

double normalized_euclidean_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val);

shared_ptr<vector<shared_ptr<int[]>>> bfs(double **array, int dimension, int *goal);

double bfs_distance(const pair<int, int> &p1, const pair<int, int> &p2,double min_val ,double **array, int dimension, int *goal);
#endif //AI_PROJECT_HEURISTICFUNCTIONS_H
