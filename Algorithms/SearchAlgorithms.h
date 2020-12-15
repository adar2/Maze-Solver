//
// Created by adar on 12/5/2020.
//

#ifndef AI_PROJECT_SEARCHALGORITHMS_H
#define AI_PROJECT_SEARCHALGORITHMS_H

#include "Node.h"

int uniformCostSearch(int **array, int dimension, int *start, int *target);

int IDS(int **array, int dimension, int *source, int *goal);

pair<Node *, bool> DLS(int **array, int dimension, Node *current_node, Node *goal, int limit);

#endif //AI_PROJECT_SEARCHALGORITHMS_H
