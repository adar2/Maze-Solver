//
// Created by r00t on 12/20/20.
//

#ifndef AI_PROJECT_UTILS_H
#define AI_PROJECT_UTILS_H


int zero_function(const std::pair<int, int>&, const std::pair<int, int>&);

int euclidean_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2);

int manhattan_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2);

void parse_file(char* file_name);

#endif //AI_PROJECT_UTILS_H
