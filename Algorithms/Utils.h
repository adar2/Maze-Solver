

#ifndef AI_PROJECT_UTILS_H
#define AI_PROJECT_UTILS_H

#include "AbstractSearchAlgorithm.h"

static AbstractSearchAlgorithm *getInstanceOf(const std::string &algorithm_name, double min_val);
void parse_file(const char *file_name);

#endif //AI_PROJECT_UTILS_H
