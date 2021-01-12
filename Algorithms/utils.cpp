

#include <iostream>
#include <fstream>
#include <cstring>
#include <algorithm>
#include "utils.h"
#include "UniformCostSearch.h"
#include "IterativeDeepeningSearch.h"
#include "AStarSearch.h"
#include "IDAStarSearch.h"
#include "BiDirectionalAStar.h"

static AbstractSearchAlgorithm *getInstanceOf(const std::string &algorithm_name) {
    if (algorithm_name == "BIASTAR") {
        BiDirectionalAStar::getInstance().setHeuristicFunction(chebyshev_distance);
        return &BiDirectionalAStar::getInstance();
    } else if (algorithm_name == "IDASTAR") {
        IDAStarSearch::getInstance().setHeuristicFunction(chebyshev_distance);
        return &IDAStarSearch::getInstance();
    } else if (algorithm_name == "ASTAR") {
        AStarSearch::getInstance().setHeuristicFunction(avg_distance);
        return &AStarSearch::getInstance();
    } else if (algorithm_name == "UCS") {
        return &UniformCostSearch::getInstance();
    } else if (algorithm_name == "IDS") {
        return &IterativeDeepeningSearch::getInstance();
    } else {
        return nullptr;
    }
}

double chebyshev_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
    return std::max(abs(p1.first - p2.first), abs(p1.second - p2.second));
}

double avg_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2) {
    static double square_root = sqrt(2);
    int dx = abs(p1.first - p2.first);
    int dy = abs(p1.second - p2.second);
    return sqrt(pow(dx,2)+pow(dy,2)) / square_root;
}

void parse_file(const char *file_name) {
    clock_t startTime = clock();
    std::string algorithm_name, dimension, sourceStr, targetStr;
    auto *source = new int[2];
    auto *target = new int[2];
    float time_limit;
    std::ifstream inputFile(file_name);
    getline(inputFile, algorithm_name, '\n');
    // remove carriage return from algorithm name string.
    if (algorithm_name.find('\r') != -1)
        algorithm_name.erase(algorithm_name.size() - 1);
    getline(inputFile, dimension, '\n');
    getline(inputFile, sourceStr, '\n');
    getline(inputFile, targetStr, '\n');
    source[0] = stoi(sourceStr.substr(0, sourceStr.find(',')));
    sourceStr.erase(0, sourceStr.find(',') + 1);
    source[1] = stoi(sourceStr);
    target[0] = stoi(targetStr.substr(0, sourceStr.find(',')));
    targetStr.erase(0, targetStr.find(',') + 1);
    target[1] = stoi(targetStr);
    int d = stoi(dimension);
    time_limit = 2 * float(log2(d));
    std::string tmpStr;
    auto **array = new double *[d];
    for (int i = 0; i < d; ++i) {
        array[i] = new double[d];
        getline(inputFile, tmpStr, '\n');
        tmpStr.erase(remove(tmpStr.begin(), tmpStr.end(), ' '), tmpStr.end());
        size_t pos;
        std::string token;
        for (int j = 0; j < d; ++j) {
            pos = tmpStr.find(',');
            token = tmpStr.substr(0, pos);
            array[i][j] = stod(token);
            tmpStr.erase(0, pos + 1);
        }
    }
    if (source[0] < 0 || source[0] >= d || source[1] < 0 || source[1] >= d || array[source[0]][source[1]] < 0 ||
        target[0] < 0 || target[0] >= d || target[1] < 0 || target[1] >= d || array[target[0]][target[1]] < 0) {
        delete[] source;
        delete[] target;
        for (int i = 0; i < d; ++i) {
            delete[] array[i];
        }
        delete[] array;
        std::cout << "Illegal input , aborting.." << std::endl;
        return;
    }
    AbstractSearchAlgorithm *algorithm = getInstanceOf(algorithm_name);
    if (algorithm) {
        algorithm->setStartTime(startTime);
        algorithm->setProblemName(file_name);
        algorithm->run_algorithm(array, d, source, target, time_limit);

    } else {
        std::cout << "unknown algorithm, exiting.." << std::endl;
    }
    delete[] source;
    delete[] target;
    for (int i = 0; i < d; ++i) {
        delete[] array[i];
    }
    delete[] array;
}