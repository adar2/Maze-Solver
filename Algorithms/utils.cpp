//
// Created by r00t on 12/20/20.
//

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


int euclidean_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
    return int(sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2)));
}

int chebyshev_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
    return std::max(abs(p1.first - p2.first), abs(p1.second - p2.second));
}

int octile_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2){
    int dx = abs(p1.first - p2.first);
    int dy = abs(p1.second - p2.second);
    return int((dx + dy) + (-0.585)*std::min(dx, dy));
}

void parse_file(const char *file_name) {
    std::string algorithm_name, dimension, sourceStr, targetStr;
    auto *source = new int[2];
    auto *target = new int[2];
    float time_limit;
    std::ifstream inputFile(file_name);
    getline(inputFile, algorithm_name, '\n');
    // remove carriage return from algorithm name string.
    if(algorithm_name.find('\r') != -1)
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
    time_limit = float(log2(d));
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
    if(source[0] < 0 || source[0] >= d || source[1] < 0 || source[1] >= d || array[source[0]][source[1]] < 0 ||target[0] < 0 || target[0] >= d || target[1] < 0 || target[1] >= d || array[target[0]][target[1]] < 0 ){
        delete[] source;
        delete[] target;
        for (int i = 0; i < d; ++i) {
            delete[] array[i];
        }
        delete[] array;
        std::cout << "Illegal input , exiting.." << std::endl;
        return;
    }
    if (algorithm_name == "BIASTAR") {
        BiDirectionalAStar::getInstance().setProblemName(file_name);
        BiDirectionalAStar::getInstance().setHeuristicFunction(octile_distance);
        BiDirectionalAStar::getInstance().run_algorithm(array, d, source, target, time_limit);
    } else if (algorithm_name == "IDASTAR") {
        IDAStarSearch::getInstance().setProblemName(file_name);
        IDAStarSearch::getInstance().setHeuristicFunction(octile_distance);
        IDAStarSearch::getInstance().run_algorithm(array, d, source, target, time_limit);
    } else if (algorithm_name == "ASTAR") {
        AStarSearch::getInstance().setProblemName(file_name);
        AStarSearch::getInstance().setHeuristicFunction(octile_distance);
        AStarSearch::getInstance().run_algorithm(array, d, source, target, time_limit);
    } else if (algorithm_name == "UCS") {
        UniformCostSearch::getInstance().setProblemName(file_name);
        UniformCostSearch::getInstance().run_algorithm(array,d,source,target,time_limit);
    } else if (algorithm_name == "IDS") {
        IterativeDeepeningSearch::getInstance().setProblemName(file_name);
        IterativeDeepeningSearch::getInstance().run_algorithm(array, d, source, target, time_limit);
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