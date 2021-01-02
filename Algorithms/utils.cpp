//
// Created by r00t on 12/20/20.
//

#include <iostream>
#include <fstream>
#include <cstring>
#include <algorithm>
#include "utils.h"
#include "SearchAlgorithms.h"
#include "IterativeDeepeningSearch.h"
#include "AStarSearch.h"
#include "IDAStarSearch.h"
#include "BiDirectionalAStar.h"

#define TIME_LIMIT 20


int zero_function(const pair<int, int> &, const pair<int, int> &) {
    return 0;
}

int euclidean_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

int manhattan_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

void parse_file(char *file_name) {
    std::string algorithm_name, dimension, sourceStr, targetStr;
    auto *source = new int[2];
    auto *target = new int[2];
    std::ifstream inputFile(file_name);
    getline(inputFile, algorithm_name, '\n');
    // remove carriage return from algorithm name string.
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
    std::string tmpStr;
    auto **array = new int *[d];
    for (int i = 0; i < d; ++i) {
        array[i] = new int[d];
        getline(inputFile, tmpStr, '\n');
        tmpStr.erase(remove(tmpStr.begin(), tmpStr.end(), ' '), tmpStr.end());
        size_t pos = 0;
        std::string token;
        for (int j = 0; j < d; ++j) {
            pos = tmpStr.find(',');
            token = tmpStr.substr(0, pos);
            array[i][j] = stoi(token);
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
        BiDirectionalAStar::getInstance().setHeuristicFunction(manhattan_distance);
        BiDirectionalAStar::getInstance().run_algorithm(array, d, source, target, TIME_LIMIT);
    } else if (algorithm_name == "IDASTAR") {
        IDAStarSearch::getInstance().setProblemName(file_name);
        IDAStarSearch::getInstance().setHeuristicFunction(manhattan_distance);
        IDAStarSearch::getInstance().run_algorithm(array, d, source, target, TIME_LIMIT);
    } else if (algorithm_name == "ASTAR") {
        AStarSearch::getInstance().setHeuristicFunction(manhattan_distance);
        AStarSearch::getInstance().setProblemName(file_name);
        AStarSearch::getInstance().run_algorithm(array, d, source, target, TIME_LIMIT);
    } else if (algorithm_name == "UCS") {
        AStarSearch::getInstance().setHeuristicFunction(zero_function);
        AStarSearch::getInstance().setProblemName(file_name);
        AStarSearch::getInstance().run_algorithm(array, d, source, target, TIME_LIMIT);
    } else if (algorithm_name == "IDS") {
        IterativeDeepeningSearch::getInstance().setProblemName(file_name);
        IterativeDeepeningSearch::getInstance().run_algorithm(array, d, source, target, TIME_LIMIT);
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