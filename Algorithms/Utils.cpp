

#include <iostream>
#include <cstring>
#include <algorithm>
#include "Utils.h"
#include "UniformCostSearch.h"
#include "IterativeDeepeningSearch.h"
#include "AStarSearch.h"
#include "IDAStarSearch.h"
#include "BiDirectionalAStar.h"
#include "HeuristicFunctions.h"

static AbstractSearchAlgorithm *getInstanceOf(const std::string &algorithm_name, double min_val) {
    if (algorithm_name == "BIASTAR") {
        BiDirectionalAStar::getInstance().setMinOfCostMatrix(min_val);
        BiDirectionalAStar::getInstance().setHeuristicFunction(chebyshev_distance);
        return &BiDirectionalAStar::getInstance();
    } else if (algorithm_name == "IDASTAR") {
        IDAStarSearch::getInstance().setMinOfCostMatrix(min_val);
        IDAStarSearch::getInstance().setHeuristicFunction(chebyshev_distance);
        return &IDAStarSearch::getInstance();
    } else if (algorithm_name == "ASTAR") {
        AStarSearch::getInstance().setMinOfCostMatrix(min_val);
        AStarSearch::getInstance().setHeuristicFunction(chebyshev_distance);
        return &AStarSearch::getInstance();
    } else if (algorithm_name == "UCS") {
        return &UniformCostSearch::getInstance();
    } else if (algorithm_name == "IDS") {
        return &IterativeDeepeningSearch::getInstance();
    } else {
        return nullptr;
    }
}

void parse_file(const char *file_name) {
    std::string algorithm_name, dimensionStr, sourceStr, targetStr, input_time_limit;
    auto *source = new int[2];
    auto *goal = new int[2];
    int dimension;
    float time_limit;
    double min = std::numeric_limits<double>::infinity();
    std::ifstream inputFile(file_name);
    if (!inputFile.good()) {
        delete[] source;
        delete[] goal;
        std::cout << "File does not exist , aborting.." << std::endl;
        exit(1);

    }
    getline(inputFile, algorithm_name, '\n');
    // remove carriage return from algorithm name string.
    if (algorithm_name.find('\r') != std::string::npos) {
        algorithm_name.erase(algorithm_name.size() - 1);
    }
    getline(inputFile, dimensionStr, '\n');
    dimension = stoi(dimensionStr);
    time_limit = float(log2(dimension));
    std::cout << "The default time limit is currently: " << time_limit
              << " ,If you wish to change it please enter new float , Or press enter to continue: ";
    std::getline(std::cin, input_time_limit);
    if (!input_time_limit.empty())
        time_limit = std::stof(input_time_limit);
    clock_t startTime = clock();
    getline(inputFile, sourceStr, '\n');
    getline(inputFile, targetStr, '\n');
    source[0] = stoi(sourceStr.substr(0, sourceStr.find(',')));
    sourceStr.erase(0, sourceStr.find(',') + 1);
    source[1] = stoi(sourceStr);
    goal[0] = stoi(targetStr.substr(0, sourceStr.find(',')));
    targetStr.erase(0, targetStr.find(',') + 1);
    goal[1] = stoi(targetStr);
    std::string tmpStr;
    auto **array = new double *[dimension];
    for (int i = 0; i < dimension; ++i) {
        array[i] = new double[dimension];
        getline(inputFile, tmpStr, '\n');
        tmpStr.erase(remove(tmpStr.begin(), tmpStr.end(), ' '), tmpStr.end());
        size_t pos;
        std::string token;
        for (int j = 0; j < dimension; ++j) {
            pos = tmpStr.find(',');
            token = tmpStr.substr(0, pos);
            array[i][j] = stod(token);
            if (array[i][j] < min && array[i][j] > 0)
                min = array[i][j];
            tmpStr.erase(0, pos + 1);
        }
    }
    inputFile.close();
    if (source[0] < 0 || source[0] >= dimension || source[1] < 0 || source[1] >= dimension ||
        array[source[0]][source[1]] < 0 ||
        goal[0] < 0 || goal[0] >= dimension || goal[1] < 0 || goal[1] >= dimension || array[goal[0]][goal[1]] < 0) {
        delete[] source;
        delete[] goal;
        for (int i = 0; i < dimension; ++i) {
            delete[] array[i];
        }
        delete[] array;
        std::cout << "Illegal input , aborting.." << std::endl;
        return;
    }
    AbstractSearchAlgorithm *algorithm = getInstanceOf(algorithm_name, min);
    if (algorithm) {
        algorithm->setAlgorithmName(algorithm_name);
        algorithm->setStartTime(startTime);
        algorithm->setProblemName(file_name);
        algorithm->run_algorithm(array, dimension, source, goal, time_limit);

    } else {
        std::cout << "unknown algorithm, aborting.." << std::endl;
    }
    delete[] source;
    delete[] goal;
    for (int i = 0; i < dimension; ++i) {
        delete[] array[i];
    }
    delete[] array;
}