
#ifndef AI_PROJECT_ABSTRACTSEARCHALGORITHM_H
#define AI_PROJECT_ABSTRACTSEARCHALGORITHM_H

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include "Node.h"

class AbstractSearchAlgorithm {
protected:
    // for statistics file generation
    std::string _algorithm_name;
    // true for success false for failure.
    bool _end_status;
    // file name of the maze
    std::string _problem_name;
    // depth divided by number of nodes explored
    double _dN;
    // effective branching factor
    double _ebf;
    // total number of nodes explored bu the algorithm
    int _expanded;
    // min depth cutoff
    int _min;
    // max depth cutoff
    int _max;
    // total cutoff
    int _no_of_cutoffs;
    // sum of cutoffs depths
    int _sum_of_cutoffs_depths;
    // algorithm time
    clock_t _start_time;
    // keep track on time in order to limit algorithm run time
    clock_t _current_time;

    AbstractSearchAlgorithm() : _end_status(false), _dN(0), _ebf(0), _expanded(0),
                                _min(std::numeric_limits<int>::max()), _max(0), _no_of_cutoffs(0),
                                _sum_of_cutoffs_depths(0),
                                _start_time(0), _current_time(0) {}

public:

    const std::string &getAlgorithmName() const;

    void setAlgorithmName(const std::string &algorithmName);

    bool getEndStatus() const {
        return _end_status;
    }

    void setEndStatus(bool endStatus) {
        _end_status = endStatus;
    }

    const std::string &getProblemName() const {
        return _problem_name;
    }

    void setProblemName(const std::string &problemName);

    double getDN() const { return _dN; };

    void setDN(double dN) { _dN = dN; };

    // calculate d/N branching factor using solution depth in case of success or with max cutoff depth in failure.
    void calcDN(int depth);

    double getEBF() const { return _ebf; };

    void setEBF(double ebf) { _ebf = ebf; };

    // calculate effective branching factor using solution depth in case of success or with max cutoff depth in failure.
    void calcEBF(int depth);

    int getExpanded() const { return _expanded; };;

    int getMin() const { return _min; };;

    double getAvg() const;

    int getMax() const { return _max; };;

    void addCutoffToSum(int cut_off_depth);

    clock_t getStartTime() const {
        return _start_time;
    }

    clock_t getCurrentTime() const {
        return _current_time;
    }

    void setStartTime(const clock_t &startTime) { _start_time = startTime; }

    void setCurrentTime(const clock_t &currentTime) {
        _current_time = currentTime;
    }

    // static function for comparing time
    static double diff_clock(const clock_t &clock1, const clock_t &clock2);

    // function for maintain algorithm min,max and avg cutoff statistics/
    void update_cutoffs(int cutoff_depth);

    // function for statistics generation, overloaded in heuristic search algorithm to combine with heuristic stats.
    void generate_stats(const Node &current_node, double avg_heuristic_val = 0);

    // pure virtual function must be overridden to implement algorithm
    virtual int run_algorithm(double **array, int dimension, int *source, int *goal, float time_limit) = 0;
};


// for debugging and path visualization purposes , will print the entire cost matrix with cells on optimal path marked by red on linux os
//static void print_path(double **array, int dimension, const Node &current_node) {
//    auto path_start = current_node.getPathTilNow().begin();
//    auto path_end = current_node.getPathTilNow().end();
//    for (int i = 0; i < dimension; ++i) {
//        for (int j = 0; j < dimension; ++j) {
//            if (std::find(path_start, path_end, pair<int, int>(i, j)) != path_end) {
//                std::cout << "\033[1;31m" << '(' << array[i][j] << "), " << "\033[0m";
//            } else
//                std::cout << '(' << array[i][j] << "), ";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;
//}

#endif //AI_PROJECT_ABSTRACTSEARCHALGORITHM_H
