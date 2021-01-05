//
// Created by r00t on 12/13/20.
//

#ifndef AI_PROJECT_ALGORITHMSTATISTICS_H
#define AI_PROJECT_ALGORITHMSTATISTICS_H

#include <iostream>
#include <string>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include "Node.h"

class AlgorithmStatistics {
protected:
    // true for success false for failure.
    bool _end_status;
    // file name of the maze
    std::string _problem_name;
    // depth divided by number of nodes explored
    double _dN;
    // effective branching factor
    double _ebf;
    // total number of nodes explored bu the algorithm
    int _explored;
    // min depth cutoff
    int _min;
    // avg depth cutoff
    double _avg;
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

    AlgorithmStatistics() : _end_status(false), _dN(0), _ebf(0), _explored(0), _min(0), _avg(0), _max(0), _no_of_cutoffs(0),_sum_of_cutoffs_depths(0),
                            _start_time(clock()),_current_time(0) {}

public:
    bool getEndStatus() const {
        return _end_status;
    }

    void setEndStatus(bool endStatus) {
        _end_status = endStatus;
    }

    const std::string &getProblemName() const {
        return _problem_name;
    }

    void setProblemName(const std::string &problemName) {
        _problem_name = problemName;
    }

    double getDN() const { return _dN; };

    void setDN(double dN) { _dN = dN; };
    // calculate d/N branching factor using solution depth in case of success or with max cutoff depth in failure.
    void calcDN(int depth);

    double getEBF() const { return _ebf; };

    void setEBF(double ebf) { _ebf = ebf; };
    // calculate effective branching factor using solution depth in case of success or with max cutoff depth in failure.
    void calcEBF(int depth);

    int getExplored() const { return _explored; };

    void setExplored(int explored) { _explored = explored; };

    int getMin() const { return _min; };

    void setMin(int min) { _min = min; };

    double getAvg() const ;

    int getMax() const { return _max; };

    void setMax(int max) { _max = max; };

    void addCutoffToSum(int cut_off_depth);

    clock_t getStartTime() const {
        return _start_time;
    }

    clock_t getCurrentTime() const {
        return _current_time;
    }

    void setCurrentTime(const clock_t &currentTime) {
        _current_time = currentTime;
    }
    // static function for comparing time
    static double diff_clock(const clock_t& clock1, const clock_t& clock2);
    // function for maintain algorithm min,max and avg cutoff statistics/
    void update_cutoffs(int cutoff_depth);
    // function for statistics generation, overloaded in heuristic search algorithm to combine with heuristic stats.
    virtual void generate_stats(const Node &current_node);
};



// for debugging and path visualization purposes , will print the path marked by red on linux os
static void print_path(double **array, int dimension,const Node &current_node) {
    auto path_start = current_node.getPathTilNow().begin();
    auto path_end = current_node.getPathTilNow().end();
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            if (std::find(path_start, path_end, pair<int, int>(i, j)) != path_end) {
                std::cout << "\033[1;31m" << '(' << array[i][j] << "), " << "\033[0m";
            } else
                std::cout << '(' << array[i][j] << "), ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

#endif //AI_PROJECT_ALGORITHMSTATISTICS_H
