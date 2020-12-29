//
// Created by r00t on 12/13/20.
//

#ifndef AI_PROJECT_ABSTRACTSEARCHALGORITHM_H
#define AI_PROJECT_ABSTRACTSEARCHALGORITHM_H

#include <iostream>
#include <string>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include "Node.h"

class AbstractSearchAlgorithm {
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
    double _min;
    // avg depth cutoff
    double _avg;
    // max depth cutoff
    double _max;
    // total cutoff
    double _no_of_cutoffs{};
    double _sum_of_cutoffs_depths{};
    clock_t _start_time;
    clock_t _current_time{};

    AbstractSearchAlgorithm() : _end_status(false), _dN(0), _ebf(0), _explored(0), _min(0), _avg(0), _max(0),
                                _start_time(clock()) {}

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

    double getEbf() const { return _ebf; };

    void setEbf(double ebf) { _ebf = ebf; };

    int getExplored() const { return _explored; };

    void setExplored(int explored) { _explored = explored; };

    double getMin() const { return _min; };

    void setMin(double min) { _min = min; };

    double getAvg() const {
        if (_no_of_cutoffs == 0)return 0;
        return _sum_of_cutoffs_depths / _no_of_cutoffs;
    };

    double getMax() const { return _max; };

    void setMax(double max) { _max = max; };

    void addCutoffToSum(int cut_off_depth) {
        _sum_of_cutoffs_depths += cut_off_depth;
        _no_of_cutoffs++;
    }

    clock_t getStartTime() const {
        return _start_time;
    }

    clock_t getCurrentTime() const {
        return _current_time;
    }

    void setCurrentTime(clock_t currentTime) {
        _current_time = currentTime;
    }

    double diff_clock(clock_t clock1, clock_t clock2) {
        double diff_ticks = clock1 - clock2;
        double diff_ms = (diff_ticks) / (CLOCKS_PER_SEC / 1000);
        return diff_ms / 1000;
    }

    virtual int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) = 0;

    virtual void generate_stats(Node &current_node) {
        std::cout << "Problem : " << getProblemName() << std::endl;

        std::cout << "total nodes explored: " << getExplored() << std::endl;
        if (getEndStatus()) {
            auto path_start = current_node.getPathTilNow().begin();
            auto path_end = current_node.getPathTilNow().end();
            for (auto item = path_start + 1; item != path_end; ++item) {
                if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second - 1)
                    std::cout << "LU" << '-';
                if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second)
                    std::cout << "U" << '-';
                if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second - 1)
                    std::cout << "LD" << '-';
                if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second)
                    std::cout << "D" << '-';
                if (item->first == (item - 1)->first && item->second == (item - 1)->second + 1)
                    std::cout << "R" << '-';
                if (item->first == (item - 1)->first && item->second == (item - 1)->second - 1)
                    std::cout << "L" << '-';
                if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second + 1)
                    std::cout << "RD" << '-';
                if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second + 1)
                    std::cout << "RU" << '-';
            }
            std::cout << " solution cost: " << current_node.getActualCost();
        } else std::cout << "Failed";
        std::cout << std::endl;
        std::cout << "d/N : " << getDN() << std::endl;
        std::cout << "time in seconds: " << diff_clock(_current_time, _start_time) << std::endl;
        std::cout << "EBF : " << getEbf() << std::endl;
        std::cout << "Min cutoff : " << getMin() << std::endl;
        std::cout << "Max cutoff : " << getMax() << std::endl;
        std::cout << "Avg cutoff : " << getAvg() << std::endl;
        std::cout << std::endl;
    };
};

// taken from Boost cpp , implementation for pair hashing used in unordered map.
template<typename T>
inline void hash_combine(std::size_t &seed, const T &val) {
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// auxiliary generic functions to create a hash value using a seed
template<typename T>
inline void hash_val(std::size_t &seed, const T &val) {
    hash_combine(seed, val);
}

template<typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &... args) {
    hash_combine(seed, val);
    hash_val(seed, args...);
}

template<typename... Types>
inline std::size_t hash_val(const Types &... args) {
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}

struct pair_hash {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        return hash_val(p.first, p.second);
    }
};

// for debugging purposes
static void print_path(int **array, int dimension, Node &current_node) {
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
    for (auto item = path_start + 1; item != path_end; ++item) {
        if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second - 1)
            std::cout << "LU" << ',';
        if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second)
            std::cout << "U" << ',';
        if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second - 1)
            std::cout << "LD" << ',';
        if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second)
            std::cout << "D" << ',';
        if (item->first == (item - 1)->first && item->second == (item - 1)->second + 1)
            std::cout << "R" << ',';
        if (item->first == (item - 1)->first && item->second == (item - 1)->second - 1)
            std::cout << "L" << ',';
        if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second + 1)
            std::cout << "RD" << ',';
        if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second + 1)
            std::cout << "RU" << ',';
    }
    std::cout << std::endl;
}

#endif //AI_PROJECT_ABSTRACTSEARCHALGORITHM_H
