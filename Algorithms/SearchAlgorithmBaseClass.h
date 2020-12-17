//
// Created by r00t on 12/13/20.
//

#ifndef AI_PROJECT_SEARCHALGORITHMBASECLASS_H
#define AI_PROJECT_SEARCHALGORITHMBASECLASS_H

#include <iostream>
#include <string>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include "Node.h"

class SearchAlgorithmBaseClass {
protected:
    double _dN;
    double _ebf;
    int _explored;
    double _min;
    double _avg;
    double _max;
    double _no_of_cutoffs{};
    double _sum_of_cutoffs_depths{};
    time_t _start_time;
    time_t _current_time{};

    SearchAlgorithmBaseClass() : _dN(0), _ebf(0), _explored(0), _min(0), _avg(0), _max(0), _start_time(time(nullptr)) {}

public:
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

    time_t getStartTime() const {
        return _start_time;
    }

    time_t getCurrentTime() const {
        return _current_time;
    }

    void setCurrentTime(time_t currentTime) {
        _current_time = currentTime;
    }

    virtual int run_algorithm(int **array, int dimension, int *source, int *goal, float time_limit) = 0;

    virtual void generate_stats(const Node &goalNode) {
        std::queue<pair<int,int>> path = std::queue<pair<int,int>> (goalNode.getPathTilNow());
        while (!path.empty()){
            std::cout << path.front().first << ',' << path.front().second << ':';
            path.pop();
        }
        std::cout << std::endl;
        std::cout << "solution/failure time: " << difftime(getCurrentTime(),getStartTime()) << std::endl;
        std::cout << "d/N : " << getDN() << std::endl;
        std::cout << "EBF : " << getEbf() << std::endl;
        std::cout << "total nodes explored: " << getExplored() << std::endl;
        std::cout << "Min cutoff : " << getMin() << std::endl;
        std::cout << "Max cutoff : " << getMax() << std::endl;
        std::cout << "Avg cutoff : " << getAvg() << std::endl;
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

#endif //AI_PROJECT_SEARCHALGORITHMBASECLASS_H
