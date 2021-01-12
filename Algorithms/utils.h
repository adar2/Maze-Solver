

#ifndef AI_PROJECT_UTILS_H
#define AI_PROJECT_UTILS_H

#include "AbstractSearchAlgorithm.h"

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

static AbstractSearchAlgorithm *getInstanceOf(const char *algorithm_name);

double chebyshev_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2);

double avg_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2);

void parse_file(const char *file_name);

#endif //AI_PROJECT_UTILS_H
