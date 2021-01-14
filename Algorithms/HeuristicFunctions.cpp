
#include <algorithm>
#include <cmath>


double chebyshev_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val) {
    return min_val * std::max(abs(p1.first - p2.first), abs(p1.second - p2.second));
}

double normalized_euclidean_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val) {
    static double square_root = sqrt(2);
    int dx = abs(p1.first - p2.first);
    int dy = abs(p1.second - p2.second);
    return min_val * sqrt(pow(dx, 2) + pow(dy, 2)) / square_root;
}
