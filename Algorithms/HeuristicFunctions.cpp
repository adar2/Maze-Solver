//
// Created by r00t on 1/14/21.
//
#include <iostream>
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <queue>
#include "PairHashing.h"
#include "Node.h"

using std::pair;
using std::shared_ptr;

double chebyshev_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val) {
    return min_val * std::max(abs(p1.first - p2.first), abs(p1.second - p2.second));
}

double normalized_euclidean_distance(const std::pair<int, int> &p1, const std::pair<int, int> &p2, double min_val) {
    static double square_root = sqrt(2);
    int dx = abs(p1.first - p2.first);
    int dy = abs(p1.second - p2.second);
    return min_val * sqrt(pow(dx, 2) + pow(dy, 2)) / square_root;
}

shared_ptr<vector<shared_ptr<int[]>>> bfs(double **array, int dimension, int *goal) {
    shared_ptr<vector<shared_ptr<int[]>>> bfs(new vector<shared_ptr<int[]>>());
    std::queue<std::pair<int,int>> q = std::queue<std::pair<int,int>>();
    std::unordered_map<std::pair<int,int>,bool,pair_hash> discovered = std::unordered_map<std::pair<int,int>,bool,pair_hash>();
    int row,col;
    std::pair<int,int> current;
    for(int i=0;i<dimension;++i){
        (*bfs).push_back(shared_ptr<int[]>(new int[dimension]{0}));
    }
    (*bfs)[goal[0]][goal[1]] = 0;
    q.push(std::pair<int,int>(goal[0],goal[1]));
    while(!q.empty()){
        current = q.front();
        q.pop();
        discovered[std::pair<int,int>(current.first,current.second)] = true;
        for (int i = 0; i < ACTIONS_SIZE; ++i) {
            switch (actions(i)) {
                case U:
                    row = current.first - 1;
                    col = current.second;
                    break;
                case RU:
                    row = current.first - 1;
                    col = current.second + 1;
                    break;
                case R:
                    row = current.first;
                    col = current.second + 1;
                    break;
                case RD:
                    row = current.first + 1;
                    col = current.second + 1;
                    break;
                case D:
                    row = current.first + 1;
                    col = current.second;
                    break;
                case LD:
                    row = current.first + 1;
                    col = current.second - 1;
                    break;
                case L:
                    row = current.first;
                    col = current.second - 1;
                    break;
                case LU:
                    row = current.first - 1;
                    col = current.second - 1;
                    break;
            }
            // if its index is out of range or its a wall, represented by -1 in the matrix continue.
            if (row < 0 || row >= dimension || col < 0 || col >= dimension || array[row][col] < 0 || discovered[std::pair<int,int>(row,col)])
                continue;
            (*bfs)[row][col] = (*bfs)[current.first][current.second] + 1;
            q.push(std::pair<int,int>(row,col));
            discovered[std::pair<int,int>(row,col)] = true;
        }
    }
    for(int i=0;i<dimension;++i){
        for(int j=0;j<dimension;++j){
            std::cout << (*bfs)[i][j] << ',';
        }
        std::cout << std::endl;
    }

    return bfs;
}

double bfs_distance(const pair<int, int> &p1, const pair<int, int> &p2,double min_val ,double **array, int dimension, int *goal){
    static shared_ptr<vector<shared_ptr<int[]>>> bfs_matrix = bfs(array,dimension,goal);
    return min_val*(*bfs_matrix)[p1.first][p2.second];
}
