#include <iostream>
#include <fstream>
#include <cstring>
#include <algorithm>
#include "Algorithms/SearchAlgorithms.h"

using namespace std;

int main(int argc, char *argv[]) {
    string algo, dimension, sourceStr, targetStr;
    auto *source = new int[2];
    auto *target = new int[2];
    if (argc < 2) {
        exit(1);
    }
    ifstream inputFile(argv[1]);
    getline(inputFile, algo, '\n');
    getline(inputFile, dimension, '\n');
    getline(inputFile, sourceStr, '\n');
    getline(inputFile, targetStr, '\n');
    source[0] = stoi(sourceStr.substr(0,sourceStr.find(',')));
    sourceStr.erase(0,sourceStr.find(',') + 1);
    source[1] = stoi(sourceStr);
    target[0] = stoi(targetStr.substr(0,sourceStr.find(',')));
    targetStr.erase(0,targetStr.find(',') + 1);
    target[1] = stoi(targetStr);
    int d = stoi(dimension);
    string tmpStr;
    auto **array = new int *[d];
    for (int i = 0; i < d; ++i) {
        array[i] = new int[d];
        getline(inputFile, tmpStr, '\n');
        tmpStr.erase(remove(tmpStr.begin(),tmpStr.end(),' '),tmpStr.end());
        size_t pos = 0;
        string token;
        for (int j = 0; j < d; ++j) {
            pos = tmpStr.find(',');
            token = tmpStr.substr(0, pos);
            array[i][j] = stoi(token);
            tmpStr.erase(0, pos + 1);
        }
    }
    uniformCostSearch(array,d,source,target);

    return 0;
}
