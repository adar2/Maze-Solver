# cmake_minimum_required(VERSION <specify CMake version here>)
project(AI_Project)

set(CMAKE_CXX_STANDARD 11)

add_executable(AI_Project main.cpp Algorithms/SearchAlgorithms.cpp Algorithms/SearchAlgorithms.h Algorithms/Node.cpp Algorithms/Node.h Algorithms/SearchAlgorithmInterface.h Algorithms/IterativeDeepeningSearch.cpp Algorithms/IterativeDeepeningSearch.h Algorithms/AStarSearch.cpp Algorithms/AStarSearch.h Algorithms/HeuristicSearch.h Algorithms/IDAStarSearch.cpp Algorithms/IDAStarSearch.h)