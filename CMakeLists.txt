# cmake_minimum_required(VERSION <specify CMake version here>)
project(AI_Project)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_executable(AI_Project main.cpp Algorithms/Node.cpp Algorithms/Node.h Algorithms/AbstractSearchAlgorithm.h Algorithms/IterativeDeepeningSearch.cpp Algorithms/IterativeDeepeningSearch.h Algorithms/AStarSearch.cpp Algorithms/AStarSearch.h Algorithms/HeuristicSearch.h Algorithms/IDAStarSearch.cpp Algorithms/IDAStarSearch.h Algorithms/BiDirectionalAStar.cpp Algorithms/BiDirectionalAStar.h Algorithms/HeuristicSearch.cpp Algorithms/Utils.cpp Algorithms/Utils.h Algorithms/AbstractSearchAlgorithm.cpp Algorithms/UniformCostSearch.cpp Algorithms/UniformCostSearch.h Algorithms/PairHashing.h Algorithms/HeuristicFunctions.h Algorithms/HeuristicFunctions.cpp)
