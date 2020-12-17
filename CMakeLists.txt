# cmake_minimum_required(VERSION <specify CMake version here>)
project(AI_Project)

set(CMAKE_CXX_STANDARD 11)
# delete this line
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
add_executable(AI_Project main.cpp Algorithms/SearchAlgorithms.cpp Algorithms/SearchAlgorithms.h Algorithms/Node.cpp Algorithms/Node.h Algorithms/SearchAlgorithmBaseClass.h Algorithms/IterativeDeepeningSearch.cpp Algorithms/IterativeDeepeningSearch.h Algorithms/AStarSearch.cpp Algorithms/AStarSearch.h Algorithms/HeuristicSearch.h Algorithms/IDAStarSearch.cpp Algorithms/IDAStarSearch.h Algorithms/BiDirectionalAStar.cpp Algorithms/BiDirectionalAStar.h Algorithms/HeuristicSearch.cpp)