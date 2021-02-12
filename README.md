# Maze-Solver

Final project of University AI course.

In the project, I had to implement informed and uninformed search algorithms: UCS, IDS, A*, Bi directional A* and IDA* that are used to find a route to exit a given maze, if at all possible.

Input: a text file that contains the desired algorithm, size of the matrix, start and goal node locations and the maze itself - with each cell containing it's cost (-1 stands for a wall; all other costs are none-negative).

The agent can move in 8 directions (RU, R, RD, D, LD, L, LU, U)

Example:
```
ASTAR
17
0,0
16,16
 2, 1, 6, 2, 8, 2, 9, 4, 8, 7, 5, 9, 8, 2, 1, 3, 6
 4, 1, 2, 9,12, 2, 7, 9, 2, 1, 2, 8, 9, 3,12, 6, 7
 1, 7, 8, 9, 1, 2, 1,12,12, 2,12, 1, 2,12, 3, 7, 5
 2, 7, 6, 9,12, 5,12, 6, 3, 8, 2, 7, 9, 7, 1, 7, 1
 9, 1, 3, 5, 6, 2,12, 4, 1, 7, 4, 6, 6, 2, 8,12, 5
 2, 2, 1, 1, 6, 2, 6, 4, 9, 5, 6,12, 7, 3, 3,12, 5
 3, 2, 3,12, 1, 8,12, 4, 9, 4, 9, 2, 3, 1, 2, 3, 8
 4, 1, 4, 8, 9, 1, 9, 3, 7, 8, 5, 2, 5, 1, 4, 4, 2
-1, 3, 9, 5, 5,12, 2, 3, 2, 8,12, 3, 5, 8, 6, 3, 2
 1, 4, 8,12, 9, 6, 2, 4, 2, 2, 5, 2,12, 3, 6, 8, 4
 2, 6, 2, 7, 1, 6, 4, 3, 4, 7, 6, 2, 7, 2, 5,12, 1
 9, 3, 2, 8, 8, 6, 7, 8, 2, 2, 1, 6, 3, 6, 4, 6, 9
 9, 6, 3, 6, 7, 3, 9, 5,12, 6, 7, 1, 3, 6, 9,12, 2
 7, 4, 3, 5, 4, 3, 6, 6, 6, 7, 2, 8, 4, 1, 6, 8, 2
 1, 2, 1, 2, 5, 2, 7, 2, 9, 3, 3, 4,12, 2, 1, 7, 9
 6, 5, 2, 3, 4,12, 3, 3, 2, 2, 5, 8, 2, 1, 9, 2, 9
 3, 2, 2, 3, 2, 4, 9, 3, 5, 3, 6, 5, 8, 4, 7, 6, 7
```

Output: text file containing the solution if found under the time constraint, and series of statistics about the run of the algorithm.

Example:

```
Problem : problem_name
total nodes explored: 127
RD-LD-D-RD-RD-R-RD-RD-RD-D-RD-RD-R-R-RD-R-RD-RD-RD-RD- solution cost: 36
d/N : 0.15748
time in seconds: 0.005794
EBF : 1.27406
Min cutoff : 4
Max cutoff : 20
Avg cutoff : 8.2
Avg H Value: 10.6698
```
