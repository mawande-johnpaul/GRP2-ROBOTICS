#ifndef ASTAR_H
#define ASTAR_H

/* ================================================================
 * astar.h — A* Path Planning
 * CSC 2207 Robotics | GRP2 | Lecture 4 Practical
 * ================================================================
 *
 * THEORY (lecture notes, slide 18):
 *
 *   f(n) = g(n) + h(n)
 *   g(n) = cost from start to current node
 *   h(n) = Manhattan distance heuristic to goal
 *   f(n) = total estimated path cost
 *
 *   A* always expands the node with the LOWEST f(n).
 *   Manhattan heuristic: h(n) = |row_goal-row| + |col_goal-col|
 *   Admissible → never overestimates → guarantees optimal path.
 * ================================================================ */

#define GRID_ROWS  10
#define GRID_COLS  10
#define MAX_PATH   200
#define INF        999999

typedef struct { int row; int col; } Cell;

typedef struct {
    Cell cell;
    int  g;       /* cost from start                    */
    int  f;       /* g + h  — open-list priority        */
    Cell parent;  /* for path reconstruction            */
} AStarNode;

typedef struct {
    Cell steps[MAX_PATH];
    int  length;  /* 0 = no path found                  */
} Path;

/*
 * grid_map[row][col]:  0=free  1=obstacle
 * Mirrors Lecture 7 occupancy grid values (0.0=free, 1.0=wall).
 */
extern int grid_map[GRID_ROWS][GRID_COLS];

Path astar_plan(Cell start, Cell goal);
void astar_print_map(const Path *path, Cell start, Cell goal);

#endif /* ASTAR_H */
