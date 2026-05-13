/* ================================================================
 * astar.c — A* Path Planning Implementation
 * CSC 2207 Robotics | GRP2 | Lecture 4 Practical
 * ================================================================
 *
 * ALGORITHM (from lecture notes, slides 19-22):
 *
 * Step 1 — Initialise
 *   open_list  = { start }
 *   closed_list = {}
 *   g[start] = 0,  f[start] = h(start, goal)
 *
 * Step 2 — Loop
 *   current = node in open_list with lowest f(n)
 *   if current == goal  → reconstruct and return path
 *   move current to closed_list
 *   for each neighbour of current:
 *     skip if in closed_list or is obstacle
 *     tentative_g = g[current] + 1       (uniform step cost)
 *     if tentative_g < g[neighbour]:
 *       g[neighbour] = tentative_g
 *       f[neighbour] = tentative_g + h(neighbour, goal)
 *       parent[neighbour] = current
 *       add/update neighbour in open_list
 *
 * Step 3 — open_list empty → no path exists
 * ================================================================ */

#include <stdio.h>
#include <string.h>
#include "astar.h"

/* ------------------------------------------------------------------
 * MAP DEFINITION
 * 10×10 grid:  0=free   1=obstacle
 *
 * Visual layout (S=start row0,col0  G=goal row9,col9):
 *
 *   col  0 1 2 3 4 5 6 7 8 9
 * row 0 [ S . . . . . . . . . ]
 * row 1 [ . . X X . . . . . . ]
 * row 2 [ . . X . . . . . . . ]
 * row 3 [ . . X . X X . . . . ]
 * row 4 [ . . . . X . . . . . ]
 * row 5 [ . . . . . . X X . . ]
 * row 6 [ . . . . . . X . . . ]
 * row 7 [ . . . . . . . . . . ]
 * row 8 [ . . . . . . . . . . ]
 * row 9 [ . . . . . . . . . G ]
 * ------------------------------------------------------------------ */
int grid_map[GRID_ROWS][GRID_COLS] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

/* ------------------------------------------------------------------ */
/* Direction vectors: Up, Down, Left, Right (4-connected grid)         */
/* ------------------------------------------------------------------ */
static const int DR[4] = {-1,  1,  0, 0};
static const int DC[4] = { 0,  0, -1, 1};

/* ------------------------------------------------------------------ */
/* Helper functions                                                     */
/* ------------------------------------------------------------------ */
static int in_bounds(int r, int c)
{
    return (r >= 0 && r < GRID_ROWS && c >= 0 && c < GRID_COLS);
}

static int cell_eq(Cell a, Cell b)
{
    return (a.row == b.row && a.col == b.col);
}

/* Manhattan distance — the heuristic h(n) from the lecture */
static int heuristic(Cell a, Cell b)
{
    int dr = a.row - b.row;
    int dc = a.col - b.col;
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

/* ------------------------------------------------------------------ */
/* Open list (array + linear scan — fine for 10×10)                    */
/* For large maps replace with a binary min-heap.                      */
/* ------------------------------------------------------------------ */
#define MAX_OPEN (GRID_ROWS * GRID_COLS * 2)
static AStarNode open_list[MAX_OPEN];
static int       open_size = 0;

static void open_push(AStarNode node)
{
    if (open_size < MAX_OPEN)
        open_list[open_size++] = node;
}

/* Pop the node with the lowest f value */
static AStarNode open_pop_min(void)
{
    int best = 0;
    for (int i = 1; i < open_size; i++)
        if (open_list[i].f < open_list[best].f)
            best = i;

    AStarNode result = open_list[best];
    open_list[best]  = open_list[--open_size]; /* fill gap with last */
    return result;
}

/* Returns index of cell in open list, or -1 */
static int open_find(Cell c)
{
    for (int i = 0; i < open_size; i++)
        if (cell_eq(open_list[i].cell, c)) return i;
    return -1;
}

/* ------------------------------------------------------------------ */
/* Search tables                                                        */
/* ------------------------------------------------------------------ */
static int  closed[GRID_ROWS][GRID_COLS];
static int  g_cost[GRID_ROWS][GRID_COLS];
static Cell parent_map[GRID_ROWS][GRID_COLS];

/* ================================================================
 * astar_plan() — main planner
 * Returns a Path struct; path.length == 0 means failure.
 * ================================================================ */
Path astar_plan(Cell start, Cell goal)
{
    Path result;
    result.length = 0;

    /* ---- initialise tables ---- */
    memset(closed, 0, sizeof(closed));
    open_size = 0;

    for (int r = 0; r < GRID_ROWS; r++)
        for (int c = 0; c < GRID_COLS; c++) {
            g_cost[r][c]         = INF;
            parent_map[r][c].row = -1;
            parent_map[r][c].col = -1;
        }

    /* ---- push start node ---- */
    g_cost[start.row][start.col] = 0;

    AStarNode s;
    s.cell      = start;
    s.g         = 0;
    s.f         = heuristic(start, goal);
    s.parent    = (Cell){-1, -1};
    open_push(s);

    /* ================================================================
     * MAIN SEARCH LOOP
     * ================================================================ */
    while (open_size > 0) {

        /* Step 1: pick node with lowest f(n) */
        AStarNode cur = open_pop_min();

        /* Already processed? (can happen if updated in open list) */
        if (closed[cur.cell.row][cur.cell.col]) continue;

        /* Step 2: move to closed list, record parent */
        closed[cur.cell.row][cur.cell.col]     = 1;
        parent_map[cur.cell.row][cur.cell.col] = cur.parent;

        /* Step 3: goal check → reconstruct path */
        if (cell_eq(cur.cell, goal)) {
            Cell  rev[MAX_PATH];
            int   len = 0;
            Cell  trace = goal;

            while (trace.row != -1 && len < MAX_PATH) {
                rev[len++] = trace;
                trace = parent_map[trace.row][trace.col];
            }
            /* reverse into result (start→goal order) */
            for (int i = len - 1; i >= 0; i--)
                result.steps[result.length++] = rev[i];

            return result;
        }

        /* Step 4: expand neighbours */
        for (int d = 0; d < 4; d++) {
            int nr = cur.cell.row + DR[d];
            int nc = cur.cell.col + DC[d];

            if (!in_bounds(nr, nc))        continue;
            if (grid_map[nr][nc] == 1)     continue; /* obstacle */
            if (closed[nr][nc])            continue;

            int tg = cur.g + 1; /* uniform cost: every step costs 1 */

            if (tg < g_cost[nr][nc]) {
                g_cost[nr][nc] = tg;

                AStarNode nb;
                nb.cell   = (Cell){nr, nc};
                nb.g      = tg;
                nb.f      = tg + heuristic(nb.cell, goal);
                nb.parent = cur.cell;

                int idx = open_find(nb.cell);
                if (idx >= 0)
                    open_list[idx] = nb; /* update with better cost */
                else
                    open_push(nb);
            }
        }
    }

    printf("[A*] ERROR: No path found from (%d,%d) to (%d,%d)\n",
           start.row, start.col, goal.row, goal.col);
    return result; /* length == 0 */
}

/* ================================================================
 * astar_print_map() — ASCII debug visualisation
 * ================================================================ */
void astar_print_map(const Path *path, Cell start, Cell goal)
{
    int on_path[GRID_ROWS][GRID_COLS];
    memset(on_path, 0, sizeof(on_path));
    for (int i = 0; i < path->length; i++)
        on_path[path->steps[i].row][path->steps[i].col] = 1;

    printf("\n========== A* GRID MAP ==========\n");
    printf("S=start  G=goal  *=path  X=obstacle  .=free\n\n");

    printf("     ");
    for (int c = 0; c < GRID_COLS; c++) printf("%2d", c);
    printf("  <- col\n");

    for (int r = 0; r < GRID_ROWS; r++) {
        printf("r%2d  ", r);
        for (int c = 0; c < GRID_COLS; c++) {
            Cell here = {r, c};
            if      (cell_eq(here, start))    printf(" S");
            else if (cell_eq(here, goal))     printf(" G");
            else if (grid_map[r][c] == 1)     printf(" X");
            else if (on_path[r][c])           printf(" *");
            else                              printf(" .");
        }
        printf("\n");
    }
    printf("\nTotal path length : %d cells  (%d moves)\n",
           path->length, path->length > 0 ? path->length - 1 : 0);
    printf("=================================\n\n");
}
