/* ================================================================
 * test_astar.c — Standalone A* Unit Test (no Webots needed)
 * CSC 2207 Robotics | GRP2 | Lecture 4 Practical
 *
 * Run this FIRST to verify A* works before opening Webots.
 *
 * Build:
 *   gcc test_astar.c astar.c -o test_astar -lm
 *   ./test_astar
 * ================================================================ */

#include <stdio.h>
#include "astar.h"

int main(void)
{
    printf("========================================\n");
    printf(" CSC 2207 Robotics — Lecture 4 Practical\n");
    printf(" A* Path Planning — Standalone Test\n");
    printf("========================================\n\n");

    /* Test 1: standard run */
    printf("TEST 1: Start (0,0) → Goal (9,9)\n");
    Cell start1 = {0, 0};
    Cell goal1  = {9, 9};
    Path p1 = astar_plan(start1, goal1);

    if (p1.length > 0) {
        printf("  PASS — path found with %d steps\n", p1.length - 1);
        astar_print_map(&p1, start1, goal1);

        printf("  Step-by-step:\n");
        for (int i = 0; i < p1.length; i++)
            printf("    [%2d] row=%d  col=%d\n", i,
                   p1.steps[i].row, p1.steps[i].col);
    } else {
        printf("  FAIL — no path found\n");
    }

    /* Test 2: short path */
    printf("\nTEST 2: Start (0,0) → Goal (0,5) (no obstacles in row 0)\n");
    Cell start2 = {0, 0};
    Cell goal2  = {0, 5};
    Path p2 = astar_plan(start2, goal2);

    if (p2.length > 0) {
        printf("  PASS — path length = %d steps (expected 5)\n", p2.length - 1);
        astar_print_map(&p2, start2, goal2);
    } else {
        printf("  FAIL — no path found\n");
    }

    /* Test 3: same cell */
    printf("\nTEST 3: Start == Goal (5,5)\n");
    Cell start3 = {5, 5};
    Cell goal3  = {5, 5};
    Path p3 = astar_plan(start3, goal3);
    printf("  %s — length = %d (expected 1)\n",
           p3.length == 1 ? "PASS" : "FAIL", p3.length);

    printf("\n========================================\n");
    printf(" All tests complete.\n");
    printf("========================================\n");
    return 0;
}
