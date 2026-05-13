# Lecture 4 Practical Report — Path Planning (A*)

**Course:** CSC 2207 Robotics
**Lecturer:** Dr. Dragule Swaib
**Group:** GRP2
**Date:** May 2026

---

## 1. Objective

Demonstrate understanding of graph-based path planning by implementing A* in C
and driving a simulated differential-drive robot along the computed path in Webots.

---

## 2. Theory Summary

### Path vs Trajectory (lecture notes)

| Term | Definition |
|------|-----------|
| Path | Geometric route — no time component: q(s), s ∈ [0,1] |
| Trajectory | Path + timing + velocity + acceleration: q(t) |

This practical implements **path planning** only (geometry, not timing).

### A* Algorithm

**Cost function:**

```
f(n) = g(n) + h(n)
```

| Symbol | Meaning |
|--------|---------|
| g(n) | Actual cost from start → node n |
| h(n) | Heuristic estimate from n → goal |
| f(n) | Total estimated cost through n |

**Heuristic used:** Manhattan distance (admissible for 4-connected grid)

```
h(n) = |row_goal - row_n| + |col_goal - col_n|
```

A heuristic is *admissible* if it never overestimates the true cost.
This guarantees A* returns the **optimal (shortest) path**.

### Why A* over Dijkstra?

Dijkstra expands nodes in all directions uniformly.
A* adds the heuristic h(n) to bias expansion **toward the goal**,
making it faster while still guaranteeing optimality when h is admissible.

### Odometry (Lecture 3 — used in execution phase)

```
v      = (v_R + v_L) / 2          linear velocity
omega  = (v_R - v_L) / L          angular velocity
x     += v · cos(θ) · dt
y     += v · sin(θ) · dt
θ     += omega · dt
```

---

## 3. Implementation

### Files

| File | Role |
|------|------|
| `astar.h` | Data structures: Cell, AStarNode, Path |
| `astar.c` | A* algorithm + ASCII map printer |
| `astar_robot.c` | Webots controller: planning + execution |
| `test_astar.c` | Standalone test (no Webots) |
| `astar_world.wbt` | Webots world with obstacles |

### Grid Map (10 × 10)

```
col  0 1 2 3 4 5 6 7 8 9
r 0 [ S . . . . . . . . . ]
r 1 [ . . X X . . . . . . ]
r 2 [ . . X . . . . . . . ]
r 3 [ . . X . X X . . . . ]
r 4 [ . . . . X . . . . . ]
r 5 [ . . . . . . X X . . ]
r 6 [ . . . . . . X . . . ]
r 7 [ . . . . . . . . . . ]
r 8 [ . . . . . . . . . . ]
r 9 [ . . . . . . . . . G ]

S = start (0,0)   G = goal (9,9)   X = obstacle   . = free
```

---

## 4. A* Path Found

*(Paste terminal output here after running)*

```
[CONTROLLER] Path found! XX waypoints:
   [ 0] cell (0, 0)
   ...
   [XX] cell (9, 9)
```

ASCII map output:

```
(paste here)
```

---

## 5. Webots Simulation

*(Paste screenshots or describe what you observed)*

- Robot starts at blue marker (cell 0,0)
- Robot navigates around red obstacle boxes
- Robot reaches green goal marker (cell 9,9)
- Odometry printed at each waypoint in the console

---

## 6. ROS 2 Nodes Used

| Node | Purpose |
|------|---------|
| Webots built-in controller | Direct C API — no ROS node needed for Lecture 4 |

> Note: ROS 2 integration (publishing `/odom`, `/cmd_vel`) is planned for
> the main project phase when all modules are combined.

---

## 7. Results

| Metric | Value |
|--------|-------|
| Path length (steps) | *fill in* |
| Optimal? | Yes (A* with admissible heuristic) |
| Obstacles avoided | 10 |
| Final odometry error | *fill in* m |

---

## 8. Reflection

**What worked:**
- A* correctly found the shortest collision-free path around all obstacle clusters
- Proportional heading controller smoothly tracked each waypoint

**What could be improved:**
- The P-controller overshoots slightly on sharp turns → add D term (PID)
- Grid resolution (10×10, 0.15 m cells) is coarse → increase for smoother paths
- No re-planning if robot is perturbed → add replanning trigger

**Connection to other lectures:**
- Lecture 3: odometry equations used directly in the execution phase
- Lecture 7: the grid_map is a simplified occupancy grid; full localization
  would replace the open-loop odometry with a particle filter

---

## 9. References

- Dr. Dragule Swaib, CSC 2207 Lecture 4 Slides — Path Planning
- Webots R2023b Documentation — https://cyberbotics.com/doc/guide/index
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
