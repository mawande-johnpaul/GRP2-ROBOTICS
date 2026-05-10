# GRP2-ROBOTICS

This repository is for lecture-aligned robotics practice and one main project.

The course workflow used in this repo:
- ROS 2 Jazzy Jalisco in WSL Ubuntu 24.04
- Webots GUI on Windows
- ROS 2 and Webots connected through WSL/Windows networking

## Repository Structure

- `resources/`: lecture slides and reference materials
- `practice/`: practical exercises per lecture topic
- `project/`: the main project that will be assigned

## Environment Setup (WSL + Webots)

### 1. Install and prepare WSL Ubuntu 24.04

On Windows PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
```

Open Ubuntu and update:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install ROS 2 Jazzy in WSL

Follow the official ROS 2 Jazzy Ubuntu instructions, then install common tools:

```bash
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions git
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Webots on Windows

- Install Webots on Windows (latest stable release).
- Confirm Webots opens and sample worlds run.

### 4. ROS 2 <-> Webots integration notes

- Run ROS 2 nodes in WSL.
- Run Webots GUI in Windows.
- Use a single ROS domain for all terminals (example below).

```bash
echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
source ~/.bashrc
```

If communication issues occur:
- Verify Windows firewall allows Webots network traffic.
- Ensure both sides use the same `ROS_DOMAIN_ID`.
- Keep all terminals on the same network interface (avoid mixed VPN routes).

## Practical Plan by Lecture

The topics below are based on files in `resources/`.

### Lecture 1: Introduction to Robotics

Core ideas:
- robot types and classifications
- autonomy vs teleoperation
- robot subsystems (mechanics, sensing, actuators, control)

Practical task:
- Build a "robot taxonomy demo" in Webots where multiple robot models are shown (wheeled, legged, manipulator), and document their subsystem differences.

Suggested Webots mini-project:
- `L1_robot_types_showcase`: one world with at least 3 robot classes and a short comparison report.

### Lecture 2: Locomotion and Kinematics I

Core ideas:
- forward/inverse kinematics
- DOF and workspace
- Jacobian and end-effector velocity

Practical task:
- Implement 2-link planar arm kinematics and Jacobian-based velocity mapping.

Suggested Webots mini-project:
- `L2_2link_kinematics_lab`: move end-effector to targets and compare analytical vs simulated motion.

### Lecture 3: Mobile Robot Locomotion

Core ideas:
- differential drive equations
- odometry state `(x, y, theta)`
- PWM, encoders, PID closed-loop control

Practical task:
- Build a differential-drive controller using encoder feedback and tune PID for straight-line and turning performance.

Suggested Webots mini-project:
- `L3_diff_drive_pid`: closed-loop wheel speed control with odometry logging and error plots.

### Lecture 4: Path Planning

Core ideas:
- path vs trajectory
- graph-based planning (BFS, Dijkstra, A*)
- sampling planners (RRT, PRM)

Practical task:
- Implement grid-based A* and compare with one sampling planner (RRT or PRM) in cluttered maps.

Suggested Webots mini-project:
- `L4_planner_benchmark`: run A* and RRT on the same world, compare path length, runtime, and clearance.

### Lecture 5: Trajectory

Core ideas:
- cubic/quintic trajectories
- LSPB and spline trajectories
- smoothness, acceleration, jerk constraints

Practical task:
- Convert a geometric path into a time-parameterized trajectory and evaluate smoothness metrics.

Suggested Webots mini-project:
- `L5_trajectory_profiles`: execute cubic vs quintic (or LSPB) and compare jerk/acceleration behavior.

### Lecture 6: Perception and Sensing

Core ideas:
- proprioceptive vs exteroceptive sensing
- encoder/IMU/range sensor behavior
- sensor noise and sensor fusion concepts

Practical task:
- Fuse encoder + IMU for improved heading/odometry and add one exteroceptive sensor for obstacle-aware behavior.

Suggested Webots mini-project:
- `L6_sensor_fusion_basics`: EKF-lite or complementary fusion for heading and position stabilization.

### Lecture 7: Localization and Navigation

Core ideas:
- probabilistic localization and belief updates
- Bayesian filtering
- occupancy grids and particle filter concepts

Practical task:
- Localize a robot in a mapped corridor world using odometry + range updates.

Suggested Webots mini-project:
- `L7_particle_localization`: basic Monte Carlo localization over an occupancy grid.

## Main Project (To Be Given)

The final project details will be provided later and implemented in `project/`.

Expected integration scope:
- mobile robot locomotion + closed-loop control
- path planning
- trajectory generation
- sensing and localization
- autonomous navigation in a Webots environment

Recommended project milestones:
1. Define problem statement, map/world, robot model, and success metrics.
2. Implement baseline locomotion and odometry.
3. Add global planner (A* or equivalent).
4. Add trajectory tracking and controller tuning.
5. Add localization/sensor fusion.
6. Run end-to-end demos and record metrics.

## Suggested Weekly Workflow

1. Read the lecture slides in `resources/`.
2. Complete the matching practical in `practice/`.
3. Save screenshots, plots, and short notes in each practical folder.
4. Reuse validated modules in `project/`.

## Deliverables Template (Per Practical)

For each lecture practical, include:
- objective
- equations or algorithm summary
- ROS 2 nodes used
- Webots world and robot configuration
- result screenshots/plots
- short reflection (what worked, what failed, what to improve)

## Quick Start for Contributors

```bash
# in WSL
cd ~/path/to/GRP2-ROBOTICS

# create a ROS 2 workspace if needed
mkdir -p ws/src
cd ws
colcon build
source install/setup.bash
```

Keep practical code modular so components can be reused in the final `project/` implementation.
