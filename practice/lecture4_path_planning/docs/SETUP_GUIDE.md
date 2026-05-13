# SETUP GUIDE — ROS 2 Jazzy + Webots + A* Practical
## CSC 2207 Robotics | GRP2 | Ubuntu 24.04 LTS

---

## BEFORE YOU START — read this

Your machine: **Dell Inspiron 15-3567, Ubuntu 24.04 LTS, kernel 6.17**
You are on native Linux — no WSL needed. This is the ideal setup.

Every command block below is meant to be run in a **terminal**
(Ctrl + Alt + T). Copy each block completely, paste it, press Enter,
and wait for it to finish before moving to the next block.

---

## PART 1 — System preparation

### 1.1 Update your system

```bash
sudo apt update && sudo apt upgrade -y
```

### 1.2 Install essential build tools

```bash
sudo apt install -y \
  build-essential \
  git \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  software-properties-common \
  python3-pip \
  cmake \
  libssl-dev
```

---

## PART 2 — Install ROS 2 Jazzy

ROS 2 Jazzy is the recommended version for Ubuntu 24.04.

### 2.1 Add the ROS 2 apt repository

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 2.2 Install ROS 2 Jazzy Desktop

```bash
sudo apt install -y ros-jazzy-desktop
```

This installs ROS 2, RViz, rqt, and example packages.
It will take several minutes — the download is ~1.5 GB.

### 2.3 Install colcon and additional tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-rmw-cyclonedds-cpp
```

### 2.4 Source ROS 2 automatically in every terminal

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Verify ROS 2 installation

```bash
ros2 --version
```

Expected output: `ros2, version X.X.X`

```bash
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

You should see `Hello World: 1`, `Hello World: 2`, ...
Press Ctrl+C to stop. Kill the background talker:

```bash
kill %1
```

---

## PART 3 — Install Webots R2023b

### 3.1 Download Webots

```bash
cd ~/Downloads
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb
```

If the download is slow, you can also download it from a browser:
https://github.com/cyberbotics/webots/releases/tag/R2023b

### 3.2 Install the .deb package

```bash
sudo apt install -y ~/Downloads/webots_2023b_amd64.deb
```

### 3.3 Verify Webots installation

```bash
webots --version
```

Expected: `Webots R2023b`

Launch Webots once to confirm it opens:

```bash
webots &
```

Close it after confirming it opens correctly.

### 3.4 Set WEBOTS_HOME environment variable

```bash
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
ls $WEBOTS_HOME/lib/controller/
```

You should see `libController.so` in the listing.

---

## PART 4 — Set up the project

### 4.1 Clone or create the project directory

If using GitHub (recommended for submission):

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/GRP2-ROBOTICS.git
cd GRP2-ROBOTICS
```

**OR** if creating fresh:

```bash
mkdir -p ~/GRP2-ROBOTICS
cd ~/GRP2-ROBOTICS
git init
```

### 4.2 Copy the practical files into the project

Copy the files from wherever you received them into:

```
GRP2-ROBOTICS/
├── README.md
└── practice/
    └── lecture4_path_planning/
        ├── controllers/
        │   └── astar_robot/
        │       ├── astar.h
        │       ├── astar.c
        │       ├── astar_robot.c
        │       ├── test_astar.c
        │       └── Makefile
        ├── worlds/
        │   └── astar_world.wbt
        └── docs/
            └── PRACTICAL_REPORT.md
```

---

## PART 5 — Build and test A* (no Webots needed yet)

This step lets you confirm A* works before touching Webots.

### 5.1 Run the standalone A* test

```bash
cd ~/GRP2-ROBOTICS/practice/lecture4_path_planning/controllers/astar_robot

gcc test_astar.c astar.c -o test_astar -lm

./test_astar
```

**Expected output:**

```
========================================
 CSC 2207 Robotics — Lecture 4 Practical
 A* Path Planning — Standalone Test
========================================

TEST 1: Start (0,0) → Goal (9,9)
  PASS — path found with XX steps

========== A* GRID MAP ==========
S=start  G=goal  *=path  X=obstacle  .=free

      0 1 2 3 4 5 6 7 8 9  <- col
r 0   S * . . . . . . . .
r 1   . * X X . . . . . .
r 2   . * X . . . . . . .
r 3   . * X . X X . . . .
r 4   . * . . X . . . . .
r 5   . * . . . . X X . .
r 6   . * . . . . X . . .
r 7   . * * * * * * * * .
r 8   . . . . . . . . * .
r 9   . . . . . . . . * G

Total path length : XX cells  (XX moves)
=================================
```

If you see PASS for all 3 tests, your A* implementation is correct.

---

## PART 6 — Build the Webots controller

### 6.1 Compile

```bash
cd ~/GRP2-ROBOTICS/practice/lecture4_path_planning/controllers/astar_robot

make
```

Expected output (no errors):

```
gcc -Wall -Wextra -O2 -I/usr/local/webots/include/controller/c -c astar_robot.c -o astar_robot.o
gcc -Wall -Wextra -O2 -I/usr/local/webots/include/controller/c -c astar.c -o astar.o
gcc astar_robot.o astar.o -L/usr/local/webots/lib/controller -lController -Wl,-rpath,/usr/local/webots/lib/controller -lm -o astar_robot
```

If you see: `cannot find -lController`
→ Run: `ls /usr/local/webots/lib/controller/` and check the path.
   Then edit the Makefile WEBOTS_HOME line to match.

### 6.2 Troubleshooting the build

**Problem:** `fatal error: webots/robot.h: No such file or directory`
**Fix:**
```bash
ls /usr/local/webots/include/controller/c/webots/
# if the path is different, update WEBOTS_HOME in Makefile
```

**Problem:** linking error with libController
**Fix:**
```bash
find /usr/local/webots -name "libController.so" 2>/dev/null
# use the directory containing it in WEBOTS_HOME
```

---

## PART 7 — Run the simulation in Webots

### 7.1 Tell Webots where to find the controller

Webots looks for controllers relative to the world file.
The correct directory structure is what you already have —
Webots finds `controllers/astar_robot/astar_robot` automatically
when the world file is inside the same root folder.

### 7.2 Open the world in Webots

```bash
webots ~/GRP2-ROBOTICS/practice/lecture4_path_planning/worlds/astar_world.wbt
```

### 7.3 What you will see

1. A grey floor with a 10×10 grid
2. Red obstacle boxes in the positions from the grid map
3. A blue start marker at (0,0) — bottom-left
4. A green goal marker at (9,9) — top-right
5. A blue e-puck robot on the start marker

### 7.4 Start the simulation

- Click the **Play** button (▶) in Webots toolbar, OR
- Press **Ctrl + 3** for Run
- Press **Ctrl + 4** for Fast (no rendering delay)

### 7.5 Watch the terminal output

The Webots console (bottom panel) and your terminal will show:

```
[CONTROLLER] Running A* planner...
[CONTROLLER] Path found! 19 waypoints:
   [ 0] cell (0, 0)
   [ 1] cell (1, 0)
   ...
   [18] cell (9, 9)

========== A* GRID MAP ==========
...

[CONTROLLER] Starting navigation. 18 waypoints to follow.
[NAV] Reached waypoint 1  cell(1,0)  odom(0.021, -0.143, -2.1°)
[NAV] Reached waypoint 2  cell(2,0)  odom(0.021, -0.293, -1.8°)
...
[CONTROLLER] Goal reached! Stopping.
   Final odometry: x=0.XXX m  y=0.XXX m  theta=X.XX deg
```

---

## PART 8 — Push to GitHub

### 8.1 Initial setup (one time only)

```bash
git config --global user.name  "Your Name"
git config --global user.email "your@email.com"
```

### 8.2 Create .gitignore

```bash
cat > ~/GRP2-ROBOTICS/.gitignore << 'EOF'
# Build artefacts
*.o
*.so
*.a
controllers/astar_robot/astar_robot
controllers/astar_robot/test_astar

# Webots
*.wbproj

# ROS 2
build/
install/
log/
EOF
```

### 8.3 Commit and push

```bash
cd ~/GRP2-ROBOTICS

git add .
git commit -m "Add Lecture 4 practical: A* path planning in Webots (C)

- Implements A* algorithm from CSC 2207 Lecture 4 slides
- 10x10 grid map with 10 obstacle cells
- Differential-drive robot navigates start(0,0) to goal(9,9)
- Odometry tracking using Lecture 3 equations
- Standalone test (test_astar.c) passes all 3 test cases
- Webots world file with matching physical obstacles"

git remote add origin https://github.com/YOUR_USERNAME/GRP2-ROBOTICS.git
git push -u origin main
```

---

## PART 9 — Quick reference

| Action | Command |
|--------|---------|
| Source ROS 2 | `source /opt/ros/jazzy/setup.bash` |
| Launch Webots | `webots` |
| Build controller | `cd .../astar_robot && make` |
| Clean build | `make clean` |
| Test A* only | `gcc test_astar.c astar.c -o test_astar -lm && ./test_astar` |
| Open simulation | `webots .../worlds/astar_world.wbt` |

---

## PART 10 — Understanding the code (for the report)

### How A* maps to the lecture slides

| Lecture concept | Where in code |
|----------------|---------------|
| `f(n) = g(n) + h(n)` | `astar.c` line: `nb.f = tg + heuristic(nb.cell, goal)` |
| Manhattan heuristic | `heuristic()` function in `astar.c` |
| Open list | `open_list[]` array, `open_push()`, `open_pop_min()` |
| Closed list | `closed[][]` 2D array |
| Path reconstruction | Parent pointer trace in `astar_plan()` |
| Occupancy grid | `grid_map[][]` in `astar.c` (0=free, 1=wall) |

### How odometry maps to the lecture slides

| Lecture equation | Where in code |
|-----------------|---------------|
| `v = (vR + vL) / 2` | `double v = (dr + dl) / 2.0` |
| `ω = (vR − vL) / L` | `double omega = (dr - dl) / AXLE_LENGTH` |
| `x += v cos θ dt` | `odom_x += v * cos(odom_theta)` |
| `y += v sin θ dt` | `odom_y += v * sin(odom_theta)` |
| `θ += ω dt` | `odom_theta = normalise_angle(odom_theta + omega * dt)` |

---

*End of setup guide.*
