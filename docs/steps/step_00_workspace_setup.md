# Step 0 — Workspace Setup

## Objective

Prepare the computer and the ROS 2 workspace that we will use during the whole workshop, and
understand how the repository is organized. No robot code is written in this step.

## Paper reference

None yet. Read §1 and §2.1 of [the paper](https://doi.org/10.1007/s10846-025-02263-y) (Tejada et al.,
2025) to get the big picture: four task
stages (start → approach → caging → transport) performed by two omnidirectional robots.

## Theory: what is a ROS 2 workspace?

A **workspace** is a folder where we develop our own ROS 2 packages. It has this shape:

```
caging_multirobot_ros2/
├── src/        # our source code: one sub-folder per package (we write here)
├── build/      # intermediate build files      (created by colcon, not versioned)
├── install/    # built packages + setup files  (created by colcon, not versioned)
└── log/        # build logs                    (created by colcon, not versioned)
```

- **colcon** is the build tool: it finds every package inside `src/`, builds it and places the
  result in `install/`.
- **Sourcing** (`source install/setup.bash`) tells the terminal where our packages are, so that
  `ros2 run` and `ros2 launch` can find them.
- ROS 2 itself is also "sourced" (`/opt/ros/jazzy/setup.bash`). Our workspace is an **overlay**
  on top of it: ROS 2 Jazzy is the **underlay**.

The folders `build/`, `install/` and `log/` are generated, so they are listed in `.gitignore`.

## Software stack

| Component | Version | Why |
|-----------|---------|-----|
| Ubuntu | 24.04 (Noble) | Official platform of ROS 2 Jazzy |
| ROS 2 | Jazzy | Robot middleware |
| Gazebo | Harmonic | Official simulator paired with Jazzy |
| ros2_control | Jazzy release | Interface between controllers and (simulated) motors |
| Python 3 + numpy | system | Most of our nodes and the math |
| C++ | system compiler | Only for our custom ros2_control controller |

## Instructions

### 1. Install ROS 2 Jazzy

Follow the official guide (Debian packages, *desktop* install):
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 2. Install the extra packages used in the workshop

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro \
  python3-colcon-common-extensions \
  python3-numpy
```

- `ros-jazzy-ros-gz` installs Gazebo Harmonic and the ROS ↔ Gazebo bridge.
- `ros-jazzy-gz-ros2-control` connects ros2_control with the simulated joints in Gazebo.
- `ros-jazzy-xacro` lets us write robot models (URDF) with variables and macros.

### 3. Get the workspace

Clone the repository. It is also the workspace:

```bash
cd ~
git clone https://github.com/ddrozo31/caging_multirobot_ros2.git
cd caging_multirobot_ros2
```

The repository has one commit and one **tag per step**: `step-00` … `step-12`, then `docker`
(the simulator in a container) and `v1.0` (the first release). The default branch
holds the finished simulator. To follow the course from the beginning, start from the empty
workspace and write each step yourself, using the tags to compare:

```bash
git switch -c my-work step-00        # your own branch, starting from the empty workspace
git diff step-04 step-05 -- src      # what Step 5 added to the code
git checkout step-05 -- src          # (if you are stuck) take the code of Step 5
```

Each tag contains the guides up to that step, in their final version; to read ahead, open the
guides on the default branch (tag `v1.0`).
No Ubuntu 24.04 at hand? [`docker/`](https://github.com/ddrozo31/caging_multirobot_ros2/blob/v1.0/docker/README.md) runs the finished simulator in a
container, with no ROS 2 installation.

### 4. Build the (still empty) workspace

```bash
source /opt/ros/jazzy/setup.bash   # 1) load ROS 2 Jazzy (underlay)
colcon build                       # 2) build every package in src/ (none yet)
source install/setup.bash          # 3) load our workspace (overlay)
```

Tip: add the first line to your `~/.bashrc` so you don't type it in every new terminal.

### 5. Check the installation

```bash
ros2 doctor --report | head -20    # ROS 2 is installed and sourced
gz sim --version                   # Gazebo Harmonic (8.x)
gz sim shapes.sdf                  # opens a test world; close it when it works
```

## Repository layout

| Path | Content |
|------|---------|
| `README.md` | What the project is, the two ways to use it, the course index |
| `docs/steps/step_XX_*.md` | One guide per step (like this one) |
| `docs/paper_reference.md` | Equations, parameters and robot dimensions from the paper |
| `docs/figures/` | Robot drawing, result plots |
| `docs/data/` | Result tables (one row per run) |
| `docker/` | The finished simulator in a container |
| `src/` | ROS 2 packages (filled from Step 1 on) |

## Expected result

- `colcon build` finishes without errors.
- `gz sim shapes.sdf` opens Gazebo Harmonic.

## Exercises

1. What is the difference between an *underlay* and an *overlay*? What happens if you run
   `ros2 launch` without sourcing `install/setup.bash`?
2. Why are `build/`, `install/` and `log/` not stored in git?
3. Read §2.1 of the paper and describe in your own words the four stages of the task.
4. Look at [paper_reference.md](../paper_reference.md) §8: how were $r_w$, $l_x$ and $l_y$ obtained
   from the robot drawing?
