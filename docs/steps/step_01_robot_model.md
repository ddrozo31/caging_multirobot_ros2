# Step 1 — Robot Model in Gazebo Harmonic

## Objective

Build a simple model of the **Yahboom ROSMASTER X3** (the robot used in the paper), check it in
RViz, and spawn it in Gazebo Harmonic. The robot does **not** move yet: motion comes in Step 2.

## Paper reference

- §2.1.1 — Omnidirectional mobile robot (Yahboom ROSMASTER X3, four mecanum wheels).
- §2.1.3, Fig. 6 — Robot frame {R}, wheel numbering $i = 0..3$, distances $l_x$, $l_y$.
- [paper_reference.md](../paper_reference.md) §8 — robot dimensions and derived $r_w$, $l_x$, $l_y$.

## Theory

### URDF and xacro

- **URDF** (Unified Robot Description Format) is an XML file that describes a robot as a tree of
  **links** (rigid bodies) connected by **joints**.
- Each link can have three parts:
  - `<visual>`: what we see,
  - `<collision>`: the shape used by the physics engine,
  - `<inertial>`: mass and inertia matrix (needed by the simulator).
- **xacro** (XML macros) adds variables (`xacro:property`), math (`${lx/2}`), includes and
  macros (`xacro:macro`) to URDF. Running `xacro` produces a plain URDF.

### Joint types used here

| Type | Moves? | Used for |
|------|--------|----------|
| `fixed` | no | chassis attached to `base_link` |
| `continuous` | rotates without limits | the four wheels |

### Frames of our robot

```
            +X (forward)
              ^
     wheel_0  |  wheel_1          0 = front-left,  1 = front-right
       (FL)   |   (FR)            2 = rear-left,   3 = rear-right
              |                   (numbering of Fig. 6 in the paper)
   +Y <=======o base_link
              |
     wheel_2  |  wheel_3
       (RL)   |   (RR)
```

- `base_link` is on the **ground**, at the center of the four wheels. Its $(x, y, \theta)$ is the
  robot pose $q$ used by all controllers in the paper (frame {R}).
- Each wheel frame is rotated by $-\pi/2$ around X, so the wheel axis points to $+Y$.
  With `axis = 0 0 1`, a **positive** wheel speed moves the robot **forward** for all four
  wheels — exactly row 1 of Eq. 1: $\dot x_R = \frac{r_w}{4}(\dot\theta_0+\dot\theta_1+\dot\theta_2+\dot\theta_3)$.

### Simplifications

- The body is one box; the wheels are plain cylinders. **Mecanum rollers are not modelled.**
- The wheels have **zero friction**: they only carry the weight. In Step 2 the chassis motion
  will be produced by the kinematic model (Eq. 1):

  ```
  desired twist ──(inverse Eq. 1)──► 4 wheel speeds ──► ros2_control spins the wheels
  wheel speeds  ──(forward Eq. 1)──► body twist     ──► Gazebo moves the chassis
  ```

### URDF vs. Gazebo settings

URDF has no friction field. Gazebo settings go inside `<gazebo reference="link">` blocks
(file `gazebo.xacro`). A common mistake is writing SDF tags such as `<surface><friction>`
inside a URDF `<collision>`: the URDF parser silently **drops** them.

## Files created

```
src/
├── caging_description/            (ament_python)
│   ├── package.xml, setup.py, setup.cfg, resource/
│   ├── urdf/
│   │   ├── robot.urdf.xacro       # top level: only includes the other files
│   │   ├── properties.xacro       # colors
│   │   ├── inertial_macros.xacro  # inertia of box / cylinder / sphere
│   │   ├── robot_core.xacro       # links and joints (wheel macro used 4 times)
│   │   └── gazebo.xacro           # Gazebo-only settings (wheel friction)
│   ├── launch/
│   │   ├── rsp.launch.py          # xacro -> URDF -> robot_state_publisher
│   │   └── display.launch.py      # rsp + joint sliders + RViz
│   └── rviz/display.rviz
└── caging_gazebo/                 (ament_python)
    ├── package.xml, setup.py, setup.cfg, resource/
    ├── worlds/empty_world.sdf     # ground, light, physics
    ├── config/topic_bridge.yaml   # list of topics bridged between ROS 2 and Gazebo
    └── launch/sim.launch.py       # Gazebo + rsp + spawn + bridge
```

### The ROS 2 ↔ Gazebo bridge (`config/topic_bridge.yaml`)

ROS 2 and Gazebo have **separate** message systems. `ros_gz_bridge` copies messages between
them, and every topic that must cross over is listed in one YAML file:

```yaml
- ros_topic_name: "/clock"                 # name on the ROS 2 side
  gz_topic_name: "/clock"                  # name on the Gazebo side (gz topic -l)
  ros_type_name: "rosgraph_msgs/msg/Clock" # ROS 2 message type
  gz_type_name: "gz.msgs.Clock"            # Gazebo message type
  direction: GZ_TO_ROS                     # GZ_TO_ROS | ROS_TO_GZ | BIDIRECTIONAL
```

In this step only the simulation clock is bridged. Each following step adds its topics to this
same file. Use absolute names (starting with `/`): relative names get the node namespace added.

### Model parameters (`robot_core.xacro`)

| Parameter | Value | Source |
|-----------|-------|--------|
| `wheel_radius` ($r_w$) | 0.0325 m | 65 mm diameter |
| `wheel_width` | 0.0304 m | drawing |
| `lx` ($l_x$) | 0.080 m | 160 mm wheelbase / 2 |
| `ly` ($l_y$) | 0.0845 m | (199.4 − 30.4) mm / 2 |
| chassis | 0.238 × 0.12 × 0.06 m | length from drawing, width fits between wheels |
| masses | chassis 2.0 kg, wheel 0.1 kg | approximate |

### Note on `ament_python` packages with data files

These two packages have no Python code, only files to install. In `setup.py`, the
`data_files` list says where each file goes inside `install/`. `glob('urdf/*.xacro')` only
matches files of **one** folder: every new sub-folder needs its own line.

## How to build and run

```bash
cd ~/caging_multirobot_ros2
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

**1. Check the model in RViz** (move the wheel sliders and watch the white stripes):

```bash
ros2 launch caging_description display.launch.py
```

**2. See the generated URDF and the tree of links:**

```bash
xacro install/caging_description/share/caging_description/urdf/robot.urdf.xacro > /tmp/x3.urdf
check_urdf /tmp/x3.urdf
```

**3. Spawn the robot in Gazebo:**

```bash
ros2 launch caging_gazebo sim.launch.py
```

## Expected result

- `check_urdf` shows `base_link` with 5 children: `chassis_link` and `wheel_0_link` … `wheel_3_link`.
- In RViz, a positive slider value on any wheel turns it in the "forward" direction.
- In RViz, each grid cell is **1 m**: the robot (0.238 m long) covers about a quarter of a cell.
  The model is at real scale because ROS uses SI units (REP-103) and all the paper's
  parameters are in meters.
- In Gazebo, the robot rests on the ground at the origin with the red marker pointing to $+X$.
- In Gazebo, the wheels do not publish joint states yet (that comes with ros2_control in Step 2).
- `ros2 topic echo --once /clock` prints the simulation time (the bridge works).

## Exercises

1. Run `gz sdf -p /tmp/x3.urdf` to convert the URDF to SDF. Where did `chassis_link` go?
   (Hint: Gazebo merges links connected by `fixed` joints.) Find the `<mu>` of each wheel.
2. Move `<mu1>`/`<mu2>` from `gazebo.xacro` into the `<collision>` of the wheel as
   `<surface><friction><ode><mu>…`. Convert to SDF again: what happened to the friction?
3. Compute by hand the inertia `izz` of the chassis box with the formula in
   `inertial_macros.xacro` and compare with the generated URDF.
4. Using the wheel frame rotation ($-\pi/2$ about X) and the right-hand rule, explain why a
   positive wheel speed moves the robot forward.
5. With the values in the table, compute $l_x + l_y$ and write the numeric matrix of Eq. 1.
6. With the simulation running, use `gz topic -l` and `ros2 topic list`. Which topics exist
   only in Gazebo? Which one appears on both sides, and why?
