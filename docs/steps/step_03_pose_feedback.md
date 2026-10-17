# Step 3 — Pose Feedback: Ground Truth ("Virtual Vicon") and Wheel Odometry

## Objective

Every controller of the paper needs the robot pose $q = [x, y, \theta]$. In this step we
produce it in two ways and compare them:

1. **Ground truth** from Gazebo — our replacement for the Vicon motion-capture system of the
   paper. This is the pose the controllers will use from Step 4 on.
2. **Wheel odometry** computed by our controller from the wheel speeds — what a robot without
   external cameras would have to rely on. We can add errors to it and watch it drift.

## Paper reference

- §2.5 — the real experiment used **Vicon** (6 cameras) to measure the pose of each robot.
- Eq. 3 — rotation $R(\theta)$ from the robot frame {R} to the global frame {G}.
- Eq. 5 — Euler integration $x_{(k)} = x_{(k-1)} + \dot x\,T_s$.

## Theory

### Two ways to know where the robot is

| | Ground truth (Vicon / Gazebo) | Wheel odometry |
|---|---|---|
| Source | external cameras / the simulator | wheel encoders + kinematics (Eq. 1) |
| Error | very small, **does not grow** | **grows with time and distance** (drift) |
| Needs | a prepared room | nothing external |
| In this project | `/ground_truth/odom` | `/mecanum_controller/odom` |

### Odometry = Eq. 1 + Eq. 3 + Eq. 5

Every control cycle ($T_s$ = 10 ms):

1. Forward Eq. 1: measured wheel speeds → twist in the robot frame $(\dot x_R, \dot y_R, \dot\theta)$.
2. Eq. 3: rotate it to the odom frame:
   $\dot x = \dot x_R\cos\theta - \dot y_R\sin\theta$, $\quad \dot y = \dot x_R\sin\theta + \dot y_R\cos\theta$.
3. Eq. 5: integrate: $x \mathrel{+}= \dot x\,T_s$, $\ y \mathrel{+}= \dot y\,T_s$, $\ \theta \mathrel{+}= \dot\theta\,T_s$.

Any error in step 1 (noisy encoders, wrong wheel radius, wheel slip) is **added up forever** by
step 3. A small heading error is the worst: every later meter is travelled in a slightly wrong
direction.

### Why our odometry is (almost) perfect unless we add errors

In our simulation the chassis is moved **by** the twist computed from the wheel speeds
(Step 2), and plain wheels cannot slip. So, without added errors, the odometry only differs from
the ground truth by the Euler integration error. To see realistic behavior, the controller can
add two errors **to the odometry only** (the real motion of the robot is not affected):

| Parameter (`ros2_controllers.yaml`) | Simulates | Default |
|---|---|---|
| `odom_noise_stddev` [rad/s] | random encoder noise (Gaussian, added to each wheel speed) | 0.0 |
| `odom_radius_scale` | systematic error: the odometry uses $r_w \times$ scale (worn tires, slip) | 1.0 |

### Frames (REP-105)

```
world ──(static: spawn pose)──► odom ──(wheel odometry)──► base_link ──► wheels
  │
  └──(Gazebo ground truth = "virtual Vicon")──► ground_truth
```

- `world`: fixed frame of the arena (the {G} frame of the paper).
- `odom`: where the odometry started. It is placed at the spawn pose, so both estimates start
  at the same point.
- `base_link`: where the robot **believes** it is. RViz draws the robot model here.
- `ground_truth`: where the robot **really** is.

A frame can only have one parent in TF, so the ground truth gets its own frame instead of also
publishing `base_link`.

## Files created / changed

```
src/
├── caging_mecanum_controller/src/mecanum_controller.cpp   # + odometry (part 6 of update()),
│   include/.../mecanum_controller.hpp                     #   ~/odom and TF odom -> base_link
├── caging_description/
│   ├── config/ros2_controllers.yaml   # + odom_frame, base_frame, publish_tf, noise, radius scale
│   └── urdf/gazebo.xacro              # + OdometryPublisher system (ground truth)
├── caging_gazebo/
│   ├── config/topic_bridge.yaml       # + /ground_truth/odom and ground-truth /tf
│   ├── launch/sim.launch.py           # + x, y, yaw, rviz args; static TF world -> odom; RViz
│   └── rviz/sim.rviz                  # NEW: fixed frame "world", robot + both odometries
├── caging_control/                    # NEW package (ament_python)
│   ├── caging_control/core/geometry.py          # wrap_angle, yaw_from_quaternion,
│   │                                            # transform_pose (Eq. 3), pose_error
│   ├── caging_control/nodes/odom_error_monitor.py   # compares both, publishes /odom_error
│   └── test/test_geometry.py                    # unit tests of core/ (no ROS needed)
└── caging_bringup/launch/odometry.launch.py     # NEW: sim + monitor with the same initial pose
```

### Topics

| Topic | Type | Frame | Meaning |
|-------|------|-------|---------|
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | `world` → `ground_truth` | true pose (Vicon) |
| `/mecanum_controller/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_link` | wheel odometry |
| `/odom_error` | `geometry_msgs/msg/Vector3Stamped` | `world` | odometry − truth: x = $e_x$, y = $e_y$, z = $e_\theta$ |
| `/tf` | `tf2_msgs/msg/TFMessage` | | `odom → base_link`, `world → ground_truth` |

### The `core/` + `nodes/` pattern (first time used)

`caging_control` separates **math** from **ROS**:

- `core/geometry.py` has no ROS imports. You can try it in a Python shell:
  ```python
  from caging_control.core.geometry import transform_pose
  transform_pose((1.0, 0.0, 0.0), (1.0, 2.0, 1.5708))   # -> (1.0, 3.0, 1.5708)
  ```
- `nodes/odom_error_monitor.py` only receives messages, converts them to tuples, calls `core/`,
  and publishes.
- `test/test_geometry.py` checks `core/` automatically:
  ```bash
  colcon test --packages-select caging_control --event-handlers console_direct+
  ```

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

# Simulation + RViz + error monitor, robot at the paper's leader start pose
ros2 launch caging_bringup odometry.launch.py x:=0.5 y:=-2.5 yaw:=1.5708

# second terminal: drive (joystick / keyboard / topic pub)
ros2 launch caging_bringup teleop.launch.py

# third terminal: inspect
ros2 topic echo /odom_error
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo world ground_truth
```

In RViz: the **green** arrows are the ground truth, the **red** arrows the wheel odometry; the
robot model is drawn where the odometry says it is.

To add errors, edit `src/caging_description/config/ros2_controllers.yaml`, then
`colcon build` and launch again. (Tip: `colcon build --symlink-install` links YAML files instead
of copying them, so you only need to relaunch.)

## Expected result

Measured in a headless test, spawned at $(0.5, -2.5, \pi/2)$, path: 0.3 m/s forward for 4 s,
0.3 m/s left for 4 s, then $(0.2, 0.1, 0.6)$ for 6 s:

| Check | Result |
|-------|--------|
| `/ground_truth/odom` at start | frame `world`, $(0.5, -2.5)$ |
| TF `world → base_link` and `world → ground_truth` at start | both $(0.500, -2.500, 1.571)$ |
| Final ground truth | $(-1.259, -1.763)$ (identical with and without odometry errors) |
| Odometry error, no added errors | **0.002 m**, heading 0.001 rad (Euler integration only) |
| Odometry error, `noise 0.5`, `scale 1.02` | **0.034 m**, heading **0.072 rad (4°)** |

With noise, the odometry drifts even while the robot stands still (the noise is integrated):
in RViz the red arrows and the robot model slowly leave the green ones.

## Exercises

1. Set `odom_radius_scale: 1.05` (no noise). Drive 2 m straight forward. Predict the
   odometry error, then measure it with `/odom_error`. Now rotate 360° in place: which error do
   you get?
2. Set `odom_noise_stddev: 1.0` (scale 1.0) and leave the robot **still** for 60 s. Plot
   `/odom_error` (`rqt_plot /odom_error/vector/x /odom_error/vector/y`). Why does the error grow
   even though the robot does not move? How does it grow with time (linearly? like $\sqrt t$?)
3. Why does the heading error cause a position error that keeps growing, while a position error
   alone does not?
4. In `transform_pose`, why do we need the spawn pose to compare the two estimates? What does
   `/odom_error` show if you launch with `x:=0.5` for the simulation but forget it for the
   monitor?
5. Change `update_rate` in `ros2_controllers.yaml` to 20 Hz (no added errors). What happens to
   the odometry error after a curved path, and why? (Hint: Eq. 5.)
6. The paper used Vicon instead of odometry. Using your results, explain why.
