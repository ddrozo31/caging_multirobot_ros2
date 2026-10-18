# Step 4 — LQR Pose Controller for One Robot

## Objective

Drive one robot **automatically** from its current pose $q = [x, y, \theta]^T$ to a goal pose
$q^* = [x^*, y^*, \theta^*]^T$ with the LQR controller of the paper. The pose comes from the
ground truth (our "virtual Vicon", Step 3). The goal is given either by launch parameters
(automatic tests) or with the RViz **2D Goal Pose** button (manual).

This controller is used in every later stage of the paper: approach (Step 6), caging (Step 7)
and, for the leader, the whole transport (Steps 8–9).

> **Struggling with the matrices?** Read the companion
> [step_04_lqr_math.md](step_04_lqr_math.md): matrix toolkit, LQR derivation from scalar to
> matrices, and worked examples (integrator, cart with mass, differential drive, and one full
> control step of our mecanum robot down to the wheel speeds).

## Paper reference

- §2.1.4 — discretization: Eq. 5 (Euler) and Eq. 6 (discrete model).
- §2.2 — LQR controller: Eq. 7 (feedback linearization), Eq. 8 (linear model), Eq. 9 (cost),
  Eq. 10 (control law), Eq. 11 (gain), Eq. 12 (Riccati equation).
- Table 1 — $Q = 0.1\,I_{3\times3}$, $R = 0.8\,I_{3\times3}$.

## Theory

### 1. From the robot to a linear system (Eq. 4–8)

The global-frame model (Eq. 4) is $\dot q_G = H(\theta)\,\mu$, with $\mu$ the four wheel speeds.
Discretized with Euler and sample time $T_s$ (Eq. 5–6):

$$q_{(k)} = q_{(k-1)} + \underbrace{T_s H(\theta)}_{B(\theta)}\,\mu_{(k-1)}$$

This is **nonlinear** because $B$ depends on $\theta$. The trick of Eq. 7 (*feedback
linearization*) is to choose the wheel speeds as

$$\mu = B(\theta)^{-1}\,v \tag{7}$$

so that $B(\theta)\,\mu = v$ and the model becomes

$$q_{(k)} = q_{(k-1)} + v_{(k-1)}, \qquad A_l = B_l = I_{3\times3} \tag{8}$$

The new input $v$ is simply **how much the pose changes in one sample** (meters and radians
*per step*). It is not a velocity: the velocity in {G} is $v / T_s$.

($B$ is 3×4, so $B^{-1}$ means the pseudo-inverse. Our C++ controller already computes the
wheel speeds from a body twist with inverse Eq. 1, so we split Eq. 7 in two parts:)

| Part of $\mu = (T_s R(\theta) J_R)^{-1} v$ | Where |
|---|---|
| divide by $T_s$: $v/T_s$ = velocity in {G} | `lqr_controller.py` |
| rotate with $R(\theta)^T$ (inverse Eq. 3): velocity in {R} | `lqr_controller.py` (`global_to_robot`) |
| $J_R^{-1}$ (inverse Eq. 1): body twist → 4 wheel speeds | `mecanum_controller.cpp` (Step 2) |

### 2. LQR on the linear model (Eq. 9–12)

LQR picks the input that minimizes

$$J = \sum_{k=0}^{\infty} \left( q_{(k)}^T Q\, q_{(k)} + v_{(k)}^T R\, v_{(k)} \right) \tag{9}$$

$Q$ punishes being far from the goal, $R$ punishes big inputs. The answer is a constant gain:

$$v = K\,(q^* - q) \tag{10}$$
$$K = (R + B_l^T P B_l)^{-1} B_l^T P A_l \tag{11}$$
$$P = A_l\left(P - P B_l (B_l^T P B_l + R)^{-1} B_l^T P\right)A_l + Q \tag{12}$$

`core/lqr.py` solves Eq. 12 by **iteration**: start with $P = Q$, plug it into the right-hand
side, repeat until $P$ stops changing.

**Hand check.** With $A_l = B_l = I$ and $Q = qI$, $R = rI$, each axis is the same scalar
problem. Eq. 12 reduces to $P^2 - qP - qr = 0$:

$$P = \frac{q + \sqrt{q^2 + 4qr}}{2} = 0.3372, \qquad K = \frac{P}{r + P} = 0.2965$$

for $q = 0.1$, $r = 0.8$. So $K = 0.2965\,I$: every sample, the controller asks to cover
about **30 %** of the remaining error. The error shrinks as $(1 - K)^k = 0.7035^k$.

The heading error $\theta^* - \theta$ is wrapped to $[-\pi, \pi]$, so the robot always turns
the short way.

### 3. The sample time $T_s$ matters (and the paper does not give it)

The paper defines $T_s$ but never states its value. Because $K$ is "fraction of the error per
**sample**", the response in **seconds** depends on $T_s$:

| $T_s$ | velocity gain $K/T_s$ | time constant $\tau = -T_s / \ln(1-K)$ | command for 2.5 m error |
|---|---|---|---|
| **0.05 s (20 Hz, our choice)** | 5.9 1/s | 0.14 s | 14.8 m/s |
| 0.10 s (10 Hz) | 3.0 1/s | 0.28 s | 7.4 m/s |

We use $T_s = 0.05$ s, a usual rate for a WiFi + Vicon setup (our ground truth arrives at
100 Hz). It is a parameter, so you can try others (exercise 2).

### 4. Saturation: what the real robot can do

A command of 14.8 m/s is impossible, so the velocity is **limited** before it is sent.

**Wheel limit.** The ROSMASTER X3 gearmotors give roughly 333 rpm ≈ 35 rad/s without load
(Yahboom 520 motor, 1:30; not stated in the paper, check the datasheet). With
$r_w = 0.0325$ m that is about 1.1 m/s at the wheel rim, maybe 0.8 m/s under load.

**Kinematic coupling (inverse Eq. 1).** All four wheels share that limit:

$$|\dot x_R| + |\dot y_R| + (l_x + l_y)\,|\dot\theta| \le r_w\,\omega_{max} \approx 0.8\text{–}1.1 \text{ m/s},
\qquad l_x + l_y = 0.1645 \text{ m}$$

so pure forward motion up to ≈ 0.8–1.1 m/s, diagonal ≈ 0.4–0.55 m/s per axis, pure rotation
≈ 5–7 rad/s.

**The task.** The transport circle (radius 1 m in 30 s, §2.5) needs only
$2\pi/30 \approx 0.21$ m/s and ≈ 0.21 rad/s. A small, tall robot carrying an object with soft
fingers should move smoothly.

**Our limits** (parameters in `config/lqr.yaml`):

| | limit | check |
|---|---|---|
| linear speed $\sqrt{v_x^2 + v_y^2}$ | 0.3 m/s | |
| angular speed | 1.0 rad/s | worst case $\sqrt2 \cdot 0.3 + 0.1645 \cdot 1.0 = 0.59$ m/s (diagonal + full turn), inside the wheel budget |

The linear velocity is limited as a **vector** (both components scaled by the same factor), so
the robot still moves straight toward the goal. Clipping $v_x$ and $v_y$ separately would bend
the path.

In our simulation the wheels have no motor limit, so this saturation is the only thing that
models the real hardware.

**Consequence:** far from the goal the robot moves at the limit (constant speed); the LQR gain
only shapes the last part of the motion, when $K e / T_s$ falls below the limit
(about 5 cm from the goal for 0.3 m/s).

## Files created / changed

```
src/
├── caging_control/
│   ├── caging_control/core/lqr.py          # NEW: solve_riccati (Eq. 12), lqr_gain (Eq. 11),
│   │                                       #      paper_gain (Table 1), lqr_step (Eq. 10), saturate
│   ├── caging_control/core/geometry.py     # + global_to_robot (inverse Eq. 3)
│   ├── caging_control/nodes/lqr_controller.py   # NEW: ROS 2 node
│   ├── test/test_lqr.py                    # NEW: 6 unit tests (closed form, Riccati residual, ...)
│   └── setup.py                            # + lqr_controller executable
├── caging_bringup/
│   ├── config/lqr.yaml                     # NEW: Ts, Q, R, speed limits
│   └── launch/lqr.launch.py                # NEW: sim + twist_mux/teleop + LQR
└── caging_gazebo/rviz/sim.rviz             # + "2D Goal Pose" tool and goal arrow (yellow)
```

### Data flow

```
/ground_truth/odom ──┐
                     ├─► lqr_controller ─► /cmd_vel_nav ─┐
/goal_pose ──────────┘   (every Ts)                      │  priority 10
  ▲  RViz "2D Goal Pose"                                  ├─► twist_mux ─► /mecanum_controller/cmd_vel ─► wheels
  └  or start_with_goal + goal_x/y/yaw                    │
                          joystick / keyboard ───────────┘  priority 100 / 50 (they win)
```

### Topics and parameters

| Topic | Type | Meaning |
|---|---|---|
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | pose $q$ (in) |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | goal $q^*$, frame `world` (in) |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | body twist in {R} (out) |
| `/lqr_error` | `geometry_msgs/msg/Vector3Stamped` | $q^* - q$: x, y [m], z = heading [rad] (out) |

| Parameter | Default | |
|---|---|---|
| `sample_time` | 0.05 | $T_s$ [s] |
| `q_weight`, `r_weight` | 0.1, 0.8 | Table 1 |
| `max_linear_speed`, `max_angular_speed` | 0.3, 1.0 | [m/s], [rad/s] |
| `start_with_goal`, `goal_x`, `goal_y`, `goal_yaw` | false, 0, 0, 0 | first goal (launch args) |
| `position_tolerance`, `yaw_tolerance` | 0.01, 0.02 | only for the "Goal reached" message |

Without a goal the node publishes nothing, so the robot stays free for teleoperation. With a
goal it publishes every $T_s$; the joystick (enable button) or keyboard still override it.

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

# Unit tests of core/ (no simulation)
colcon test --packages-select caging_control --event-handlers console_direct+

# A) Automatic test: leader start pose of the paper -> goal (0, -1, pi)
ros2 launch caging_bringup lqr.launch.py x:=0.5 y:=-2.5 yaw:=1.5708 \
    start_with_goal:=true goal_x:=0.0 goal_y:=-1.0 goal_yaw:=3.1416

# B) Manual: start without a goal, then use RViz "2D Goal Pose"
ros2 launch caging_bringup lqr.launch.py x:=0.5 y:=-2.5 yaw:=1.5708
#    click "2D Goal Pose" in the toolbar, click on the floor and drag to set the heading

# Watch it (another terminal)
ros2 topic echo /lqr_error
rqt_plot /lqr_error/vector/x /lqr_error/vector/y /lqr_error/vector/z

# Send a goal from the terminal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
```

## Expected result

Measured in a headless test (spawn $(0.5, -2.5, \pi/2)$, default parameters):

| Check | Result |
|---|---|
| Gain printed at start | $K = 0.2965\,I$, $P = 0.3372\,I$, $K/T_s = 5.93$ 1/s |
| Goal 1 (parameters): $(0, -1, \pi)$, 1.58 m + 90° away | reached in ≈ 5.5 s, final error < 1 mm, < 0.001 rad |
| Goal 2 (`/goal_pose`): $(1, 0.5, 0)$, 1.80 m + 180° away | reached in ≈ 6.1 s, final error < 1 mm |
| Distance to goal during the motion | drops 0.30 m every second (speed saturated at 0.3 m/s) |
| Heading | turns at the 1 rad/s limit until aligned, while translating |
| `/cmd_vel_nav` rate | 20.0 Hz ($T_s = 0.05$ s) |

The robot moves on a **straight line** to the goal (it is omnidirectional: it does not need to
face the goal first) while it rotates to the goal heading. In RViz the yellow arrow is the goal.

## Exercises

1. Verify the hand computation of $P$ and $K$ for Table 1. Then compute $K$ for $Q = I$,
   $R = 0.8I$ and for $Q = 0.1I$, $R = 0.01I$. What happens to $K$ when $R \to 0$? Explain with
   Eq. 9 what $R$ means for the robot.
2. Run goal A with `sample_time` 0.05, 0.1 and 0.2 (edit `config/lqr.yaml`, rebuild). Plot
   `/lqr_error`. Compare the last 10 cm of the motion with the table in theory section 3.
3. Set `max_linear_speed: 5.0` and `max_angular_speed: 10.0`. What changes? Why would this be
   dangerous on the real robot? Use the wheel budget inequality to find the wheel speed the
   first command would ask for.
4. Change `saturate` to clip $v_x$ and $v_y$ separately (each to ±0.3). Send a goal at
   $(+2, +0.5)$ from the origin and draw the path. Why is it no longer straight?
5. Remove `wrap_angle` from `lqr_step`. Send the robot from $\theta = 170°$ to
   $\theta^* = -170°$. What happens?
6. While the robot moves to a goal, take over with the keyboard
   (`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key`),
   push it away and release. Who wins, and what happens 0.5 s after you release? (Hint:
   `twist_mux` timeouts in `teleop.yaml`.)
7. The controller uses the ground truth. Change the subscription to
   `/mecanum_controller/odom` with odometry noise on (Step 3). The robot says it reached the
   goal: is it really there? Measure the difference.
