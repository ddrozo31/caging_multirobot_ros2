# Step 9 — Transporting the Object Along a Trajectory

## Objective

Carry the caged box along the **circular trajectory** of the paper's experiments. The control
point is now the **object**: we say where the box must be, Eq. 26 tells where the leader must
be, the leader's LQR drives there, and the follower keeps the cage with the Leader-Follower
controller (Step 8). At the end of each lap the errors are printed next to Table 2 of the paper.

## Paper reference

- §2.4.2 — control point translation, Eq. 26–27.
- §2.5 — trajectories: circle of radius 1 m, $N = 30$ waypoints, 30 s; experiment 1 (static
  orientation), experiment 2 (orientation changing along the circle).
- §3, Table 2 — mean errors of $e_{d_O}$, $x_O$, $y_O$, $\theta_L$, $d$, $\theta_F$, $\alpha$.

## Theory

### 1. Steering from the object (Eq. 26–27)

Until now we gave goals to the **leader**. But the task is to move the **object**. Since the
object sits halfway between the robots, $d/2$ in front of the leader:

$$q_L^* = q_O^* + \frac d2\begin{bmatrix}-\cos\theta_L^*\\ -\sin\theta_L^*\\ 0\end{bmatrix},\qquad \theta_L^* = \theta_O^* \tag{26}$$

The orientation of the whole system is the leader heading: $\theta_{MRS} = \theta_O = \theta_L$.
With the corner grasp (Step 7) the leader looks along the box diagonal, and that is the
orientation we control. $d$ is the distance between the robots when the trajectory starts
(0.527 m, the $d^*$ of the Leader-Follower).

Example: object goal $(1, 0, \pi)$, $d = 0.52$ → leader goal $(1.26, 0, \pi)$: the leader stands
0.26 m to the right of the object, looking left at it.

### 2. The circle (§2.5)

$$\varphi_i = \frac{2\pi i}{N-1},\quad i = 0 \dots N-1,\qquad x^* = \cos\varphi_i,\quad y^* = \sin\varphi_i$$

30 waypoints, one every second, one lap in 30 s: about $2\pi/30 = 0.21$ m/s and 0.21 rad/s.

| `orientation` | $\theta^*$ | Paper |
|---|---|---|
| `static` | $\pi$ | experiment 1 |
| `tangent` | $\varphi_i + \pi/2$ (direction of travel) | experiment 2, as the **text** says ("tangent to the circle") |
| `radial` | $\operatorname{atan2}(y^*, x^*) = \varphi_i$ (pointing away from the center) | experiment 2, as the **formula** is printed |

The text and the formula of experiment 2 differ by 90°. We implement both; `tangent` is the
default. In `radial` the leader rides on the inside (0.74 m from the center) and the follower on
the outside (1.26 m): the follower must go 1.7 times faster than the leader.

### 3. The run

```
/start_trajectory ─► TRANSFER: object (0, 0) ─► first waypoint (1, 0, θ*(0)) at 0.1 m/s
                  ─► LAP: waypoint i at t = i · 1 s (Eq. 26 -> /leader/goal_pose), 30 s
                  ─► END: hold the last waypoint, print the errors
```

- The object starts at the origin but the circle starts at (1, 0): first a straight **transfer**
  (with the corner grasp the formation also turns 45° to $\theta^* = \pi$ in experiment 1).
- During the lap the leader may go up to 0.3 m/s / 0.5 rad/s and the follower up to 0.4 m/s
  (wheel budget: $0.4 + 0.1645 \cdot 0.5 = 0.48 < 0.8$ m/s).
- With the paper's waypoints the goal **jumps** 0.21 m every second; the LQR (a regulator, Step 4)
  chases each one. The object is always a little behind the active waypoint: this lag is most of
  $e_{d_O}$, while the object stays within millimeters of the circle itself.

### 4. A smoother motion: a moving goal instead of jumping waypoints

With waypoints the LQR drives to each goal at its limit (0.3 m/s, about 0.7 s for 0.21 m),
slows down and **waits** for the next one: stop-and-go (11 % of the lap stopped). The option
`reference: smooth` (default) moves the object goal **continuously** along the same circle,

$$\varphi(t) = 2\pi\,\frac{t}{T},\qquad q_O^*(t) = \text{circle}(\varphi(t)),\qquad \text{updated at 20 Hz}$$

so the robots drive at a constant 0.21 m/s. The waypoints of the paper are points of this
reference (`circle_pose` gives exactly the same poses at $\varphi_i$).

The LQR now follows a goal that moves at speed $v$. A regulator follows a ramp with a constant
**lag**: in steady state it moves at $v$, so $K e / T_s = v$ and

$$e = \frac{v\,T_s}{K} = \frac{0.21 \cdot 0.05}{0.2965} \approx 3.5 \text{ cm}$$

This lag is most of the remaining $e_{d_O}$ (≈ 0.047 m). A feed-forward of the goal velocity in
the LQR would remove it (exercise 3). `reference: waypoints` keeps the paper's behavior.

### 5. Errors (like Table 2)

Sampled at 20 Hz during the lap, against the waypoint active at that moment:

| Error | Definition |
|---|---|
| $e_{d_O}$ | $\sqrt{(x_O - x^*)^2 + (y_O - y^*)^2}$ |
| $e_{x_O}$, $e_{y_O}$ | $\|x_O - x^*\|$, $\|y_O - y^*\|$ |
| $e_{\theta_L}$ | $\|\theta_L - \theta^*\|$ (wrapped) |
| $e_d$, $e_{\theta_F}$, $e_\alpha$ | $\|d - d^*\|$, $\|\theta_F - (\theta_L - \pi)\|$, $\|\alpha\|$ |
| extra: circle | distance of the object to the circle (path error, no time) |
| extra: cage | distance of the object to the midpoint of the robots |

The full logging and plots for all objects are Step 10.

## Files created / changed

```
src/
├── caging_control/
│   ├── caging_control/core/trajectory.py          # NEW: circle_pose, circle_waypoints, leader_goal_from_object (Eq. 26)
│   ├── caging_control/nodes/trajectory_tracker.py # NEW: transfer, lap (smooth or waypoints), error summary
│   ├── caging_control/nodes/lqr_controller.py     # "New goal" log throttled (moving goal at 20 Hz)
│   ├── test/test_trajectory.py                    # NEW: 4 tests
│   └── setup.py                                   # + trajectory_tracker
├── caging_bringup/
│   ├── config/trajectory.yaml                     # NEW: orientation, N, duration, speeds
│   └── launch/trajectory.launch.py                # NEW: Step 8 + trajectory_tracker
└── caging_gazebo/rviz/two_robots.rviz             # + desired trajectory, current object goal
```

| Topic / service | Type | Meaning |
|---|---|---|
| `/start_trajectory` | `std_srvs/srv/Trigger` | transfer + one lap (object must be closed) |
| `/trajectory/desired_path` | `nav_msgs/msg/Path` | the 30 object waypoints (RViz, yellow) |
| `/trajectory/object_goal` | `geometry_msgs/msg/PoseStamped` | active object waypoint (RViz, axes) |
| `/leader/goal_pose` | `geometry_msgs/msg/PoseStamped` | leader goal from Eq. 26 |

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

ros2 launch caging_bringup trajectory.launch.py orientation:=static     # experiment 1
ros2 launch caging_bringup trajectory.launch.py                          # experiment 2 (tangent)
ros2 launch caging_bringup trajectory.launch.py orientation:=radial
ros2 launch caging_bringup trajectory.launch.py orientation:=static reference:=waypoints   # paper

# other terminal
ros2 service call /start_approach std_srvs/srv/Trigger       # wait for "TRANSPORT mode"
ros2 service call /start_trajectory std_srvs/srv/Trigger     # transfer + lap; summary at the end
```

## Expected result

Headless tests, box taken by the corners, Table 1 start, mean errors over the 30 s lap (Table 2,
box, in brackets).

**Smooth reference (default):**

| Error | static (exp. 1) | tangent (exp. 2) | radial |
|---|---|---|---|
| $e_{d_O}$ [m] | **0.047** [0.10] | **0.046** [0.13] | **0.047** [0.13] |
| $e_{x_O}$ / $e_{y_O}$ [m] | – | 0.030 / 0.029 [0.09 / 0.07] | 0.030 / 0.030 [0.09 / 0.07] |
| $e_{\theta_L}$ [rad] | 0.000 [0.08] | 0.045 [0.15] | 0.046 [0.15] |
| $e_d$ / $e_{\theta_F}$ / $e_\alpha$ | – / – / 0.003 rad | 0.001 m / 0.001 / 0.000 rad | 0.000 m / 0.002 / 0.001 rad |
| object to circle, mean / max | 1 / 14 mm | 1 / 12 mm | 0 / 1 mm |
| object to midpoint (cage), max | 4.7 mm | 4.6 mm | 3.6 mm |
| leader speed: mean, std | 0.209, 0.007 m/s | 0.216, 0.005 m/s | 0.154, 0.006 m/s (inner circle 0.74 m) |
| time stopped | 0 % | 0 % | 0 % |

**Paper's waypoints** (`reference: waypoints`, 1 s steps):

| Error | static (exp. 1) | tangent (exp. 2) | radial |
|---|---|---|---|
| $e_{d_O}$ [m] | **0.103** [0.10] | **0.105** [0.13] | 0.139 [0.13] |
| $e_{x_O}$ / $e_{y_O}$ [m] | 0.065 / 0.066 [0.07 / 0.06] | 0.067 / 0.067 [0.09 / 0.07] | 0.089 / 0.088 [0.09 / 0.07] |
| $e_{\theta_L}$ [rad] | 0.000 [0.08] | 0.069 [0.15] | 0.072 [0.15] |
| $e_d$ [m] / $e_{\theta_F}$ / $e_\alpha$ [rad] | 0.003 / 0.001 / 0.007 | 0.003 / 0.010 / 0.003 | 0.007 / 0.010 / **0.192** |
| object to midpoint (cage), max | 15 mm | 10 mm | 20 mm |
| leader speed: mean, min–max, std (static) | 0.217, 0.003–0.300, 0.116 m/s; stopped 11 % of the time | | |

- With waypoints, the object position errors match Table 2: they come mostly from the 1 s steps.
  The smooth reference halves them and the motion has no stops.
- Orientation and formation errors are much smaller than in the paper: ideal simulation (no
  friction changes, no WiFi delays, perfect "Vicon").
- **Radial with waypoints**: the leader moves in bursts at 0.3 m/s; the follower on the outer
  circle then needs about $0.3 \cdot 1.26 / 0.74 = 0.51$ m/s, more than its 0.4 m/s limit, saturates
  and lags sideways ($e_\alpha$ = 0.19 rad; with a 0.6 m/s limit 0.012 rad). With the smooth
  reference the leader moves at a constant 0.154 m/s and the follower never saturates
  ($e_\alpha$ = 0.001 rad), so the 0.4 m/s limit is kept.
- The box stays caged in every run.

## Exercises

1. Compute the leader goals (Eq. 26) for the waypoints $i = 0, 7, 15$ of the static and the
   tangent circle ($d = 0.527$ m).
2. Why is $e_{d_O}$ about 0.10 m while the object is only 3 mm from the circle? Estimate the lag
   from the speed (0.21 m/s) and the waypoint interval (1 s).
3. Add a feed-forward to the LQR: send also the goal velocity (the derivative of $q_O^*(t)$,
   through Eq. 26) and add it to $v/T_s$ in the LQR node. Check that the 3.5 cm lag of the smooth
   reference disappears.
4. Run `radial` with `follower_max_speed` 0.4 and 0.6. Explain the difference in $e_\alpha$ with the
   radii of the leader and follower circles.
5. Make the lap in 15 s. Which limits are reached first? What happens to the cage?
6. In which experiment does the follower move backwards? Draw the formation at $\varphi = 0$ and
   $\varphi = \pi/2$ for `static`, `tangent` and `radial`.
