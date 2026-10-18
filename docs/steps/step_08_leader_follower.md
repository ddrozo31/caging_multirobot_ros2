# Step 8 — Leader-Follower Controller (Keeping the Object Caged)

## Objective

After the closure (Step 7) the object is caged between the two grippers. Now the **leader**
moves (driven by its LQR) and the **follower** must move with it so that the cage never opens:
same distance, facing the leader, opposite heading. This is the Leader-Follower controller of
the paper. When it works, moving the leader moves the whole formation and the box with it.

## Paper reference

- §2.4, Fig. 8 — Leader-Follower scheme: formation state $\eta = [d, \alpha, \theta_F]$.
- Eq. 14–21 — kinematics of each robot and of the formation; Eq. 22–23 — control law;
  Eq. 24–25 — stability.
- §2.5 — "once the object closure condition was achieved, the follower robot disabled the LQR
  controller and activated the Leader-Follower controller … the LQR controller of the leader
  remained active".
- Table 1 — $k_d = 2$, $k_\alpha = 1$, $k_{\theta_F} = 1$, $d^* = 0.52$ m, $\alpha^* = 0$.

## Theory

### 1. Three numbers describe the formation

```
            leader  (x_L, y_L, theta_L)
              ^
              |  d = distance between the centers
              |
   follower ──┘  alpha = theta_F - (direction of the line F -> L)
```

$$d = \sqrt{(x_L - x_F)^2 + (y_L - y_F)^2} \tag{15}$$
$$\alpha = \theta_F - \operatorname{atan2}(y_L - y_F,\ x_L - x_F) \tag{16}$$

and the follower heading $\theta_F$. The cage is kept when

| | Reference | Meaning |
|---|---|---|
| $d^*$ | distance measured at the closure (0.526 m) | the grippers keep the same grip |
| $\alpha^*$ | 0 | the follower looks straight at the leader |
| $\theta_F^*$ | $\theta_L - \pi$ | opposite headings |

With the corner grasp of Step 7 the follower still looks at the leader along the grasp axis,
so these references do not change.

**Why $d^*$ = measured?** Table 1 gives 0.52 m, but with the corner grasp the robots stop at
0.526 m (the 1 cm thick fingers touch the corner just before the apex). With $d^* = 0.52$ m the
follower would push 6 mm further into the box; with the measured value the grip stays exactly as
it was closed. Both values are in `config/leader_follower.yaml` (`use_measured_distance`).

### 2. How the formation moves (Eq. 14, 21)

Each robot moves with a body twist $u_r = [\dot x_R, \dot y_R, \dot\theta]^T$ in **its own
frame** (Eq. 14: $\dot q_r = R(\theta_r)\,u_r$) — the same twist our mecanum controller
receives. The formation changes as

$$\dot\eta = A(d,\alpha,\theta_F,\theta_L)\,u_L - B(d,\alpha)\,u_F \tag{21}$$

$$
A=\begin{bmatrix}\cos\gamma & -\sin\gamma & 0\\ -\frac1d\sin\gamma & -\frac1d\cos\gamma & 0\\ 0&0&0\end{bmatrix},\quad
B=\begin{bmatrix}\cos\alpha & -\sin\alpha & 0\\ -\frac1d\sin\alpha & -\frac1d\cos\alpha & -1\\ 0&0&-1\end{bmatrix},\quad
\gamma=\theta_L-\theta_F+\alpha
$$

- Row 1: how fast $d$ changes (both robots moving along the line between them).
- Row 2: how fast $\alpha$ changes (moving across the line, divided by $d$, and turning).
- Row 3: $\dot\theta_F = $ the follower's own turn rate (minus sign because of $-B$).

**We checked A and B numerically** (unit test `test_matrices_match_numerical_derivative`):
moving both robots a tiny step with Eq. 14 and computing $\eta$ before and after gives the same
$\dot\eta$ as Eq. 21, for 200 random poses and twists. Eq. 18 in the paper has a sign misprint
(it should be $\dot\alpha = \dot\theta_F - \frac{d_x\dot d_y - d_y\dot d_x}{d^2}$, the derivative of
atan2); $A$ and $B$ are correct.

### 3. The control law (Eq. 22–25)

We want each error $e = \eta - \eta^*$ to decay on its own: $\dot e = -Ke$. So we ask for

$$p_d = \begin{bmatrix}\dot d^* - k_d(d - d^*)\\ \dot\alpha^* - k_\alpha(\alpha-\alpha^*)\\ \dot\theta_F^* - k_{\theta_F}(\theta_F - \theta_F^*)\end{bmatrix}
= \begin{bmatrix}-k_d\,e_d\\ -k_\alpha\,e_\alpha\\ \dot\theta_L - k_{\theta_F}\,e_{\theta_F}\end{bmatrix} \tag{23}$$

($\dot d^* = \dot\alpha^* = 0$, $\dot\theta_F^* = \dot\theta_L$) and solve Eq. 21 for $u_F$ with
$\dot\eta = p_d$:

$$p_d = A u_L - B u_F \;\Rightarrow\; u_F = B^{-1}(A u_L - p_d) \tag{22}$$

Putting it back: $\dot e = -Ke$, $K = \text{diag}(2, 1, 1)$ (Eq. 24–25). Each error decays as
$e(t) = e(0)\,e^{-kt}$: $e_d$ with time constant $1/k_d = 0.5$ s, $e_\alpha$ and $e_{\theta_F}$ with 1 s.

- $A u_L$ is a **feed-forward** term: the follower already knows how the leader moves and
  copies it. Without it the follower would only react after an error appears.
- $\det B = 1/d$: $B$ can always be inverted for $d > 0$ (and $d \approx 0.52$ m here).
- The angle errors are wrapped to $[-\pi, \pi]$.
- $u_L$ is the leader's **commanded** body twist (`/leader/mecanum_controller/cmd_vel`, after
  twist_mux), so a joystick override of the leader is also followed.
- $u_F$ is saturated (0.3 m/s, 1.0 rad/s, vector scaling) and published on
  `/follower/cmd_vel_nav`. While the leader carries the box it is slowed to **0.1 m/s and
  0.3 rad/s**, so the follower never reaches its limits (a saturated $u_F$ breaks Eq. 25).

### 4. Switching controllers

```
/object_closed = true ─► approach_manager:
     1. follower LQR      active = false      (stops publishing)
     2. leader_follower   active = true       (d* = d measured now)
     3. leader LQR        0.1 m/s, 0.3 rad/s  (transport speed)
```

The LQR node and the Leader-Follower node both publish on `/follower/cmd_vel_nav`, so only one
of them may be active (new parameter `active`). The switch is done with the parameter service,
like `ros2 param set /follower/lqr_controller active false`.

**Safety:** if the leader command is older than 0.5 s, $u_L = 0$; if a ground-truth pose is older
than 0.5 s, the node stops publishing and the mecanum controller's timeout (Step 5) stops the
follower.

## Files created / changed

```
src/
├── caging_control/
│   ├── caging_control/core/leader_follower.py   # NEW: formation_state (Eq. 15-16),
│   │                                            #      formation_matrices (Eq. 21), follower_command (Eq. 22-23)
│   ├── caging_control/nodes/leader_follower.py  # NEW: the follower controller node
│   ├── caging_control/nodes/lqr_controller.py   # + parameter "active"
│   ├── caging_control/nodes/approach_manager.py # + start_transport (switch after the closure)
│   ├── test/test_leader_follower.py             # NEW: 5 tests (A, B vs numerical derivative,
│   │                                            #      det B, closed loop = Eq. 25, rest)
│   └── setup.py                                 # + leader_follower executable
└── caging_bringup/
    ├── config/leader_follower.yaml              # NEW: gains (Table 1), d*, limits, timeouts
    ├── config/transport.yaml                    # NEW: start_leader_follower, transport speed
    └── launch/caging.launch.py                  # + argument leader_follower
```

| Topic | Type | Meaning |
|---|---|---|
| `/follower/cmd_vel_nav` | `geometry_msgs/msg/Twist` | $u_F$ |
| `/formation_error` | `geometry_msgs/msg/Vector3Stamped` | x = $e_d$ [m], y = $e_\alpha$, z = $e_{\theta_F}$ [rad] |

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

ros2 launch caging_bringup caging.launch.py leader_follower:=true
# other terminal: approach + closure + switch to the formation
ros2 service call /start_approach std_srvs/srv/Trigger
# wait for "TRANSPORT mode" in the log, then move the LEADER (RViz: 1st "2D Goal Pose" button)
ros2 topic pub --once /leader/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: -0.168, y: -0.168}, orientation: {z: -0.92388, w: 0.38268}}}"

rqt_plot /formation_error/vector/x /formation_error/vector/y /formation_error/vector/z
```

Step response by hand: `ros2 param set /leader_follower distance_ref 0.556` (3 cm more), watch
$e_d$ decay, then set it back to the measured value printed in the log.

## Expected result

Headless test, box at $q_O = (0, 0, \pi)$ taken by the corners, leader moves at 0.1 m/s / 0.3 rad/s:

| Phase | max $\|e_d\|$ | max $\|e_\alpha\|$ | max $\|e_{\theta_F}\|$ | box – midpoint | box displacement |
|---|---|---|---|---|---|
| At rest after the switch | 0.1 mm | 0.01° | 0.01° | 0.0 mm | – |
| Leader 0.5 m forward (into the box) | 3.7 mm | 0.01° | 0.01° | 1.9 mm | 0.50 m |
| Leader 0.5 m sideways (+y) | 3.6 mm | 0.39° | 0.08° | 2.8 mm | 0.50 m |
| Leader turns 90° in place | 0.1 mm | 0.07° | 0.83° | 1.1 mm | 0.372 m (chord of its 0.263 m circle around the leader) |
| Step: $d^*$ + 3 cm | $e_d$ falls to 1/e in **0.55 s** (Eq. 25: 0.50 s) | | | 14.8 mm (grip opened) | – |

- $d^*$ measured at the switch: 0.526 m. The distance stayed between 0.525 and 0.530 m while
  moving.
- The 0.05 s difference in the step response is the 20 Hz sampling and the message delays.
- RViz: in the first version the box marker **jumped** while carried (smooth in Gazebo). The
  marker is published once per second and RViz placed it only when it arrived; now it has
  `frame_locked = true` and follows the TF frame `object` in every frame.

## Exercises

1. Compute $\eta$ for $q_L = (0.3, 0.1, 3.0)$ and $q_F = (-0.2, -0.05, -0.1)$. Is it a good
   formation? Which error is largest?
2. Derive row 1 of Eq. 21: differentiate $d = \sqrt{d_x^2 + d_y^2}$ and use Eq. 14 for both
   robots.
3. Set $A u_L$ to zero in `follower_command` (no feed-forward). Repeat the sideways move and
   compare max $|e_d|$ and $|e_\alpha|$. Explain.
4. Change $k_d$ to 0.5 and 8. Measure the step response time constant. What limits large
   gains on the real robot? (Hint: 20 Hz, saturation, WiFi delay.)
5. Use `use_measured_distance: false` ($d^* = 0.52$ m). Plot the finger angles
   (`/follower/joint_states`) at rest. Why do they bend more?
6. Move the leader at 0.3 m/s (`transport_speed: 0.3`) and turn it 90°. Does $u_F$ saturate?
   What happens to $e_\alpha$ and to the box?
7. Take over the leader with the keyboard during the transport. Does the follower follow?
   Why? (Hint: where $u_L$ comes from.)
