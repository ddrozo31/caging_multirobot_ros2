# Step 2 — Mecanum Kinematics: a Custom ros2_control Controller

## Objective

Make the robot move. We write our own **ros2_control controller in C++** that implements the
mecanum kinematics of the paper (Eq. 1) in both directions, and connect it to Gazebo.

> New to C++ or ros2_control? Read the companion
> [step_02_controller_walkthrough.md](step_02_controller_walkthrough.md): a line-by-line tour
> of the controller code, the C++ concepts it uses, and why ros2_control is organized this way.

## Paper reference

- §2.1.3 — Eq. 1 (forward kinematics), Eq. 2 ($\dot q_R = J_R\mu$), Fig. 6 (wheel numbering).
- §2.1.3 — Eq. 3–4: the twist we compute is in the **robot frame** {R}; Gazebo rotates it to
  the world frame {G} for us (exactly $R(\theta)$ of Eq. 3).
- [paper_reference.md](../paper_reference.md) §2 and §8.

## Theory

### Eq. 1 in both directions

**Forward** (wheel speeds → body twist), Eq. 1 of the paper:

$$
\begin{aligned}
\dot x_R &= \tfrac{r_w}{4}\,(\ \ \dot\theta_0 + \dot\theta_1 + \dot\theta_2 + \dot\theta_3)\\
\dot y_R &= \tfrac{r_w}{4}\,(-\dot\theta_0 + \dot\theta_1 + \dot\theta_2 - \dot\theta_3)\\
\dot\theta &= \tfrac{r_w}{4(l_x+l_y)}\,(-\dot\theta_0 + \dot\theta_1 - \dot\theta_2 + \dot\theta_3)
\end{aligned}
$$

**Inverse** (body twist → wheel speeds), with $L = l_x + l_y$:

$$
\begin{aligned}
\dot\theta_0 &= (\dot x_R - \dot y_R - L\dot\theta)/r_w \quad\text{front-left}\\
\dot\theta_1 &= (\dot x_R + \dot y_R + L\dot\theta)/r_w \quad\text{front-right}\\
\dot\theta_2 &= (\dot x_R + \dot y_R - L\dot\theta)/r_w \quad\text{rear-left}\\
\dot\theta_3 &= (\dot x_R - \dot y_R + L\dot\theta)/r_w \quad\text{rear-right}
\end{aligned}
$$

$J_R$ is 3×4, so it has no ordinary inverse; the matrix above is a **right inverse**
($J_R\,J_R^{+} = I_3$). Exercise 1 asks you to check it.

### How the robot moves in the simulation

Our wheels are plain cylinders with zero friction (Step 1), so they cannot push the robot.
The chain is:

```
 /mecanum_controller/cmd_vel  (desired twist, robot frame)
              │
              ▼
 ┌──────────────────────── MecanumController (C++, 100 Hz) ────────────────────────┐
 │ 1. inverse Eq. 1 ──► 4 wheel velocity commands ──► ros2_control ──► wheels spin │
 │ 2. read the 4 MEASURED wheel velocities                                         │
 │ 3. forward Eq. 1 ──► body twist                                                 │
 └──────────────────────────────────┬──────────────────────────────────────────────┘
                                    ▼
 /mecanum_controller/body_twist ──► bridge ──► /model/rosmaster_x3/cmd_vel
                                                     │
                                                     ▼
                                   Gazebo VelocityControl moves the chassis
```

If the kinematics are wrong (a sign, a wheel order, $r_w$), the robot visibly moves wrong.

### ros2_control in 1 minute

| Piece | What it is | In this step |
|-------|-----------|--------------|
| Hardware interface | Moves the real/simulated joints | `gz_ros2_control/GazeboSimSystem` |
| `<ros2_control>` tag | Lists joints and their **command** / **state** interfaces | `urdf/ros2_control.xacro` |
| Controller | Plugin that reads states and writes commands in `update()` | `MecanumController`, `joint_state_broadcaster` |
| controller_manager | Loads controllers and calls `update()` at `update_rate` | started by gz_ros2_control inside Gazebo |
| spawner | Command-line tool that loads + activates a controller | in `sim.launch.py` |

### Lifecycle of a controller

`on_init` (declare parameters) → `on_configure` (read parameters, create topics) →
`on_activate` (start at rest) → `update()` every 10 ms → `on_deactivate` (stop the wheels).

### Two threads, one command

The `~/cmd_vel` callback and `update()` run in different threads. They share the last command
through a `RealtimeThreadSafeBox`, and `update()` publishes with a `RealtimePublisher`, so the
control loop never waits for the network.

## Files created / changed

```
src/
├── caging_mecanum_controller/          (NEW, ament_cmake, C++)
│   ├── CMakeLists.txt                  # builds a SHARED library (plugin)
│   ├── package.xml
│   ├── mecanum_controller_plugin.xml   # tells pluginlib about our class
│   ├── include/caging_mecanum_controller/mecanum_controller.hpp
│   └── src/mecanum_controller.cpp      # Eq. 1 inverse + forward
├── caging_description/
│   ├── urdf/ros2_control.xacro         # NEW: wheel interfaces (velocity cmd, pos/vel state)
│   ├── urdf/gazebo.xacro               # + gz_ros2_control plugin + VelocityControl system
│   ├── urdf/robot.urdf.xacro           # includes ros2_control.xacro
│   └── config/ros2_controllers.yaml    # NEW: controllers + r_w, l_x, l_y (in the URDF since Step 12)
└── caging_gazebo/
    ├── config/topic_bridge.yaml        # + body_twist (ROS) -> /model/rosmaster_x3/cmd_vel (Gazebo)
    └── launch/sim.launch.py            # + spawners for the two controllers
```

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/mecanum_controller/cmd_vel` | `geometry_msgs/msg/Twist` | you → controller |
| `/mecanum_controller/body_twist` | `geometry_msgs/msg/Twist` | controller → bridge → Gazebo |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster → robot_state_publisher |

## How to build and run

```bash
cd ~/caging_multirobot_ros2
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch caging_gazebo sim.launch.py
```

In a second terminal:

```bash
ros2 control list_controllers          # both controllers must be "active"

# forward 0.2 m/s
ros2 topic pub -r 10 /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# sideways (left) 0.2 m/s  -> only possible with an omnidirectional robot
ros2 topic pub -r 10 /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}"
# rotate 0.5 rad/s
ros2 topic pub -r 10 /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# stop
ros2 topic pub --once /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "{}"
```

## Teleoperation: joystick, keyboard and twist_mux

Typing `ros2 topic pub` is fine for tests, but driving by hand is easier with a joystick or the
keyboard. Package **`caging_bringup`** adds a teleoperation chain:

```
joystick -> joy_node -> teleop_node -> /cmd_vel_joy ─┐ priority 100
keyboard -> teleop_twist_keyboard   -> /cmd_vel_key ─┼─> twist_mux ─> /mecanum_controller/cmd_vel
LQR / state machine (later steps)   -> /cmd_vel_nav ─┘ priority 10
```

- **`teleop_twist_joy`** maps sticks to a Twist. For a mecanum robot we also map
  **`linear.y`** (left stick, left/right): this is what a differential robot cannot do.
- **Enable button** (L1/LB): the robot only moves while you hold it — a *dead man's switch*.
  Turbo button (R1/RB) doubles the speed.
- **`twist_mux`** forwards the command of the source with the **highest priority**; a source
  silent for more than 0.5 s is ignored. The joystick (human) always overrides the automatic
  controllers we will write later, which will publish on `/cmd_vel_nav`.
- All messages are `geometry_msgs/msg/Twist` (`publish_stamped_twist: false`,
  `use_stamped: false`), because our controller subscribes to `Twist`.

Files:

```
src/caging_bringup/                (NEW, ament_python)
├── config/teleop.yaml             # joy_node, teleop_node and twist_mux parameters
└── launch/teleop.launch.py        # joy_node + teleop_node + twist_mux
```

Run it (with the simulation already running):

```bash
ros2 launch caging_bringup teleop.launch.py
ros2 topic echo /joy        # move the sticks: find YOUR axis/button numbers

# keyboard, in its own terminal (hold Shift for sideways motion)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key
```

Measured in a headless test: keyboard alone, $\dot x_R = 0.2$ m/s for 5 s → $\Delta x = 1.000$;
keyboard ($\dot x_R$) and joystick ($\dot y_R$) at the same time → $\Delta y = 1.000$,
$\Delta x = 0.000$ (the joystick wins).

## Expected result

Measured in a headless test (commands applied for an exact simulated time):

| Command | Duration | Expected | Measured pose change |
|---------|----------|----------|----------------------|
| $\dot x_R = 0.2$ m/s | 5 s | $\Delta x = 1.0$ m | $\Delta x = 1.000$, $\Delta y = 0.000$ |
| $\dot y_R = 0.2$ m/s | 5 s | $\Delta y = 1.0$ m | $\Delta y = 1.000$, $\Delta x = 0.000$ |
| $\dot\theta = 0.5$ rad/s | 3 s | $\Delta\theta = 1.5$ rad | $\Delta\theta = 1.500$ |
| $\dot x_R = 0.2$ m/s at $\theta = 1.5$ | 5 s | $(\cos 1.5, \sin 1.5) = (0.071, 0.997)$ m | $(0.0707, 0.9975)$ |

The last row shows Eq. 3–4: a forward command in {R} becomes a motion along the robot heading
in {G}.

**Note:** the controller keeps applying the **last** command it received. If you stop
publishing, the robot keeps moving: always send a zero twist to stop (see exercise 5).

## Exercises

1. Multiply the 3×4 matrix of Eq. 1 by the 4×3 inverse matrix used in `update()`.
   Do you get $I_3$? What do you get with the product in the other order (4×4), and why?
2. Swap two names in `wheel_joints` of `ros2_controllers.yaml` (e.g. wheel 0 and 1), rebuild,
   and command $\dot x_R = 0.2$. Describe and explain the motion.
   *Since Step 12 the controller checks the wheel order against the URDF and refuses to
   configure; to see the wrong motion, check out the tag `step-11` (or older).*
3. Change `wheel_radius` in the YAML to 0.065 (diameter instead of radius). Predict the
   distance travelled in 5 s at 0.2 m/s, then measure it.
   *Since Step 12 the radius is not in the YAML any more; reproduce the same mistake with
   `odom_radius_scale: 2.0` (Step 3) and compare the odometry with the ground truth.*
4. Why is the controller a **shared library** loaded by pluginlib, and not a node with `main()`?
5. Add a safety **timeout**: if no command arrives for 0.5 s, `update()` must use a zero
   twist. (Hint: store the time of the last message together with the twist.)
   *The project adds this timeout in Step 5 (`cmd_timeout`); try your own solution first, then
   compare it with [step_05_two_robots.md](https://github.com/ddrozo31/caging_multirobot_ros2/blob/v1.0/docs/steps/step_05_two_robots.md), theory section 9.*
6. `wheel_radius`, `lx` and `ly` are written twice (xacro and YAML). What could go wrong?
   Propose a way to keep a single source.
   *Solved in Step 12: the controller reads them from the URDF. Propose your own solution first,
   then compare it with [step_12_single_source.md](https://github.com/ddrozo31/caging_multirobot_ros2/blob/v1.0/docs/steps/step_12_single_source.md).*
7. Drive with the keyboard, then grab the joystick and press the enable button. What happens,
   and which `twist_mux` parameters explain it? What happens 0.5 s after you release it?
8. Your joystick is different: use `ros2 topic echo /joy` to find its axes and buttons and
   update `teleop.yaml`.
