# Step 12 — Single Source of the Robot Dimensions

## Objective

Until Step 11 the three dimensions of Eq. 1 — the wheel radius $r_w$ and the half distances
$l_x$, $l_y$ — were written in **three** places:

| Where | Used for |
|---|---|
| `caging_description/urdf/robot_core.xacro` | the robot model: Gazebo, RViz, TF |
| `caging_description/config/ros2_controllers.yaml` | the parameters of the mecanum controller (Eq. 1) |
| `mecanum_controller.cpp` (`on_init`) | default values of those parameters |

If someone changes the wheel in the xacro and forgets the YAML, Gazebo simulates one robot and
the controller computes Eq. 1 for another: the robot drives the wrong distances and the odometry
drifts, with no error message. This was Step 2 exercise 6.

In this step the **robot description becomes the only source**: the controller reads $r_w$, $l_x$
and $l_y$ from the URDF.

## Paper reference

- Eq. 1 and Fig. 6 — the mecanum kinematics and the wheel numbering.
- [paper_reference.md](../paper_reference.md) §8 — the ROSMASTER X3 dimensions.

## Theory

### "Single source of truth"

A value that is written twice will, sooner or later, be different in the two places. The usual fix
is to keep it in **one** place and let everything else *read* it. For a robot, the natural place is
the **robot description** (URDF): it already contains every dimension, and every ROS 2 tool
(robot_state_publisher, RViz, Gazebo, MoveIt, Nav2, ros2_control) reads it.

```
robot_core.xacro ──xacro──► URDF ──► robot_state_publisher (/robot_description)
  wheel_radius, lx, ly                  ├──► Gazebo     (wheel size and position)
  (written ONCE)                        ├──► RViz / TF
                                        └──► controller_manager ──► MecanumController
                                                                    r_w, l_x, l_y for Eq. 1
```

### Where the controller finds each value

The controller_manager gives every controller the URDF it received; in the code,
`get_robot_description()` returns it as text, and the standard `urdf` library turns it into a tree
of links and joints (`urdf::Model`).

| Value | Read from | In `robot_core.xacro` |
|---|---|---|
| $l_x$, $l_y$ | the origin `xyz` of each wheel joint, a child of `base_link` | `<origin xyz="${x} ${y} ${wheel_radius}"/>` with $x = \pm l_x$, $y = \pm l_y$ |
| $r_w$ | the `<cylinder radius>` of the wheel's `<collision>` | `<cylinder radius="${wheel_radius}" .../>` |

The collision shape is used (not the visual) because it is what the physics engine uses: the
wheel that rolls on the floor.

### A free safety check: the wheel order

Fig. 6 fixes where each wheel must be:

| Wheel $i$ | Position in `base_link` |
|---|---|
| 0, front-left | $(+l_x, +l_y)$ |
| 1, front-right | $(+l_x, -l_y)$ |
| 2, rear-left | $(-l_x, +l_y)$ |
| 3, rear-right | $(-l_x, -l_y)$ |

The controller computes $l_x, l_y$ from wheel 0 (which must be front-left: $x > 0$, $y > 0$) and
checks that the other three give the same values with the signs of the table. So a wrong order in
`wheel_joints` (Step 2 exercise 2) is now detected **at configure time**, before the robot moves:

```
[mecanum_controller]: Wheel 0 ('wheel_1_joint') is at (0.0800, -0.0845): wheel 0 must be
front-left (x > 0, y > 0). Check the order of 'wheel_joints' (Fig. 6).
```

## Files changed

```
src/
├── caging_mecanum_controller/
│   ├── include/.../mecanum_controller.hpp   # + read_dimensions_from_urdf(); r_w, l_x, l_y no longer parameters
│   ├── src/mecanum_controller.cpp           # + read_dimensions_from_urdf(); on_init/on_configure without them
│   ├── CMakeLists.txt, package.xml          # + dependency "urdf"
└── caging_description/config/ros2_controllers.yaml   # wheel_radius, lx, ly removed (comment explains why)
docs/steps/
├── step_12_single_source.md                 # this guide
├── step_02_mecanum_controller.md            # "Since Step 12" notes in exercises 2, 3, 6
└── step_02_controller_walkthrough.md        # "Since Step 12" notes (6.1, Python pseudo-code, section 11)
```

### The new function, step by step

```cpp
urdf::Model model;
model.initString(get_robot_description());               // 1) URDF text -> tree

auto joint = model.getJoint(wheel_joints_[i]);            // 2) wheel joint i
const auto & position = joint->parent_to_joint_origin_transform.position;
const double lx_i = sign_x[i] * position.x;               //    Fig. 6 signs: + + - -
const double ly_i = sign_y[i] * position.y;               //                  + - + -

auto wheel_link = model.getLink(joint->child_link_name);  // 3) the wheel link
wheel_radius_ = std::static_pointer_cast<urdf::Cylinder>( //    its collision geometry
    wheel_link->collision->geometry)->radius;             //    is a cylinder: take the radius
```

`static_pointer_cast` is needed because the URDF library stores every shape as a general
`Geometry` (box, cylinder, sphere or mesh). After checking `geometry->type == CYLINDER`, the cast
says "treat it as a cylinder", so we can read `radius`. In Python this would just be
`geometry.radius`.

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build && source install/setup.bash
ros2 launch caging_gazebo sim.launch.py
# in the log:
#   [mecanum_controller]: From the URDF: r_w = 0.0325 m, l_x = 0.0800 m, l_y = 0.0845 m
ros2 param list /mecanum_controller        # no wheel_radius, lx, ly any more
```

## Expected result

Measured headless on 20261019:

| Check | Result |
|---|---|
| One robot (`sim.launch.py`): values read from the URDF | $r_w = 0.0325$, $l_x = 0.0800$, $l_y = 0.0845$ m |
| Two robots (`task.launch.py`): leader and follower | the same values, logged by both controllers |
| Drive test: 0.2 m/s for 5 s, twice; 0.2 m/s sideways; 0.5 rad/s for 3 s | $\Delta x$ = 1.000 then 2.000 m, $\Delta y$ = 1.000 m, $\Delta\theta$ = 1.500 rad |
| `wheel_joints` with wheels 0 and 1 swapped | controller refuses (stays `unconfigured`) with the message above |
| Whole task, box, static, smooth reference (no noise) | DONE, $e_{d_O}$ = 0.047 m, as in Steps 10–11 |

Note: if the first command is sent immediately after the controller becomes active, the first leg
can be a few centimeters short (the robot is still settling after the spawn). This start-up
effect is not related to the dimensions: a wrong $r_w$ or $l_x$ would shorten *every* leg by the
same factor.

## Exercises

1. Change `wheel_radius` in `robot_core.xacro` to 0.04 (and nothing else). Rebuild and run the
   drive test. Does the robot still travel 1.0 m in 5 s at 0.2 m/s? Why? What changed in Gazebo?
2. Do the same before this step (`git checkout step-11`, change only the xacro). Measure the
   distance and explain the difference.
3. Move one wheel 1 cm forward in the xacro (e.g. only wheel 2). What does the controller say?
   Why is it better to fail at configure time than to drive with a non-symmetric model?
4. The radius is read from the `<collision>` cylinder. What would go wrong if the visual and the
   collision cylinders had different radii? Which one rolls on the floor?
5. `wheel_width` is also in the xacro but not in Eq. 1. Which other parameters of the project are
   still written in two places? (Hint: Table 1 values in `caging_bringup/config/*.yaml`.)
