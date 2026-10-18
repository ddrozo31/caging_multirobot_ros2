# Step 7 — Soft Gripper and Object Closure (Caging)

## Objective

Mount a simplified **soft gripper** on each robot and perform the **object closure** stage of
the paper: after the approach (Step 6), both robots move towards the box at the same time until
it is trapped between their grippers. A node checks the closure condition and publishes
`/object_closed = true`, ready for the transport (Step 8).

In this step we work with the **box** only.

## Paper reference

- §2.1.2, Fig. 3–5 — soft gripper with Fin-Ray fingers (TPU), PLA frame, SG5010 servo; FEA:
  15.83 mm maximum bending under lateral load.
- §2.1 (stage 3) — *object closure*: the object stays confined in the region enclosed by the
  soft grippers, so it cannot escape by translation or rotation.
- §2.5 — "once the robots reached the approximation pose, they performed the object closure
  stage, moving to their respective goal positions".
- Table 1 — $l_g = 0.12$ m, $r_O = 0.15$ m, $d^* = 0.52$ m.

## Theory

### 1. Where do the robots stop? $d^* = 0.52$ m

After the approach the robot centers are $2d_a = 0.84$ m apart. Table 1 gives the distance
between robots during the transport, $d^* = 0.52$ m, so each robot advances

$$\frac{0.84 - 0.52}{2} = 0.16 \text{ m}$$

and stops with its center 0.26 m from the object center. Its gripper center
($l_g = 0.12$ m ahead) is then $0.26 - 0.12 = 0.14$ m $\approx r_O$ from the object center: the
gripper touches the object at its circumscribed circle. This is why we use **0.52 m from
Table 1** (the text of §2.4 says $d^* = r_O$, which cannot be a distance between robot centers;
this step resolves that ambiguity).

$$q_{L,\text{close}} = q_O + \frac{d^*}{2}\begin{bmatrix}-\cos\theta_O\\ -\sin\theta_O\end{bmatrix},\qquad
q_{F,\text{close}} = q_O + \frac{d^*}{2}\begin{bmatrix}\cos\theta_O\\ \sin\theta_O\end{bmatrix}$$

with the same headings as the approach poses (Eq. 13). Table 1 object: leader → (0.26, 0, π),
follower → (−0.26, 0, 0).

### 2. Faces or corners? The grasp offset

A V gripper holds a **corner** best: the corner sits in the apex and the two fingers stop it
from sliding sideways. With Eq. 13 as written, the robots approach along the object axis
$\theta_O$, so they meet the **faces** of the box: the face stays 4 cm in front of the apex and
only the box corners touch the fingers, pushing them open (first version of this step, see the
results table).

So we rotate the approach axis by a **grasp offset** (our extension of Eq. 13):

$$\theta_{\text{grasp}} = \theta_O + \text{grasp\_offset}$$

| Object | Corners (object frame) | grasp_offset | Result |
|---|---|---|---|
| Box | ±45°, ±135° | **π/4** | both robots on opposite corners |
| Pyramid | 0°, 120°, 240° | 0 | follower on a corner, leader on the opposite face (a triangle has no opposite corners) |
| Hemisphere | – | any | round: the direction does not matter |

The box corner is $0.1\sqrt2 = 0.141$ m from its center, and at $d^* = 0.52$ m the apex is
$0.26 - 0.12 = 0.14$ m from it: **the corner just reaches the apex**. This fits $r_O = 0.15$ m
(the circle through the corners) and is probably how Table 1 was chosen.

### 3. The gripper: a V of two fingers

```
                         finger_left  (60 deg from X)
                       /
      [ chassis ]=== o  <- apex (two hinges) at x = l_g = 0.12 m, the chassis front
                       \
                         finger_right (-60 deg from X)
```

| Parameter | Value | Why |
|---|---|---|
| Apex position | 0.12 m ahead of the robot center | $l_g$ of Table 1 |
| Opening | 120° (each finger 60° from X) | wide enough for the box corners |
| Finger length | 0.16 m | tips at (0.20, ±0.139) m |
| Finger section | 1 cm × 6 cm | same height band as the chassis (z = 0.03–0.09 m) |

**Can the object escape?** At $d^* = 0.52$ m the fingertips of the two robots leave a gap of
0.12 m on each side. The box (0.2 m) cannot pass. With 0.12 m fingers the gap would be 0.16 m.

**Where does the box touch?** Taken by a corner (section 2), the corner rests at the apex and
the fingers hardly bend. Taken by a face, the two front corners push the fingers open by
≈ 10°: this is where the "soft" part matters.

### 4. Making the fingers soft

The paper's TPU fingers bend and spring back. Two ways to imitate that in Gazebo:

| | Contact stiffness `kp`/`kd` on the surface | Finger on a hinge with spring + damper |
|---|---|---|
| Idea | the surfaces may overlap a little, pushing back like a spring | the rigid finger rotates when pushed, a spring brings it back |
| Gazebo Harmonic (DART) | **ignored** (tested: a 10 kg block on a surface with kp = 1000 did not sink at all) | **works** (`<spring_stiffness>`) |

So each finger is a rigid bar on a revolute joint with:

- spring stiffness $k = 3.2$ N·m/rad (a 2 N push at the tip bends it $F L^2 / k = 1.6$ cm, the
  order of the paper's 15.8 mm), rest angle 0 (the 120° V);
- damping 0.05 N·m·s/rad (no oscillation);
- limits ±0.35 rad (±20°).

URDF has no springs, so the spring is a Gazebo extension in `gripper.xacro`
(`<gazebo reference="finger_left_joint"><springStiffness>`). The finger joints are added to
`ros2_control` with **state interfaces only** (no command): `joint_state_broadcaster` then
publishes their angle, and RViz can draw the bending fingers.

The gripper only exists from Step 7 on: xacro argument `gripper:=true` (default false, so
Steps 1–6 are unchanged).

### 5. A bigger robot for A\*

With the gripper the fingertips are at (0.20, ±0.139) m from the robot center, so the
footprint circle grows from 0.155 m to **0.245 m**. With the Step 6 margin the inflated object
would be $0.15 + 0.245 + 0.05 = 0.445$ m, more than $d_a = 0.42$ m: the approach pose would be
inside the forbidden area and A\* would find no path. Step 7 uses an **object margin of
0.02 m**: $0.15 + 0.245 + 0.02 = 0.415 < 0.42$ m (`config/caging.yaml`).

### 6. Closing: both at once, slowly

- **At the same time**: the box is light (0.1 kg, $\mu = 0.2$). If one robot closed first, it
  could push the box away before the other arrives.
- **Slowly**: the LQR speed limit is lowered to 0.05 m/s during the closure and restored to
  0.3 m/s afterwards. The LQR node now accepts changes of `max_linear_speed` while running;
  `approach_manager` changes it through the parameter service
  (the same as `ros2 param set /leader/lqr_controller max_linear_speed 0.05`).

### 7. When is the object closed?

Checked every 0.1 s with the ground truth:

1. each robot within 1 cm and 0.02 rad of its closing goal;
2. distance between the robots $|d - d^*| < 1$ cm;
3. object center less than 3 cm from the midpoint of the two robots.

Condition 1 alone is not enough: two robots each 1 cm short leave $d$ 2 cm too long (this
happened in the first test). If the condition is not met within 30 s, a warning is printed.

```
/start_approach ─► approach (Step 6) ─► closing goals (d*/2 each side) ─► LQR at 0.05 m/s
                                         ─► condition 1-2-3 ─► /object_closed = true, LQR back to 0.3 m/s
```

## Files created / changed

```
src/
├── caging_description/
│   ├── urdf/gripper.xacro          # NEW: V gripper, fingers on spring hinges
│   ├── urdf/robot.urdf.xacro       # + xacro argument "gripper" (default false)
│   ├── urdf/ros2_control.xacro     # + finger joints, state interfaces only
│   └── launch/rsp.launch.py        # + "gripper" argument
├── caging_gazebo/launch/spawn_robot.launch.py   # + "gripper" argument
├── caging_control/caging_control/core/approach.py   # + grasp_offset in Eq. 13
├── caging_control/caging_control/core/astar.py      # + circles, point_is_free (goal cell fix)
├── caging_control/caging_control/nodes/
│   ├── approach_manager.py         # + object_margin, grasp_offset, closure stage, /object_closed
│   └── lqr_controller.py           # speed limits can change while running
└── caging_bringup/
    ├── config/approach.yaml        # object_margin separated from safety_margin
    ├── config/caging.yaml          # NEW: bigger footprint, grasp_offset, closure parameters
    ├── launch/caging.launch.py     # NEW: scene with grippers + approach + closure
    └── launch/two_robots.launch.py # + "gripper" argument
```

| Topic / service | Type | Meaning |
|---|---|---|
| `/start_approach` | `std_srvs/srv/Trigger` | approach + closure |
| `/object_closed` | `std_msgs/msg/Bool` (latched) | true when the box is caged |
| `/<robot>/joint_states` | `sensor_msgs/msg/JointState` | now also `finger_left_joint`, `finger_right_joint` |

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

ros2 launch caging_bringup caging.launch.py                       # Table 1 box
ros2 launch caging_bringup caging.launch.py object_yaw:=1.5708    # box rotated 90 deg

# other terminal, when everything is loaded
ros2 service call /start_approach std_srvs/srv/Trigger
ros2 topic echo /object_closed
ros2 topic echo /leader/joint_states --field position     # last two values: finger angles
```

**Shake test** (is it really caged?): after the closure, move both robots 0.25 m sideways
together, slowly:

```bash
for r in leader follower; do ros2 param set /$r/lqr_controller max_linear_speed 0.05; done
ros2 topic pub --once /leader/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.26, y: 0.25}, orientation: {z: 1.0, w: 0.0}}}"
ros2 topic pub --once /follower/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: -0.26, y: 0.25}, orientation: {w: 1.0}}}"
```

## Expected result

Measured in headless tests (box), **taken by the corners** (`grasp_offset: 0.7854`):

| Object yaw | Approach paths | `OBJECT CLOSED` | Final $d$ | Finger angles | Box moved |
|---|---|---|---|---|---|
| π (Table 1) | leader around the box (2 waypoints), follower straight | 3.3 s after the closing started | 0.526 m | ≤ 0.012 rad | 0.7 mm |
| −45° | 2 waypoints each | 3.2 s | 0.526 m | ≤ 0.008 rad | 0 |
| π/2 | leader straight, follower around (2 waypoints) | 3.2 s | 0.526 m | ≤ 0.007 rad | 0 |

- The robots stop 3 mm before their goals ($d$ = 0.526 m instead of 0.52 m): the corner
  touches the inner faces of the 1 cm thick fingers just before the apex. $|d - d^*| < 1$ cm, so
  the closure condition holds.
- **Shake test** (yaw π, both robots 0.25 m along +y at 0.05 m/s): the box follows to
  y = 0.250 m, exactly at the midpoint; fingers flex at most 0.034 rad while starting.

**First version, taken by the faces** (`grasp_offset: 0`), for comparison:

| | Faces | Corners |
|---|---|---|
| Contact | the 2 front corners of the box push the fingers open | the corner sits at the apex |
| Finger angles at rest | ±0.168 rad (≈ 10°, tip ≈ 2.7 cm) | ≤ 0.012 rad |
| Shake test: box behind the midpoint | 4 mm | 0 mm |

Object yaw −45° first gave "No path found" (goal cell inside the inflated circle; cause and fix
in the Step 6 guide). The unit test `test_every_object_orientation_has_a_path` now plans every
5° with and without gripper.

Step 6 `approach.launch.py` (no gripper, `grasp_offset: 0`) is unchanged.

## Exercises

1. Compute by hand the closing goals for $q_O = (0.5, 0.2, \pi/2)$, $d^* = 0.52$ m and
   `grasp_offset` = π/4.
2. Why does the inflated object radius have to be smaller than $d_a$? Run `caging.launch.py`
   with `object_margin: 0.05` and read the error of `approach_manager`.
3. Set `finger_stiffness` to 0.5 and to 30 in `gripper.xacro`. Measure the finger angle on the
   box and the box displacement during the closure. What is "too soft" and "too stiff"?
4. Shorten the fingers to 0.10 m. Compute the gap between the fingertips at $d^*$. Repeat the
   shake test: does the box stay?
5. Set `closing_speed: 0.3`. What happens to the box during the closure? Why does the paper
   close slowly?
6. Change the closure to "leader first, then follower" (two steps). Compare the box displacement
   with the simultaneous closure.
7. Why is condition 1 of theory section 7 not enough? Give the worst case of $d$ when each
   robot is 1 cm from its goal.
8. Run with `grasp_offset: 0` (faces) and π/4 (corners). Compare the finger angles and the shake
   test. Then compute the V opening that would make the box **faces** touch both fingers when
   taken by a corner. Why do we keep 120° anyway? (Hint: the pyramid corner is 60°.)
