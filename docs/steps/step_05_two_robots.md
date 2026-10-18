# Step 5 — Two Robots and the Objects

## Objective

Put the whole scene of the paper in the simulator: the **leader** and the **follower** robots
and one of the three **objects**, at the initial poses of Table 1. Each robot has its own
topics, TF frames, teleoperation and LQR controller (Step 4), thanks to **ROS 2 namespaces**.
The objects are light and slippery, so the robots can push them (the caging of Step 7 needs it).

## Paper reference

- §2.5 and Fig. 9 — experiment layout: object at the origin, two robots 2.5 m away.
- Table 1 — initial poses $q_O = [0, 0, \pi]^T$, $q_L = [0.5, -2.5, \pi/2]^T$,
  $q_F = [-0.5, -2.5, \pi/2]^T$.
- Fig. 10 — objects: box 0.2 × 0.2 × 0.2 m, hemisphere radius 0.1 m, three-sided pyramid
  (base edge 0.15 m, height 0.15 m, rounded edges).

## Theory

### 1. The problem: two robots, one set of names

Until Step 4 there was one robot, and its names were unique: `/mecanum_controller/cmd_vel`,
`/ground_truth/odom`, frame `base_link`. Start a second copy and both robots would read the same
command topic and publish the same frames: they would move together and TF would be a mess.

### 2. The solution: a namespace per robot

A **namespace** is a prefix that ROS 2 adds to every name of a node. We give each robot its own:

| | One robot (Steps 1–4) | Leader (Step 5) | Follower (Step 5) |
|---|---|---|---|
| Nodes | `/controller_manager` | `/leader/controller_manager` | `/follower/controller_manager` |
| Command | `/mecanum_controller/cmd_vel` | `/leader/mecanum_controller/cmd_vel` | `/follower/mecanum_controller/cmd_vel` |
| Ground truth | `/ground_truth/odom` | `/leader/ground_truth/odom` | `/follower/ground_truth/odom` |
| LQR goal | `/goal_pose` | `/leader/goal_pose` | `/follower/goal_pose` |
| TF frames | `odom`, `base_link` | `leader/odom`, `leader/base_link` | `follower/odom`, `follower/base_link` |
| Gazebo model | `rosmaster_x3` | `leader` | `follower` |

The **same code** runs for every robot. Only the namespace changes.

### 3. Relative and absolute names

- **Absolute** name (starts with `/`): used as is. `/goal_pose` is always `/goal_pose`.
- **Relative** name (no `/`): the namespace is added. `goal_pose` in namespace `/leader`
  becomes `/leader/goal_pose`.

So we changed the LQR node from `'/goal_pose'` to `'goal_pose'` (and the same for its other
topics), and twist_mux now sends to `mecanum_controller/cmd_vel`. With no namespace, the
relative names give exactly the Step 4 names, so the old launch files still work.

**Exception: `/tf` stays absolute.** TF is ONE shared tree for the whole system; every robot
publishes into it, with frame names that carry the robot name (`leader/base_link`).

### 4. Frames: the robots are placed by the "Vicon"

In the paper every robot pose comes from the **Vicon**; odometry is not used. From this step on
we do the same: the robot `base_link` is placed in TF by the **Gazebo ground truth**. The wheel
odometry is still computed, but only as an example (red/pink arrows in RViz): it shows what a
robot without Vicon would believe, and why the paper needed the Vicon (Step 3).

| Frames | Published by | How the prefix is added |
|---|---|---|
| `world → leader/base_link` | Gazebo `OdometryPublisher` (= Vicon) | xacro argument `namespace` (gazebo.xacro) |
| `leader/base_link → leader/chassis_link`, wheels | `robot_state_publisher` | parameter `frame_prefix: leader/` |
| `world → leader/odom` | `static_transform_publisher` | written in `spawn_robot.launch.py`; only the frame of the odometry messages |
| `world → object` | Gazebo `OdometryPublisher` in the object SDF | fixed name |

A TF frame has only **one parent**. `leader/base_link` now hangs from `world` (ground truth), so
the controller must not also publish `leader/odom → leader/base_link`: the spawner loads
`caging_gazebo/config/multi_robot_controllers.yaml` (`publish_tf: false`) on top of the normal
controller parameters. The controller still prefixes its frame names with the namespace
(new parameter `tf_frame_prefix_enable`, as `diff_drive_controller` does), so its odometry
messages say `leader/odom → leader/base_link`.

The one-robot launch files of Steps 1–4 keep the Step 3 setup (model placed by the odometry,
separate `ground_truth` frame), so the drift lesson of Step 3 still works.

The URDF itself has **no prefix** (`base_link`, `wheel_0_link`): it is the same file for both
robots. The same is true for **RViz**: its RobotModel display reads the URDF
(`base_link`) and must be told the prefix to find the TF frame (`leader/base_link`), with the
property **TF Prefix** = `leader` (set in `two_robots.rviz`). Without it RViz shows
"No transform from [base_link]" for every link.

```
world ─┬──(Gazebo ground truth = "Vicon")──► leader/base_link ──► chassis, wheels ...
       ├──(Gazebo ground truth = "Vicon")──► follower/base_link ──► chassis, wheels ...
       ├──(Gazebo ground truth)────────────► object
       ├──(static)──► leader/odom          (frame of /leader/mecanum_controller/odom, red arrows)
       └──(static)──► follower/odom
```

### 5. A trap: parameter files and namespaces

A YAML parameter file says which node gets each parameter:

```yaml
lqr_controller:          # matches ONLY the node /lqr_controller
  ros__parameters:
    sample_time: 0.05
```

A node in a namespace, `/leader/lqr_controller`, **silently ignores** this block and uses its
default values (we checked it: the node printed its default `Ts` instead of the YAML one).
The fix is the wildcard `/**`, which means "any namespace":

```yaml
/**/lqr_controller:      # matches /lqr_controller, /leader/lqr_controller, /follower/lqr_controller
  ros__parameters:
    sample_time: 0.05
```

All our YAML files (`ros2_controllers.yaml`, `teleop.yaml`, `lqr.yaml`) now use `/**/`.

### 6. gz_ros2_control in a namespace

Each robot's URDF has its own `gz_ros2_control` plugin. With the xacro argument
`namespace:=leader` the plugin gets

```xml
<ros>
  <namespace>/leader</namespace>
</ros>
```

so Gazebo runs **two** controller managers, `/leader/controller_manager` and
`/follower/controller_manager`. The spawners must be told which one to use:
`spawner mecanum_controller --controller-manager /leader/controller_manager`.

### 7. The objects

| | Box | Hemisphere | Pyramid |
|---|---|---|---|
| Size | 0.2 m cube | radius 0.1 m | base edge 0.15 m, height 0.15 m |
| Geometry | Gazebo `<box>` | mesh `hemisphere.stl` | mesh `pyramid.stl` |
| Mass | 0.1 kg | 0.1 kg | 0.1 kg |
| Center of mass (height) | 0.1 m ($a/2$) | 0.0375 m ($3r/8$) | 0.0375 m ($h/4$) |
| $I_{xx} = I_{yy}$ [kg m²] | $6.67\cdot10^{-4}$ ($\frac{m}{12}(a^2 + a^2)$) | $2.59\cdot10^{-4}$ ($\frac{83}{320} m r^2$) | $1.41\cdot10^{-4}$ (from the mesh) |
| $I_{zz}$ [kg m²] | $6.67\cdot10^{-4}$ | $4.00\cdot10^{-4}$ ($\frac{2}{5} m r^2$) | $1.13\cdot10^{-4}$ (from the mesh) |
| Friction $\mu$ | 0.2 | 0.2 | 0.2 |

- **Light and slippery** (as requested): 0.1 kg and $\mu = 0.2$. Gazebo uses the smaller $\mu$
  of the two surfaces in contact, so object–floor friction is 0.2. The friction force is only
  $\mu m g = 0.2 \cdot 0.1 \cdot 9.81 \approx 0.2$ N: easy to push.
- Gazebo has no hemisphere or pyramid shape, so `scripts/make_object_meshes.py` writes them as
  **STL meshes** (a list of triangles). The paper's pyramid has rounded edges; ours has sharp
  edges (simplification).
- Each object SDF has its own `OdometryPublisher`: the object pose is published on
  `/object/ground_truth/odom`, ready for the caging and transport steps.
- Gazebo finds the meshes through `model://caging_description/meshes/...`;
  `world.launch.py` adds the folder to `GZ_SIM_RESOURCE_PATH`.
- **RViz cannot read SDF models**, so the node `object_marker` publishes a `Marker` with the same
  shape (a cube, or the STL mesh through `package://caging_description/meshes/...`) attached to
  the TF frame `object`: RViz draws the object exactly where the ground truth says it is.
  The marker has `frame_locked = true`, so RViz moves it with the TF frame in every frame; with
  the default `false` RViz places it only when a message arrives (once per second), and a carried
  object jumps (seen in Step 8).

### 8. Colors: a red leader and a blue follower

The xacro argument `color` (default `white`) sets the chassis material, so the same URDF gives a
**red leader** and a **blue follower** (`two_robots.launch.py` passes `color:=red` / `color:=blue`
through `spawn_robot.launch.py` and `rsp.launch.py`). RViz reads the color from the URDF
`<material>`, and Gazebo gets it too, because the URDF → SDF conversion copies the material color
(`<diffuse>1 0 0 1</diffuse>` for red). The front marker is now **yellow** for every robot, so it
is still visible on the red one.

### 9. Safety: command timeout

Until now the controller kept its **last command forever**: if the LQR node crashed or the
network lost messages while the robot was moving, the robot would drive on (Step 2,
exercise 5). Now the controller stops the robot if no new command arrives for `cmd_timeout`
(0.5 s, in `ros2_controllers.yaml`):

```
callback (other thread):  numbered command -> box          (1, 2, 3, ...)
update() (every 10 ms):   new number?  -> use it, remember the time
                          now - time > 0.5 s  -> vx = vy = wz = 0   ("No command ... stopping")
```

- One late or lost message is harmless: the LQR publishes every 0.05 s, so 0.5 s means about
  **10 missing commands** in a row.
- The commands are **numbered** because the box always returns its last content: the number tells
  `update()` whether the command is new. The time is taken with the clock of `update()`
  (simulation time), never mixed with another clock.
- Every robot has its own controller, so **each car stops by itself** when its commands stop.
  `twist_mux` stops forwarding a source 0.5 s after its last message, so a crashed LQR stops its
  robot about 1 s later.


## Files created / changed

```
src/
├── caging_description/
│   ├── urdf/robot.urdf.xacro        # + xacro arguments "namespace" (empty = one robot) and "color"
│   ├── urdf/robot_core.xacro        # chassis color from "color", front marker yellow
│   ├── urdf/properties.xacro        # + material "yellow"
│   ├── urdf/gazebo.xacro            # + <ros><namespace> for gz_ros2_control,
│   │                                #   ground truth publishes world -> <namespace>/base_link
│   ├── config/ros2_controllers.yaml # "/**/" keys, tf_frame_prefix_enable
│   ├── launch/rsp.launch.py         # + "namespace" and "color" arguments, frame_prefix
│   ├── config/ros2_controllers.yaml # + cmd_timeout (0.5 s)
│   ├── objects/box.sdf, hemisphere.sdf, pyramid.sdf   # NEW: the objects of Fig. 10
│   ├── meshes/hemisphere.stl, pyramid.stl             # NEW: generated meshes
│   ├── scripts/make_object_meshes.py                  # NEW: writes the meshes
│   └── setup.py                     # installs objects/ and meshes/
├── caging_mecanum_controller/src/mecanum_controller.cpp  # + frame prefix from the namespace,
│                                                         #   command timeout
├── caging_gazebo/
│   ├── launch/world.launch.py       # NEW: Gazebo + bridge + RViz, no robots
│   ├── launch/spawn_robot.launch.py # NEW: one robot in its namespace
│   ├── config/topic_bridge.yaml     # + leader, follower and object entries
│   ├── config/multi_robot_controllers.yaml   # NEW: publish_tf false (robots placed by the Vicon)
│   └── rviz/two_robots.rviz         # NEW: both robots (TF Prefix), object, two goal tools
├── caging_control/caging_control/nodes/lqr_controller.py  # relative topic names
├── caging_control/caging_control/nodes/object_marker.py   # NEW: draws the object in RViz
└── caging_bringup/
    ├── launch/two_robots.launch.py  # NEW: the whole Step 5 scene
    ├── launch/teleop.launch.py      # + "namespace" and "joy_device" arguments
    └── config/teleop.yaml, lqr.yaml # "/**/" keys
```

The one-robot launch files of Steps 1–4 (`sim.launch.py`, `lqr.launch.py`, ...) still work
unchanged.

### Launch structure

```
two_robots.launch.py
├── world.launch.py              Gazebo, ONE bridge for everything, RViz
├── for leader and follower:
│   ├── spawn_robot.launch.py    rsp (+frame_prefix), create, spawners (+publish_tf false),
│   │                            static TF world -> <name>/odom
│   ├── teleop.launch.py         joy_node, teleop_node, twist_mux in /<name>
│   └── lqr_controller           in /<name>
└── create -name object -file objects/<object>.sdf   + object_marker (RViz)
```

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

ros2 launch caging_bringup two_robots.launch.py                    # box (default)
ros2 launch caging_bringup two_robots.launch.py object:=hemisphere
ros2 launch caging_bringup two_robots.launch.py object:=pyramid
```

**Control each robot**

- RViz: the **first** "2D Goal Pose" button of the toolbar sends a goal to the **leader**
  (yellow arrow), the **second** one to the **follower** (magenta arrow).
- Joystick: leader = `/dev/input/js0`, follower = `/dev/input/js1` (hold the enable button).
- Keyboard (one terminal per robot):
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/leader -r cmd_vel:=cmd_vel_key
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/follower -r cmd_vel:=cmd_vel_key
  ```
- Terminal:
  ```bash
  ros2 topic pub --once /follower/goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: -0.5, y: -1.0}, orientation: {z: 0.7071, w: 0.7071}}}"
  ```

**Inspect**

```bash
ros2 node list                       # everything twice: /leader/..., /follower/...
ros2 topic list | grep -E "leader|follower|object"
ros2 control list_controllers -c /follower/controller_manager
ros2 run tf2_tools view_frames       # frames.pdf with the TF tree of section 4
ros2 topic echo /object/ground_truth/odom --field pose.pose.position
```

### Troubleshooting

| Symptom | Cause and fix |
|---|---|
| RViz RobotModel: "No transform from [base_link]" | RobotModel **TF Prefix** empty: set it to the robot name |
| `view_frames` has no `leader/odom -> leader/base_link`, robots jump in RViz, "jump back in time" | **two simulations at once** (e.g. a Step 4 launch still running): both Gazebo servers share topics, including the clock. Close every other simulation, or give each one its own `GZ_PARTITION` and `ROS_DOMAIN_ID` |
| A node in `/leader` ignores its YAML parameters | YAML key without `/**/` (theory section 5) |

Quick TF check: `ros2 run tf2_ros tf2_echo world leader/base_link` must print a pose.

### Restriction: no collision avoidance yet

The LQR drives each robot in a **straight line** to its goal and knows nothing about the other
robot. **Do not send goals whose straight paths cross the other robot.** (Collision-free paths
come in Step 6 with A*, and the paper moves the robots one after the other.)

What happens if they collide (analysis of a recorded rosbag, 20261017): the follower drove
through the parked leader; the robots pushed each other and the leader tilted ≈ 18°.

| | After the collision |
|---|---|
| **Ground truth** (real robots, used by the LQR) | every later goal reached, error < 1 cm |
| **Wheel odometry** (what the robot believes) | wrong for ever: leader 0.29 m and 1.3 rad, follower 3.0 m and 2.3 rad |
| **RViz robot model** (was drawn at the odometry) | moved in the wrong direction with respect to the goal arrows |

During the push the wheels turned while the bodies were blocked or shoved: for the odometry this
is exactly **wheel slip**, which it cannot see (Step 3). The controller was right; the picture
was wrong. Since then the robot models are placed by the ground truth (section 4), as in the
paper, so RViz shows the real robots; only the red/pink odometry arrows stay wrong.

## Expected result

Measured in headless tests (default parameters):

| Check | Result |
|---|---|
| Controllers | 2 active per robot, in `/leader/controller_manager` and `/follower/controller_manager` |
| Controller frames (log) | `leader/odom -> leader/base_link`, `follower/odom -> follower/base_link` |
| TF tree (`view_frames`) | `world → leader/base_link`, `world → follower/base_link` (ground truth), `world → leader/odom`, `world → follower/odom` (static), `world → object` |
| `publish_tf` of each mecanum_controller | false (other parameters kept, e.g. `wheel_radius` 0.0325, read from the URDF since Step 12) |
| Colors | leader chassis red, follower blue (URDF material and Gazebo `<diffuse>`), front marker yellow |
| Object marker | `/object/marker` with the right shape (e.g. `package://caging_description/meshes/hemisphere.stl`) |
| Timeout: one command (0.3 m/s) sent once to the leader controller | robot moves 0.153 m (≈ 0.3 m/s × 0.5 s) and stops; log "No command for more than 0.50 s" |
| Timeout: leader LQR killed while driving to a goal | the leader stops; the follower is not affected |
| Object spawn | at (0, 0), yaw π, resting on the floor (z ≈ 0) for all three objects |
| Driving only the leader | the follower does not move |
| LQR goals at the same time | leader → (0.5, −1.0, π/2), follower → (−0.5, −1.5, 0): both reached, errors < 1 cm |
| Push test: leader → (0, −0.1, π/2) (its front is ≈ 0.12 m ahead of its center) | box pushed 0.119 m, hemisphere 0.114 m, pyramid 0.077 m (hit on an edge); objects stay upright |
| Step 4 `lqr.launch.py` (one robot, no namespace) | unchanged: `/goal_pose`, frames `odom -> base_link`, goal reached |

## Exercises

1. Run `ros2 node list` and `ros2 topic list` with two robots. Which topics are **not** in a
   namespace, and why must they stay global?
2. Remove the `/**/` from `lqr.yaml` (just `lqr_controller:`), rebuild and launch. Read the
   first log line of each LQR node: which `Ts` do they use? Explain.
3. Run `ros2 run tf2_tools view_frames` and draw the TF tree. Mark which node publishes each
   arrow (use section 4).
4. Add a third robot `helper` at (1.5, −2.5, π/2). What do you need to add in
   `two_robots.launch.py`, `topic_bridge.yaml` and the RViz file? Why does the URDF not change?
5. Push the box with one robot, then with both robots side by side. Compute the friction force
   $\mu m g$. What happens if you set the box mass to 5 kg? And $\mu = 1$?
6. Push the pyramid on a face and on an edge. Why does the edge push rotate it? (Hint: where is
   the contact force with respect to the center of mass?)
7. Send two goals whose paths cross and record `ros2 bag record -a`. Plot the ground truth and
   the odometry of both robots. Why does the odometry stay wrong after the collision, while the
   LQR still reaches its goals?
8. Command timeout: send ONE command with `ros2 topic pub --once .../mecanum_controller/cmd_vel`
   and measure how far the robot goes. Repeat with `cmd_timeout: 2.0`. Which value would you
   choose for a real robot over WiFi, and why not 0.01 s?
9. Open `meshes/pyramid.stl` in a text editor. How many triangles does it have? Check with the
   right-hand rule that the base normal points down.
