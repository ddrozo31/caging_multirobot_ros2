# Step 6 — Object Approach: Approach Poses and A* Planning

## Objective

Bring both robots, **one after the other**, from their start poses to the **approach poses**
in front of the object (paper Fig. 7), without touching the object or the other robot. The
approach poses come from Eq. 13; the collision-free paths come from the **A\*** algorithm; the
robots are driven along the paths by the LQR controller of Step 4.

After this step both robots face the object from opposite sides, ready for the caging (Step 7).

## Paper reference

- §2.3 and Fig. 7 — object approach: robots parallel to the object, centers collinear with the
  object center; leader first, then follower; A* to avoid the object and the other robot.
- Eq. 13 — approach poses $q_L^*$, $q_F^*$; approach distance $d_a = 2r_O + l_g$.
- Table 1 — $r_O = 0.15$ m, $l_g = 0.12$ m, $q_O = [0, 0, \pi]^T$.

## Theory

### 1. Where must the robots stop? (Eq. 13)

$$d_a = 2r_O + l_g = 2(0.15) + 0.12 = 0.42 \text{ m}$$

$$q_L^* = q_O + d_a\begin{bmatrix}-\cos\theta_O\\ -\sin\theta_O\\ 0\end{bmatrix}, \qquad
q_F^* = q_O + d_a\begin{bmatrix}\cos\theta_O\\ \sin\theta_O\\ -\pi/d_a\end{bmatrix} \tag{13}$$

- The leader stops **behind** the object (along $-\theta_O$) and looks along $\theta_O$, at the
  object.
- The follower stops on the **other side** and looks along $\theta_O - \pi$ (the third row:
  $d_a \cdot (-\pi/d_a) = -\pi$), at the leader.
- The robot **center** is 0.42 m from the object center; the gripper center ($l_g = 0.12$ m
  ahead of the robot center) is $2r_O = 0.30$ m from it: room to close around the object in
  Step 7.

| Object pose $q_O$ | $q_L^*$ | $q_F^*$ |
|---|---|---|
| $(0, 0, \pi)$ (Table 1) | $(0.42, 0, \pi)$ | $(-0.42, 0, 0)$ |
| $(0, 0, \pi/2)$ (our second test) | $(0, -0.42, \pi/2)$ | $(0, 0.42, -\pi/2)$ |

Check the first row yourself: $\cos\pi = -1$, so $x_L^* = 0 + 0.42 \cdot (-(-1)) = 0.42$.

`r_O = 0.15` m is used for the **three** objects, as in Table 1 (the real sizes differ: box
half-diagonal 0.141 m, hemisphere 0.10 m, pyramid corner 0.087 m).

### 2. The map: an occupancy grid

A\* works on a **grid**: the arena (the paper's Vicon area, 3 m × 6 m, $x \in [-1.5, 1.5]$,
$y \in [-3, 3]$) cut into 5 cm cells, $60 \times 120 = 7200$ cells. Each cell is free or
occupied, stored as a numpy array `occupied[row, col]` (row = $y$, col = $x$, like an image).

```
world (x, y) [m]  ->  cell:  col = int((x - x_min) / resolution),  row = int((y - y_min) / resolution)
cell (row, col)   ->  its center:  x = x_min + (col + 0.5) resolution,  y = y_min + (row + 0.5) resolution
```

### 3. Inflation: plan for a point, not for a robot

Planning for a robot with a size is hard; planning for a **point** is easy. So we grow every
obstacle by the size of the robot and then plan for the robot **center** only (this is called
the *configuration space*):

| Obstacle | Real size | + robot radius | + margin | Inflated radius |
|---|---|---|---|---|
| Object | $r_O = 0.15$ | 0.155 | 0.05 | **0.355 m** |
| Other robot | 0.155 | 0.155 | 0.05 | **0.36 m** |

The robot radius $0.155 = \sqrt{0.119^2 + 0.10^2}$ m is the circle around the chassis
(half length, half width). With a **circle**, the robot can turn on the spot anywhere without
hitting anything, so the heading does not matter for the plan.

The approach pose is free: $d_a = 0.42 > 0.355$.

### 4. A\*: the shortest path on the grid

Each cell has 8 neighbors (straight steps cost 1, diagonal steps $\sqrt 2$). A\* keeps a list
of cells to explore and always takes the one with the smallest

$$f = g + h$$

- $g$: length of the best path found **from the start** to the cell;
- $h$: **straight-line distance** from the cell to the goal (the *heuristic*). It is never more
  than the true remaining length, so A\* always returns the **shortest** path.

```
open = {start}                         # cells to explore, ordered by f
while open is not empty:
    cell = cell of open with the smallest f
    if cell == goal: walk back with came_from -> path
    for each free neighbor n of cell:
        if g[cell] + step_cost < g[n]:   # found a better way to n
            g[n] = g[cell] + step_cost
            came_from[n] = cell
            put n in open with f = g[n] + h(n)
```

Without $h$ (h = 0) this is Dijkstra's algorithm: it also finds the shortest path, but
explores cells in every direction. $h$ pulls the search towards the goal.

Two details in `core/astar.py`:
- a diagonal step is not allowed if it would cut the corner of an occupied cell;
- the robot's own start cell is set free (if it was pushed into an inflated area, A\* can still
  leave it);
- the **goal cell** is set free when the exact approach pose is free. The approach pose is
  0.42 m from the object, only a few centimeters outside the inflated circle, but A\* plans to
  the **center of its cell**, which can be up to $0.05\sqrt2/2 = 3.5$ cm away and fall inside the
  circle. Without this, A\* failed for many object orientations (e.g. $\theta_O = -45°$: cell
  center 0.389 m from the object). `Grid.point_is_free` tests the exact point against the
  circles; if the exact pose is really blocked, `approach_manager` reports it.

### 5. From cells to waypoints

The A\* path is a chain of 5 cm cells (e.g. 61 cells). Sending each cell to the LQR would make
the robot stop and go 61 times. `simplify_path` keeps only the **corners**: from each kept
point it jumps to the farthest later point reachable in a straight free line. The 61 cells
become 1–3 waypoints, and the last one is replaced by the exact approach pose.

### 6. Following the waypoints with the LQR (Step 4)

`approach_manager` sends the waypoints one by one to `/<robot>/goal_pose`; the robot's LQR
drives to each one. Every waypoint carries the **final heading** $\theta^*$, so the robot turns
while it travels (limited to 1 rad/s) and arrives already looking at the object.

- **Intermediate** waypoint: when the robot is closer than 0.10 m, the next waypoint is sent.
  The robot never slows down at a corner (the LQR is still saturated at 0.3 m/s 0.10 m from
  the goal: $K e / T_s = 5.93 \cdot 0.10 = 0.59 > 0.3$), but it **cuts the corner** a little.
- **Last** waypoint: wait until the ground truth is within 1 cm and 0.02 rad of $q^*$.

### 7. One robot after the other

As in the paper: the **leader** plans and moves first, with the follower (standing at its start
pose) as an obstacle. Then the **follower** plans, with the leader (parked at $q_L^*$) as an
obstacle. Nobody moves while the other plans, so the obstacles are always where the map says.

```
/start_approach ─► object pose (ground truth) ─► Eq. 13 ─► q_L*, q_F*
   leader:   grid(object, follower) ─► A* ─► corners ─► /leader/goal_pose ... ─► at q_L*
   follower: grid(object, leader)   ─► A* ─► corners ─► /follower/goal_pose ... ─► at q_F*
```

## Files created / changed

```
src/
├── caging_control/
│   ├── caging_control/core/approach.py      # NEW: approach_distance, approach_poses (Eq. 13)
│   ├── caging_control/core/astar.py         # NEW: Grid, astar, line_is_free, simplify_path
│   ├── caging_control/nodes/approach_manager.py  # NEW: plans and drives the approach
│   ├── test/test_approach_astar.py          # NEW: 7 unit tests
│   ├── setup.py, package.xml                # + approach_manager, std_srvs
├── caging_bringup/
│   ├── config/approach.yaml                 # NEW: r_O, l_g, inflation, grid, tolerances
│   ├── launch/approach.launch.py            # NEW: Step 5 scene + approach_manager
│   └── launch/two_robots.launch.py          # + object_yaw argument
└── caging_gazebo/rviz/two_robots.rviz       # + A* grid, paths, approach poses
```

| Topic / service | Type | Meaning |
|---|---|---|
| `/start_approach` | `std_srvs/srv/Trigger` | start the approach |
| `/leader/goal_pose`, `/follower/goal_pose` | `geometry_msgs/msg/PoseStamped` | waypoints for the LQR |
| `/leader/planned_path`, `/follower/planned_path` | `nav_msgs/msg/Path` | A\* path (RViz) |
| `/approach/poses` | `geometry_msgs/msg/PoseArray` | $q_L^*$, $q_F^*$ (RViz, cyan arrows) |
| `/approach/grid` | `nav_msgs/msg/OccupancyGrid` | inflated obstacles of the last plan (RViz) |

## How to build and run

```bash
cd ~/caging_multirobot_ros2
colcon build
source install/setup.bash

colcon test --packages-select caging_control --event-handlers console_direct+   # 17 tests

# A) Table 1 scene
ros2 launch caging_bringup approach.launch.py
#    other terminal, when everything is loaded:
ros2 service call /start_approach std_srvs/srv/Trigger

# B) Object rotated 90 deg: the follower must go around the object
ros2 launch caging_bringup approach.launch.py object_yaw:=1.5708
ros2 service call /start_approach std_srvs/srv/Trigger
```

In RViz: the gray area is the inflated obstacle map of the current plan, the red/blue lines are
the A\* paths of the leader/follower, and the cyan arrows are $q_L^*$ and $q_F^*$.

## Expected result

Measured in headless tests (box):

| | Object yaw π (Table 1) | Object yaw π/2 |
|---|---|---|
| $q_L^*$, $q_F^*$ | $(0.42, 0, \pi)$, $(-0.42, 0, 0)$ | $(0, -0.42, \pi/2)$, $(0, 0.42, -\pi/2)$ |
| Leader path | 51 cells → 1 waypoint (straight) | 42 cells → 1 waypoint (straight) |
| Follower path | 50 cells → 1 waypoint (straight) | 61 cells → **3 waypoints** around the object |
| Final error | 4.5 mm, 0.000 rad (both) | 7.2 mm / 5.4 mm, 0.000 rad |
| Closest robot–object distance | 0.420 m | 0.347 m (contact at 0.296 m) |
| Closest robot–robot distance | 0.840 m | 0.388 m (contact at 0.31 m) |
| Object moved | 0 m | 0 m |

With the Table 1 poses the straight lines are already free, so A\* gives straight paths: the
second test shows why the planner is needed. The 0.347 m is a little below the inflated radius
0.355 m because the robot cuts the corners (theory section 6); the 0.05 m margin absorbs it.

## Exercises

1. Compute $q_L^*$ and $q_F^*$ by hand for $q_O = (0.5, 0.5, \pi/4)$. Check with
   `python3 -c "from caging_control.core.approach import approach_poses; print(approach_poses((0.5, 0.5, 0.785398), 0.15, 0.12))"`.
2. Why must the inflated object radius be smaller than $d_a$? What happens if you set
   `safety_margin: 0.2`? Try it.
3. Run A\* by hand on a 5 × 5 grid with a wall in the middle (pencil and paper): write $g$, $h$
   and $f$ of every expanded cell.
4. Replace the heuristic by `h = 0` (Dijkstra). Count the expanded cells (add a counter) for the
   π/2 test. Same path? How many more cells?
5. Set `resolution: 0.01`. How long does the plan take now (log the time)? And with 0.2 m?
   What goes wrong with very big cells?
6. Set `waypoint_tolerance: 0.3` and run the π/2 test. Measure the closest robot–object
   distance. Explain with theory section 6.
7. Why is it important that the robots move one after the other? Describe what could happen if
   both planned at the same time with the other robot at its *start* pose.
8. Remove `simplify_path` (send every cell). What do you see in the robot speed?
9. Comment out the line that frees the goal cell and run the unit test
   `test_every_object_orientation_has_a_path`. For which orientations does it fail, and why
   those? (Hint: draw the 5 cm cells around the approach pose for $\theta_O = 45°$.)
