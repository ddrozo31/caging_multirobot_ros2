"""
Object approach with A* (Step 6, paper §2.3, Fig. 7, Eq. 13)
and object closure / caging (Step 7, paper §2.1.2 and §3).

When the service /start_approach is called:
  1. read the object pose from the ground truth ("Vicon")
  2. compute the approach poses q_L*, q_F*                 (Eq. 13, core/approach.py)
  3. LEADER: build the grid with the object and the follower as obstacles,
     plan with A*, keep only the corners, and send the corners one by one
     to /leader/goal_pose (the LQR of Step 4 drives the robot)
  4. when the leader is at q_L*, do the same for the FOLLOWER (the leader,
     now parked at q_L*, is the obstacle)
  5. done: both robots in front of the object, ready for the caging (Step 7)
  6. (only if do_closure = true, Step 7) CLOSURE: both robots at the same time
     move slowly towards the object until their centers are d* apart:
         leader goal   = q_O + d*/2 [-cos(theta_O), -sin(theta_O)]
         follower goal = q_O + d*/2 [ cos(theta_O),  sin(theta_O)]
     (theta_O + grasp_offset: e.g. the box is taken by its corners)
     The object is CLOSED (caged) when both robots are at their goals, their
     distance is d* and the object center is between them -> /object_closed = true
  7. (only if start_leader_follower = true, Step 8) switch the follower from its
     LQR to the Leader-Follower controller, and slow the leader down to the
     transport speed. From now on, moving the leader moves the whole formation.

The robots move ONE AFTER THE OTHER, as in the paper, so a plan never has to
deal with a moving robot.

Every waypoint carries the FINAL heading (theta*), so the robot turns while it
travels and arrives looking at the object. The robot is treated as a circle,
so turning near an obstacle is safe.

Subscribes:
  /object/ground_truth/odom, /leader/ground_truth/odom, /follower/ground_truth/odom
      (or /<name>/vicon/odom with pose_source = 'vicon', Step 11)
Publishes:
  /leader/goal_pose, /follower/goal_pose   (geometry_msgs/PoseStamped)  waypoints for the LQR
  /leader/planned_path, /follower/planned_path  (nav_msgs/Path)         for RViz
  /approach/poses   (geometry_msgs/PoseArray)       q_L* and q_F* for RViz
  /approach/grid    (nav_msgs/OccupancyGrid)         inflated obstacles of the last plan
  /object_closed    (std_msgs/Bool)                  true when the object is caged (Step 7)
  /approach/stage   (std_msgs/String, latched)       approach_leader, approach_follower,
                                                     closure, transport, failed (Step 10)
Service:
  /start_approach   (std_srvs/Trigger)

Parameters: see caging_bringup/config/approach.yaml

Run:
  ros2 launch caging_bringup approach.launch.py
  ros2 service call /start_approach std_srvs/srv/Trigger
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from caging_control.core.approach import approach_poses
from caging_control.core.astar import Grid, astar, simplify_path
from caging_control.core.geometry import pose_error, yaw_from_quaternion

ROBOTS = ['leader', 'follower']   # order of the approach (paper: leader first)


def pose_from_odometry(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


def to_pose_msg(x, y, theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(theta / 2.0)   # rotation theta about z
    pose.orientation.w = math.cos(theta / 2.0)
    return pose


class ApproachManager(Node):

    def __init__(self):
        super().__init__('approach_manager')

        # --- Parameters (Table 1 and the Step 6 decisions) ---
        defaults = {
            'object_radius': 0.15,      # r_O [m]  (Table 1, the same for every object)
            'gripper_distance': 0.12,   # l_g [m]  (Table 1)
            'grasp_offset': 0.0,        # rotate the approach axis [rad] (box corners: pi/4, Step 7)
            'robot_radius': 0.155,      # circle around the chassis [m] (0.245 with gripper)
            'safety_margin': 0.05,      # extra space around the other robot [m]
            'object_margin': 0.05,      # extra space around the object [m] (0.02 with gripper)
            'resolution': 0.05,         # grid cell size [m]
            'x_min': -1.5, 'x_max': 1.5, 'y_min': -3.0, 'y_max': 3.0,   # arena (Vicon area 3 x 6 m)
            'waypoint_tolerance': 0.10, # switch to the next corner closer than this [m]
            'position_tolerance': 0.01, # final pose reached [m]
            'yaw_tolerance': 0.02,      # final pose reached [rad]
            # --- Closure (Step 7) ---
            'do_closure': False,        # true: after the approach, close on the object
            'closing_distance': 0.52,   # d* [m] between robot centers (Table 1)
            'closing_speed': 0.05,      # LQR speed limit while closing [m/s]
            'normal_speed': 0.3,        # LQR speed limit restored after closing [m/s]
            'closure_distance_tolerance': 0.01,  # |d - d*| [m]
            'closure_center_tolerance': 0.03,    # object center -> midpoint of the robots [m]
            'closure_timeout': 30.0,    # give up if the object is not closed after this [s]
            # --- Transport (Step 8) ---
            'start_leader_follower': False,  # true: after the closure, follower -> Leader-Follower
            'transport_speed': 0.1,          # leader LQR limits while carrying [m/s]
            'transport_angular_speed': 0.3,  # [rad/s]
            # --- Step 11 ---
            'pose_source': 'ground_truth',   # or 'vicon' (pose with measurement noise)
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        # --- Latest poses (ground truth, or Vicon with noise in Step 11) ---
        self.poses = {'object': None, 'leader': None, 'follower': None}
        for name in self.poses:
            self.create_subscription(
                Odometry, f"/{name}/{self.p['pose_source']}/odom",
                lambda msg, name=name: self.poses.__setitem__(name, pose_from_odometry(msg)), 10)

        # --- Publishers ---
        self.goal_pubs = {r: self.create_publisher(PoseStamped, f'/{r}/goal_pose', 10) for r in ROBOTS}
        self.path_pubs = {r: self.create_publisher(Path, f'/{r}/planned_path', 10) for r in ROBOTS}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # RViz may come later
        self.poses_pub = self.create_publisher(PoseArray, '/approach/poses', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/approach/grid', latched)
        self.closed_pub = self.create_publisher(Bool, '/object_closed', latched)
        self.closed_pub.publish(Bool(data=False))
        self.stage_pub = self.create_publisher(String, '/approach/stage', latched)

        # Clients to change parameters of other nodes: each LQR and the Leader-Follower
        self.param_clients = {r: self.create_client(SetParameters, f'/{r}/lqr_controller/set_parameters')
                              for r in ROBOTS}
        self.param_clients['leader_follower'] = self.create_client(
            SetParameters, '/leader_follower/set_parameters')

        # --- Service and the loop that follows the waypoints (10 Hz) ---
        self.create_service(Trigger, '/start_approach', self.start_callback)
        self.create_timer(0.1, self.step)

        # --- State of the approach ---
        self.targets = {}        # robot -> approach pose (Eq. 13)
        self.queue = []          # robots still to move, in order
        self.robot = None        # robot moving now
        self.waypoints = []      # remaining corners of its path (x, y)
        self.closing_goals = None  # robot -> closing goal (Step 7); None when not closing
        self.get_logger().info('Ready. Start with:  ros2 service call /start_approach std_srvs/srv/Trigger')

    # --------------------------------------------------------------- start
    def start_callback(self, request, response):
        if self.robot is not None or self.closing_goals is not None:
            response.success, response.message = False, 'Approach or closure already running'
            return response
        if None in self.poses.values():
            missing = [n for n, p in self.poses.items() if p is None]
            response.success, response.message = False, f'No ground truth yet for: {missing}'
            return response

        # Eq. 13 with the object pose measured NOW
        leader, follower = approach_poses(
            self.poses['object'], self.p['object_radius'], self.p['gripper_distance'],
            self.p['grasp_offset'])
        self.targets = {'leader': leader, 'follower': follower}
        self.get_logger().info(
            f"Object at ({self.poses['object'][0]:.3f}, {self.poses['object'][1]:.3f}, "
            f"{self.poses['object'][2]:.3f}).  q_L* = ({leader[0]:.3f}, {leader[1]:.3f}, {leader[2]:.3f}), "
            f"q_F* = ({follower[0]:.3f}, {follower[1]:.3f}, {follower[2]:.3f})")

        poses = PoseArray()
        poses.header.frame_id = 'world'
        poses.poses = [to_pose_msg(*leader), to_pose_msg(*follower)]
        self.poses_pub.publish(poses)

        self.queue = list(ROBOTS)
        ok = self.start_next_robot()
        response.success = ok
        response.message = 'Approach started (leader first)' if ok else 'No path found (see the log)'
        return response

    def start_next_robot(self):
        """Plan the path of the next robot in the queue and send its first waypoint."""
        self.robot = self.queue.pop(0)
        self.stage_pub.publish(String(data=f'approach_{self.robot}'))
        other = 'follower' if self.robot == 'leader' else 'leader'
        start = self.poses[self.robot]
        goal = self.targets[self.robot]

        # Grid with inflated obstacles: the object and the OTHER robot, where it is now
        grid = Grid(self.p['x_min'], self.p['x_max'], self.p['y_min'], self.p['y_max'], self.p['resolution'])
        r_robot, margin = self.p['robot_radius'], self.p['safety_margin']
        obj = self.poses['object']
        grid.add_circle(obj[0], obj[1], self.p['object_radius'] + r_robot + self.p['object_margin'])
        grid.add_circle(self.poses[other][0], self.poses[other][1], r_robot + r_robot + margin)

        # The robot itself may start inside an inflated area (e.g. it was pushed):
        # free its own cell so A* can leave it.
        start_cell = grid.world_to_cell(start[0], start[1])
        if grid.inside(*start_cell):
            grid.occupied[start_cell] = False

        # The goal: the approach pose is d_a = 0.42 m from the object, just outside the inflated
        # circle (0.355 m, or 0.415 m with the gripper). Its CELL CENTER can be up to 3.5 cm away
        # and fall inside the circle. If the exact pose is free, free its cell too.
        goal_cell = grid.world_to_cell(goal[0], goal[1])
        if not grid.point_is_free(goal[0], goal[1]):
            self.get_logger().error(
                f'{self.robot}: its approach pose ({goal[0]:.3f}, {goal[1]:.3f}) is inside an '
                'inflated obstacle (object margin too big, or the other robot is in the way)')
            self.publish_grid(grid)
            self.robot = None
            self.stage_pub.publish(String(data='failed'))
            return False
        if grid.inside(*goal_cell):
            grid.occupied[goal_cell] = False

        cells = astar(grid, start_cell, goal_cell)
        self.publish_grid(grid)
        if cells is None:
            self.get_logger().error(f'{self.robot}: no collision-free path to its approach pose')
            self.robot = None
            self.stage_pub.publish(String(data='failed'))
            return False

        # Cell centers -> keep the corners. The first point is the start (drop it),
        # the last cell center is replaced by the exact approach pose.
        points = simplify_path(grid, [grid.cell_to_world(*c) for c in cells])
        self.waypoints = points[1:-1] + [(goal[0], goal[1])]
        self.get_logger().info(
            f'{self.robot}: A* path of {len(cells)} cells -> {len(self.waypoints)} waypoints '
            + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in self.waypoints))
        self.publish_path(start, cells, grid)
        self.send_waypoint()
        return True

    # ---------------------------------------------------------- following
    def send_waypoint(self):
        x, y = self.waypoints[0]
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = to_pose_msg(x, y, self.targets[self.robot][2])   # always the FINAL heading
        self.goal_pubs[self.robot].publish(msg)

    def step(self):
        if self.closing_goals is not None:
            self.check_closure()
            return
        if self.robot is None:
            return
        pose = self.poses[self.robot]
        x, y = self.waypoints[0]
        distance = math.hypot(pose[0] - x, pose[1] - y)

        if len(self.waypoints) > 1:
            # Intermediate corner: close enough -> next one (no stop at every corner)
            if distance < self.p['waypoint_tolerance']:
                self.waypoints.pop(0)
                self.send_waypoint()
            return

        # Last waypoint = approach pose: wait until the LQR has really reached it
        _, _, etheta, distance = pose_error(pose, self.targets[self.robot])
        if distance < self.p['position_tolerance'] and abs(etheta) < self.p['yaw_tolerance']:
            self.get_logger().info(
                f'{self.robot} at its approach pose (error {distance:.4f} m, {etheta:+.4f} rad)')
            if self.queue:
                self.start_next_robot()
            else:
                self.get_logger().info('Approach finished: both robots in front of the object')
                self.robot = None
                if self.p['do_closure']:
                    self.start_closure()

    # ------------------------------------------------------- closure (Step 7)
    def set_remote_parameter(self, node, name, value):
        """Set a parameter of another node (like `ros2 param set`), without waiting.

        node: 'leader', 'follower' (their LQR) or 'leader_follower'.
        """
        if isinstance(value, bool):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        else:
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        request = SetParameters.Request(parameters=[Parameter(name=name, value=parameter_value)])
        self.param_clients[node].call_async(request)

    def set_lqr_speed(self, robot, speed):
        """Change the parameter max_linear_speed of the LQR of `robot`."""
        self.set_remote_parameter(robot, 'max_linear_speed', speed)

    def start_closure(self):
        """Both robots move at the same time, slowly, until they are d* apart."""
        x_o, y_o, theta_o = self.poses['object']   # measured again, after the approach
        theta_o += self.p['grasp_offset']           # same approach axis as Eq. 13 (corners)
        half = self.p['closing_distance'] / 2.0
        self.closing_goals = {
            'leader': (x_o - half * math.cos(theta_o), y_o - half * math.sin(theta_o),
                       self.targets['leader'][2]),
            'follower': (x_o + half * math.cos(theta_o), y_o + half * math.sin(theta_o),
                         self.targets['follower'][2]),
        }
        for robot, goal in self.closing_goals.items():
            self.set_lqr_speed(robot, self.p['closing_speed'])
            msg = PoseStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = to_pose_msg(*goal)
            self.goal_pubs[robot].publish(msg)
        self.closing_start = self.get_clock().now()
        self.stage_pub.publish(String(data='closure'))
        self.get_logger().info(
            f"Closing at {self.p['closing_speed']} m/s to d* = {self.p['closing_distance']} m: "
            + ', '.join(f'{r} -> ({g[0]:.3f}, {g[1]:.3f})' for r, g in self.closing_goals.items()))

    def check_closure(self):
        """Object closure condition (geometric, with the ground truth), checked every 0.1 s:

          1. each robot within position/yaw tolerance of its closing goal
          2. distance between the robots within closure_distance_tolerance of d*
          3. object center within closure_center_tolerance of the midpoint of the robots

        (1 alone is not enough: two robots each 1 cm short leave d 2 cm too long.)
        """
        leader, follower, obj = self.poses['leader'], self.poses['follower'], self.poses['object']
        at_goals = True
        for robot in ROBOTS:
            _, _, etheta, distance = pose_error(self.poses[robot], self.closing_goals[robot])
            if distance > self.p['position_tolerance'] or abs(etheta) > self.p['yaw_tolerance']:
                at_goals = False
        d = math.hypot(leader[0] - follower[0], leader[1] - follower[1])
        mid_x, mid_y = (leader[0] + follower[0]) / 2.0, (leader[1] + follower[1]) / 2.0
        center_error = math.hypot(obj[0] - mid_x, obj[1] - mid_y)
        closed = (at_goals
                  and abs(d - self.p['closing_distance']) < self.p['closure_distance_tolerance']
                  and center_error < self.p['closure_center_tolerance'])

        elapsed = (self.get_clock().now() - self.closing_start).nanoseconds * 1e-9
        if closed:
            self.get_logger().info(
                f'OBJECT CLOSED after {elapsed:.1f} s: d = {d:.3f} m (d* = {self.p["closing_distance"]}), '
                f'object {center_error:.3f} m from the midpoint')
            self.closed_pub.publish(Bool(data=True))
            if self.p['start_leader_follower']:
                self.start_transport()
                self.closing_goals = None
                return
        elif elapsed > self.p['closure_timeout']:
            self.get_logger().warn(
                f'NOT closed after {elapsed:.0f} s: d = {d:.3f} m, '
                f'object {center_error:.3f} m from the midpoint')
            self.stage_pub.publish(String(data='failed'))
        else:
            return   # keep waiting
        for robot in ROBOTS:
            self.set_lqr_speed(robot, self.p['normal_speed'])
        self.closing_goals = None

    def start_transport(self):
        """Step 8: follower LQR off -> Leader-Follower on; leader slowed down."""
        self.stage_pub.publish(String(data='transport'))
        self.set_remote_parameter('follower', 'active', False)          # 1. follower LQR off
        self.set_remote_parameter('leader_follower', 'active', True)    # 2. formation control on
        self.set_lqr_speed('leader', self.p['transport_speed'])         # 3. leader carries slowly
        self.set_remote_parameter('leader', 'max_angular_speed', self.p['transport_angular_speed'])
        self.get_logger().info(
            f"TRANSPORT mode: follower in Leader-Follower, leader limited to "
            f"{self.p['transport_speed']} m/s, {self.p['transport_angular_speed']} rad/s. "
            "Move the formation with goals for the leader (/leader/goal_pose).")

    # ------------------------------------------------------------ RViz
    def publish_path(self, start, cells, grid):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        theta = self.targets[self.robot][2]
        for x, y in [start[:2]] + [grid.cell_to_world(*c) for c in cells[1:]]:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose = to_pose_msg(x, y, theta)
            path.poses.append(stamped)
        self.path_pubs[self.robot].publish(path)

    def publish_grid(self, grid):
        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = grid.resolution
        msg.info.width = grid.cols
        msg.info.height = grid.rows
        msg.info.origin.position.x = grid.x_min
        msg.info.origin.position.y = grid.y_min
        msg.info.origin.orientation.w = 1.0
        # Row by row from y_min (the same order as grid.occupied): 100 = occupied, 0 = free
        msg.data = [100 if cell else 0 for cell in grid.occupied.flatten()]
        self.grid_pub.publish(msg)


def main():
    rclpy.init()
    node = ApproachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
