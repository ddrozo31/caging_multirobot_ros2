"""
Object approach with A* (Step 6, paper §2.3, Fig. 7, Eq. 13).

When the service /start_approach is called:
  1. read the object pose from the ground truth ("Vicon")
  2. compute the approach poses q_L*, q_F*                 (Eq. 13, core/approach.py)
  3. LEADER: build the grid with the object and the follower as obstacles,
     plan with A*, keep only the corners, and send the corners one by one
     to /leader/goal_pose (the LQR of Step 4 drives the robot)
  4. when the leader is at q_L*, do the same for the FOLLOWER (the leader,
     now parked at q_L*, is the obstacle)
  5. done: both robots in front of the object, ready for the caging (Step 7)

The robots move ONE AFTER THE OTHER, as in the paper, so a plan never has to
deal with a moving robot.

Every waypoint carries the FINAL heading (theta*), so the robot turns while it
travels and arrives looking at the object. The robot is treated as a circle,
so turning near an obstacle is safe.

Subscribes:
  /object/ground_truth/odom, /leader/ground_truth/odom, /follower/ground_truth/odom
Publishes:
  /leader/goal_pose, /follower/goal_pose   (geometry_msgs/PoseStamped)  waypoints for the LQR
  /leader/planned_path, /follower/planned_path  (nav_msgs/Path)         for RViz
  /approach/poses   (geometry_msgs/PoseArray)       q_L* and q_F* for RViz
  /approach/grid    (nav_msgs/OccupancyGrid)         inflated obstacles of the last plan
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
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
            'robot_radius': 0.155,      # circle around the chassis [m]
            'safety_margin': 0.05,      # extra space around obstacles [m]
            'resolution': 0.05,         # grid cell size [m]
            'x_min': -1.5, 'x_max': 1.5, 'y_min': -3.0, 'y_max': 3.0,   # arena (Vicon area 3 x 6 m)
            'waypoint_tolerance': 0.10, # switch to the next corner closer than this [m]
            'position_tolerance': 0.01, # final pose reached [m]
            'yaw_tolerance': 0.02,      # final pose reached [rad]
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        # --- Latest ground-truth poses ---
        self.poses = {'object': None, 'leader': None, 'follower': None}
        for name in self.poses:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth/odom',
                lambda msg, name=name: self.poses.__setitem__(name, pose_from_odometry(msg)), 10)

        # --- Publishers ---
        self.goal_pubs = {r: self.create_publisher(PoseStamped, f'/{r}/goal_pose', 10) for r in ROBOTS}
        self.path_pubs = {r: self.create_publisher(Path, f'/{r}/planned_path', 10) for r in ROBOTS}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # RViz may come later
        self.poses_pub = self.create_publisher(PoseArray, '/approach/poses', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/approach/grid', latched)

        # --- Service and the loop that follows the waypoints (10 Hz) ---
        self.create_service(Trigger, '/start_approach', self.start_callback)
        self.create_timer(0.1, self.step)

        # --- State of the approach ---
        self.targets = {}        # robot -> approach pose (Eq. 13)
        self.queue = []          # robots still to move, in order
        self.robot = None        # robot moving now
        self.waypoints = []      # remaining corners of its path (x, y)
        self.get_logger().info('Ready. Start with:  ros2 service call /start_approach std_srvs/srv/Trigger')

    # --------------------------------------------------------------- start
    def start_callback(self, request, response):
        if self.robot is not None:
            response.success, response.message = False, f'Approach already running ({self.robot})'
            return response
        if None in self.poses.values():
            missing = [n for n, p in self.poses.items() if p is None]
            response.success, response.message = False, f'No ground truth yet for: {missing}'
            return response

        # Eq. 13 with the object pose measured NOW
        leader, follower = approach_poses(
            self.poses['object'], self.p['object_radius'], self.p['gripper_distance'])
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
        other = 'follower' if self.robot == 'leader' else 'leader'
        start = self.poses[self.robot]
        goal = self.targets[self.robot]

        # Grid with inflated obstacles: the object and the OTHER robot, where it is now
        grid = Grid(self.p['x_min'], self.p['x_max'], self.p['y_min'], self.p['y_max'], self.p['resolution'])
        r_robot, margin = self.p['robot_radius'], self.p['safety_margin']
        obj = self.poses['object']
        grid.add_circle(obj[0], obj[1], self.p['object_radius'] + r_robot + margin)
        grid.add_circle(self.poses[other][0], self.poses[other][1], r_robot + r_robot + margin)

        # The robot itself may start inside an inflated area (e.g. it was pushed):
        # free its own cell so A* can leave it.
        start_cell = grid.world_to_cell(start[0], start[1])
        if grid.inside(*start_cell):
            grid.occupied[start_cell] = False

        cells = astar(grid, start_cell, grid.world_to_cell(goal[0], goal[1]))
        self.publish_grid(grid)
        if cells is None:
            self.get_logger().error(f'{self.robot}: no collision-free path to its approach pose')
            self.robot = None
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
