"""
Object transport along the circular trajectory of the paper
(Step 9, §2.4.2, §2.5, Eq. 26-27).

The OBJECT is the control point: each waypoint is a desired object pose q_O*,
converted with Eq. 26 into a goal for the leader's LQR. The follower keeps the
cage with the Leader-Follower controller (Step 8).

When /start_trajectory is called (only once the object is closed):
  1. TRANSFER: carry the object straight to the first waypoint (1, 0, theta*(0))
     at the transport speed, and wait until the leader is there
  2. LAP, two kinds of reference (parameter "reference"):
       smooth    (default): the object goal moves continuously, phi(t) = 2 pi t / duration,
                 sent at 20 Hz -> the robots drive at a constant speed (no stops)
       waypoints (paper):   waypoint i at t = i * duration / N (30 waypoints, 1 s each);
                 the LQR drives to each one and stops -> stop-and-go motion
     with faster limits (the circle needs about 0.21 m/s)
  3. END: hold the last waypoint and print the errors (like Table 2 of the paper)

Subscribes:
  /leader/ground_truth/odom, /follower/ground_truth/odom, /object/ground_truth/odom
  /object_closed (std_msgs/Bool)
Publishes:
  /leader/goal_pose          (geometry_msgs/PoseStamped)  leader goal from Eq. 26
  /trajectory/desired_path   (nav_msgs/Path)              the object waypoints (RViz)
  /trajectory/object_goal    (geometry_msgs/PoseStamped)  current desired object pose (RViz)
Service:
  /start_trajectory          (std_srvs/Trigger)

Parameters: see caging_bringup/config/trajectory.yaml

Run:
  ros2 launch caging_bringup trajectory.launch.py orientation:=tangent
  ros2 service call /start_approach std_srvs/srv/Trigger      # approach + closure + transport mode
  ros2 service call /start_trajectory std_srvs/srv/Trigger    # transfer + one lap
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from caging_control.core.geometry import pose_error, wrap_angle, yaw_from_quaternion
from caging_control.core.leader_follower import formation_state
from caging_control.core.trajectory import circle_pose, circle_waypoints, leader_goal_from_object


def pose_from_odometry(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


def to_pose_msg(x, y, theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(theta / 2.0)
    pose.orientation.w = math.cos(theta / 2.0)
    return pose


class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')
        defaults = {
            'orientation': 'tangent',   # static (experiment 1), tangent (experiment 2), radial
            'reference': 'smooth',      # smooth (phi(t) at 20 Hz) or waypoints (paper: 1 s steps)
            'n_waypoints': 30,          # N (§2.5)
            'duration': 30.0,           # time for one lap [s] (§2.5)
            'radius': 1.0,              # [m]
            'center_x': 0.0, 'center_y': 0.0,
            'static_heading': math.pi,  # theta* of experiment 1 [rad]
            'transfer_speed': 0.1, 'transfer_angular_speed': 0.3,   # leader limits, transfer
            'lap_speed': 0.3, 'lap_angular_speed': 0.5,             # leader limits, lap
            'follower_max_speed': 0.4,  # Leader-Follower limit during the lap [m/s]
            'arrival_tolerance': 0.02,  # transfer finished: leader closer than this [m]
            'arrival_yaw_tolerance': 0.03,   # [rad]
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.waypoints = circle_waypoints(
            self.p['n_waypoints'], self.p['radius'], (self.p['center_x'], self.p['center_y']),
            self.p['orientation'], self.p['static_heading'])

        # --- Inputs ---
        self.poses = {'leader': None, 'follower': None, 'object': None}
        for name in self.poses:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth/odom',
                lambda msg, name=name: self.poses.__setitem__(name, pose_from_odometry(msg)), 10)
        self.closed = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/object_closed', lambda msg: setattr(self, 'closed', msg.data), latched)

        # --- Outputs ---
        self.goal_pub = self.create_publisher(PoseStamped, '/leader/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory/desired_path', latched)
        self.object_goal_pub = self.create_publisher(PoseStamped, '/trajectory/object_goal', 10)
        self.param_clients = {
            'leader': self.create_client(SetParameters, '/leader/lqr_controller/set_parameters'),
            'leader_follower': self.create_client(SetParameters, '/leader_follower/set_parameters'),
        }
        self.create_service(Trigger, '/start_trajectory', self.start_callback)
        self.create_timer(0.05, self.step)   # 20 Hz: waypoint timing and error samples

        self.stage = 'idle'      # idle -> transfer -> lap -> done
        self.publish_path()
        self.get_logger().info(
            f"Ready: {self.p['orientation']} circle, N = {self.p['n_waypoints']}, "
            f"{self.p['duration']} s. Start with: ros2 service call /start_trajectory std_srvs/srv/Trigger")

    # ------------------------------------------------------------ helpers
    def set_remote_parameter(self, node, name, value):
        request = SetParameters.Request(parameters=[Parameter(
            name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))])
        self.param_clients[node].call_async(request)

    def send_object_goal(self, object_goal):
        """Eq. 26: object goal -> leader goal, sent to the leader's LQR."""
        leader_goal = leader_goal_from_object(object_goal, self.distance)
        now = self.get_clock().now().to_msg()
        for publisher, pose in ((self.goal_pub, leader_goal), (self.object_goal_pub, object_goal)):
            msg = PoseStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = now
            msg.pose = to_pose_msg(*pose)
            publisher.publish(msg)

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'world'
        for waypoint in self.waypoints:
            stamped = PoseStamped()
            stamped.header.frame_id = 'world'
            stamped.pose = to_pose_msg(*waypoint)
            path.poses.append(stamped)
        self.path_pub.publish(path)

    def seconds_since(self, start):
        return (self.get_clock().now() - start).nanoseconds * 1e-9

    # ------------------------------------------------------------- start
    def start_callback(self, request, response):
        if not self.closed:
            response.success, response.message = False, 'The object is not closed yet (/start_approach first)'
            return response
        if self.stage in ('transfer', 'lap'):
            response.success, response.message = False, f'Already running ({self.stage})'
            return response
        # d of Eq. 26: the distance between the robots now (= d* of the Leader-Follower)
        self.distance, _, _ = formation_state(self.poses['leader'], self.poses['follower'])
        self.set_remote_parameter('leader', 'max_linear_speed', self.p['transfer_speed'])
        self.set_remote_parameter('leader', 'max_angular_speed', self.p['transfer_angular_speed'])
        self.send_object_goal(self.waypoints[0])
        self.stage = 'transfer'
        self.get_logger().info(
            f'TRANSFER to the first waypoint ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f}, '
            f'{self.waypoints[0][2]:.2f}), d = {self.distance:.3f} m')
        response.success, response.message = True, 'Transfer started'
        return response

    # ---------------------------------------------------------- the loop
    def step(self):
        if self.stage == 'transfer':
            leader_goal = leader_goal_from_object(self.waypoints[0], self.distance)
            _, _, etheta, distance = pose_error(self.poses['leader'], leader_goal)
            if distance < self.p['arrival_tolerance'] and abs(etheta) < self.p['arrival_yaw_tolerance']:
                self.set_remote_parameter('leader', 'max_linear_speed', self.p['lap_speed'])
                self.set_remote_parameter('leader', 'max_angular_speed', self.p['lap_angular_speed'])
                self.set_remote_parameter('leader_follower', 'max_linear_speed', self.p['follower_max_speed'])
                self.lap_start = self.get_clock().now()
                self.index = -1
                self.samples = []
                self.stage = 'lap'
                self.get_logger().info(
                    f"LAP started ({self.p['orientation']}, {self.p['reference']} reference, {self.p['duration']} s)")

        elif self.stage == 'lap':
            elapsed = self.seconds_since(self.lap_start)
            if self.p['reference'] == 'smooth':
                # The goal moves continuously along the circle: phi(t) = 2 pi t / duration
                phi = 2.0 * math.pi * min(elapsed / self.p['duration'], 1.0)
                object_goal = circle_pose(phi, self.p['radius'], (self.p['center_x'], self.p['center_y']),
                                          self.p['orientation'], self.p['static_heading'])
                self.send_object_goal(object_goal)
            else:
                # Waypoint i is active from t = i * duration / N   (1 s each for N = 30, 30 s)
                index = min(int(elapsed / (self.p['duration'] / self.p['n_waypoints'])),
                            self.p['n_waypoints'] - 1)
                object_goal = self.waypoints[index]
                if index != self.index:
                    self.index = index
                    self.send_object_goal(object_goal)
            self.record_errors(object_goal)
            if elapsed >= self.p['duration']:
                self.stage = 'done'
                self.print_summary()

    def record_errors(self, object_goal):
        """One sample of the errors of Table 2 (against the waypoint active now)."""
        leader, follower, obj = self.poses['leader'], self.poses['follower'], self.poses['object']
        d, alpha, theta_f = formation_state(leader, follower)
        self.samples.append({
            'e_dO': math.hypot(obj[0] - object_goal[0], obj[1] - object_goal[1]),
            'e_xO': abs(obj[0] - object_goal[0]),
            'e_yO': abs(obj[1] - object_goal[1]),
            'e_thL': abs(wrap_angle(leader[2] - object_goal[2])),   # theta_MRS = theta_L
            'e_d': abs(d - self.distance),
            'e_thF': abs(wrap_angle(theta_f - (leader[2] - math.pi))),
            'e_alpha': abs(alpha),
            # extra: object distance to the circle and to the midpoint of the robots (caged?)
            'path': abs(math.hypot(obj[0] - self.p['center_x'], obj[1] - self.p['center_y']) - self.p['radius']),
            'cage': math.hypot(obj[0] - (leader[0] + follower[0]) / 2, obj[1] - (leader[1] + follower[1]) / 2),
        })

    def print_summary(self):
        n = len(self.samples)
        mean = {k: sum(s[k] for s in self.samples) / n for k in self.samples[0]}
        worst = {k: max(s[k] for s in self.samples) for k in ('path', 'cage', 'e_d')}
        paper = {'static': (0.10, 0.07, 0.06, 0.08, 0.01, 0.09, 0.06),
                 'tangent': (0.13, 0.09, 0.07, 0.15, 0.01, 0.10, 0.09),
                 'radial': (0.13, 0.09, 0.07, 0.15, 0.01, 0.10, 0.09)}[self.p['orientation']]
        lines = [f"LAP FINISHED ({self.p['orientation']}, {self.p['reference']} reference, {n} samples). "
                 "Mean errors (Table 2 box in brackets):"]
        for (key, unit), ref in zip([('e_dO', 'm'), ('e_xO', 'm'), ('e_yO', 'm'), ('e_thL', 'rad'),
                                     ('e_d', 'm'), ('e_thF', 'rad'), ('e_alpha', 'rad')], paper):
            lines.append(f'  {key:8s} {mean[key]:.3f} {unit:3s}  [{ref:.2f}]')
        lines.append(f"  object to circle: mean {mean['path']:.3f} m, max {worst['path']:.3f} m")
        lines.append(f"  object to midpoint (cage): mean {mean['cage'] * 1000:.1f} mm, max {worst['cage'] * 1000:.1f} mm")
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
