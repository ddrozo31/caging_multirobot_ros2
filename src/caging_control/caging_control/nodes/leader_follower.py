"""
Leader-Follower controller of the follower robot (Step 8, paper §2.4, Eq. 14-25).

While the object is carried, the LEADER is driven by its LQR (Step 4) and the
FOLLOWER by this node, which keeps the formation eta = [d, alpha, theta_F] at
[d*, 0, theta_L - pi], so the object stays caged between the grippers.
The math is in core/leader_follower.py.

Every Ts seconds (only while the parameter "active" is true):
  1. q_L, q_F  = ground-truth poses ("Vicon")
  2. u_L       = body twist commanded to the leader (after twist_mux)
  3. u_F       = B^-1 (A u_L - p_d)                            Eq. 22-23
  4. saturate u_F, publish it on /follower/cmd_vel_nav (body twist, Eq. 14)

The node starts INACTIVE. approach_manager activates it when /object_closed is
true, after switching off the follower's LQR (the paper: "the follower robot
disabled the LQR controller and activated the Leader-Follower controller").
By hand:  ros2 param set /leader_follower active true

Subscribes:
  /leader/ground_truth/odom, /follower/ground_truth/odom   (nav_msgs/Odometry)
  /leader/mecanum_controller/cmd_vel                       (geometry_msgs/Twist)  u_L
Publishes:
  /follower/cmd_vel_nav   (geometry_msgs/Twist)            u_F (twist_mux priority 10)
  /formation_error        (geometry_msgs/Vector3Stamped)   x = e_d [m], y = e_alpha, z = e_thF [rad]

Parameters (config/leader_follower.yaml):
  active                 start / stop the controller                         false
  k_d, k_alpha, k_thf    gains (Table 1)                                     2, 1, 1
  use_measured_distance  true: d* = d measured when activated                true
  distance_ref           d* [m] if use_measured_distance is false            0.52
  alpha_ref              alpha* [rad]                                        0.0
  sample_time            Ts [s]                                              0.05
  max_linear_speed, max_angular_speed  limits of u_F [m/s, rad/s]           0.3, 1.0
  leader_cmd_timeout     u_L = 0 if the leader command is older [s]          0.5
  pose_timeout           stop if a ground-truth pose is older [s]            0.5
"""
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from caging_control.core.geometry import yaw_from_quaternion
from caging_control.core.leader_follower import follower_command, formation_state
from caging_control.core.lqr import saturate


def pose_from_odometry(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


class LeaderFollower(Node):

    def __init__(self):
        super().__init__('leader_follower')
        defaults = {
            'active': False,
            'k_d': 2.0, 'k_alpha': 1.0, 'k_thf': 1.0,
            'use_measured_distance': True,
            'distance_ref': 0.52,
            'alpha_ref': 0.0,
            'sample_time': 0.05,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'leader_cmd_timeout': 0.5,
            'pose_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.d_ref = self.p['distance_ref']

        # --- Latest data and when it arrived ---
        self.poses = {'leader': None, 'follower': None}
        self.pose_time = {'leader': None, 'follower': None}
        self.u_leader = (0.0, 0.0, 0.0)
        self.u_leader_time = None

        for robot in self.poses:
            self.create_subscription(Odometry, f'/{robot}/ground_truth/odom',
                                     lambda msg, robot=robot: self.pose_callback(robot, msg), 10)
        self.create_subscription(Twist, '/leader/mecanum_controller/cmd_vel', self.leader_cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/follower/cmd_vel_nav', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, '/formation_error', 10)
        self.create_timer(self.p['sample_time'], self.control_step)
        self.create_timer(1.0, self.print_status)
        self.last_error = None

        # "active" (and the others) can be changed while running
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(
            f"Leader-Follower ready (inactive). Gains k_d={self.p['k_d']}, "
            f"k_alpha={self.p['k_alpha']}, k_thF={self.p['k_thf']}")

    # ----------------------------------------------------------------- inputs
    def pose_callback(self, robot, msg):
        self.poses[robot] = pose_from_odometry(msg)
        self.pose_time[robot] = self.get_clock().now()

    def leader_cmd_callback(self, msg):
        self.u_leader = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.u_leader_time = self.get_clock().now()

    def age(self, stamp):
        """Seconds since `stamp` (infinite if never received)."""
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def parameters_callback(self, parameters):
        for parameter in parameters:
            self.p[parameter.name] = parameter.value
            if parameter.name == 'active' and parameter.value:
                self.activate()
            elif parameter.name == 'active':
                self.get_logger().info('Leader-Follower DEACTIVATED')
            elif parameter.name == 'distance_ref':
                self.d_ref = parameter.value
                self.get_logger().info(f'd* = {self.d_ref:.3f} m')
        return SetParametersResult(successful=True)

    def activate(self):
        if self.p['use_measured_distance'] and None not in self.poses.values():
            # d* = the distance at the moment of the closure: keep the grip as it was closed
            self.d_ref, _, _ = formation_state(self.poses['leader'], self.poses['follower'])
        self.get_logger().info(
            f"Leader-Follower ACTIVE: d* = {self.d_ref:.3f} m "
            f"({'measured' if self.p['use_measured_distance'] else 'parameter'}), "
            f"alpha* = {self.p['alpha_ref']}, theta_F* = theta_L - pi")

    # ------------------------------------------------------------ control loop
    def control_step(self):
        if not self.p['active']:
            return
        # Safety: without fresh poses we do not publish; the mecanum controller's
        # command timeout (Step 5) then stops the follower.
        if max(self.age(self.pose_time['leader']), self.age(self.pose_time['follower'])) > self.p['pose_timeout']:
            self.get_logger().warn('Ground truth too old: follower command stopped', throttle_duration_sec=2.0)
            return
        # Old leader command -> the leader is not moving
        u_leader = self.u_leader if self.age(self.u_leader_time) < self.p['leader_cmd_timeout'] else (0.0, 0.0, 0.0)

        # Eq. 22-23
        u_f, e = follower_command(self.poses['leader'], self.poses['follower'], u_leader,
                                  self.d_ref, self.p['alpha_ref'],
                                  (self.p['k_d'], self.p['k_alpha'], self.p['k_thf']))
        vx, vy, wz = saturate(*u_f, self.p['max_linear_speed'], self.p['max_angular_speed'])

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.cmd_pub.publish(cmd)

        error = Vector3Stamped()
        error.header.stamp = self.get_clock().now().to_msg()
        error.vector.x, error.vector.y, error.vector.z = e
        self.error_pub.publish(error)
        self.last_error = e

    def print_status(self):
        if self.p['active'] and self.last_error is not None:
            e_d, e_alpha, e_thf = self.last_error
            self.get_logger().info(
                f'formation error: e_d = {e_d * 1000:+.1f} mm, e_alpha = {math.degrees(e_alpha):+.2f} deg, '
                f'e_thF = {math.degrees(e_thf):+.2f} deg')


def main():
    rclpy.init()
    node = LeaderFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
