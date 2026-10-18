"""
LQR pose controller of ONE robot (Step 4, paper §2.2, Eq. 7-12).

Drives the robot from where it is (ground truth = our "virtual Vicon") to a
goal pose (x*, y*, theta*). The math is in core/lqr.py; this node only moves
data between ROS topics and those functions.

Every Ts seconds:
  1. q  = latest ground-truth pose               (/ground_truth/odom)
  2. v  = K (q* - q)                              Eq. 10, K from Eq. 11-12
  3. v / Ts = velocity in the global frame {G}    Eq. 7-8 (feedback linearization)
  4. saturate to max_linear_speed / max_angular_speed
  5. rotate to the robot frame with R(theta)^T    inverse of Eq. 3
  6. publish the body twist on /cmd_vel_nav       -> twist_mux -> mecanum_controller,
                                                     which applies inverse Eq. 1 (wheels)

Steps 3 + 5 + 6 together are mu = B(theta)^-1 v (Eq. 7): the rotation is done
here, the inverse Jacobian J_R^-1 inside our C++ controller.

Topic names are RELATIVE (no "/" at the start), so they follow the namespace of the
node: with one robot "ground_truth/odom" is /ground_truth/odom; in namespace /leader it is
/leader/ground_truth/odom (Step 5).

Subscribes:
  ground_truth/odom  (nav_msgs/Odometry)        pose of the robot in "world"
  goal_pose          (geometry_msgs/PoseStamped) new goal, e.g. RViz "2D Goal Pose"
Publishes:
  cmd_vel_nav  (geometry_msgs/Twist)            body twist (priority 10 in twist_mux,
                                                so joystick and keyboard can override)
  lqr_error    (geometry_msgs/Vector3Stamped)   goal - pose: x = ex, y = ey [m], z = etheta [rad]

Parameters:
  sample_time        Ts [s] of the controller (the paper does not give it)    0.05
  q_weight, r_weight Q = q_weight I, R = r_weight I (Table 1)                  0.1, 0.8
  max_linear_speed   [m/s]                                                     0.3
  max_angular_speed  [rad/s]                                                   1.0
                     (both can be changed while running: ros2 param set ...)
  start_with_goal    true: use goal_x/goal_y/goal_yaw as first goal            false
  goal_x, goal_y, goal_yaw   first goal [m, m, rad]                            0, 0, 0
  position_tolerance, yaw_tolerance  only to print "goal reached" [m, rad]     0.01, 0.02
  active             false: publish nothing (Step 8, follower in formation)    true

Run:
  ros2 run caging_control lqr_controller --ros-args -p use_sim_time:=true
  ros2 run caging_control lqr_controller --ros-args -p use_sim_time:=true -r __ns:=/leader
  ros2 run caging_control lqr_controller --ros-args -p use_sim_time:=true \
    -p start_with_goal:=true -p goal_x:=1.0 -p goal_y:=0.5 -p goal_yaw:=0.0
Send a goal by hand:
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from caging_control.core.geometry import global_to_robot, pose_error, yaw_from_quaternion
from caging_control.core.lqr import lqr_step, paper_gain, saturate


class LqrController(Node):

    def __init__(self):
        super().__init__('lqr_controller')

        # --- Parameters ---
        self.declare_parameter('sample_time', 0.05)
        self.declare_parameter('q_weight', 0.1)
        self.declare_parameter('r_weight', 0.8)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('start_with_goal', False)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('yaw_tolerance', 0.02)
        # Step 8: false = the node stops publishing (the follower is then driven by the
        # Leader-Follower controller). Can be changed while running.
        self.declare_parameter('active', True)
        self.active = self.get_parameter('active').value

        self.Ts = self.get_parameter('sample_time').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        # --- LQR gain, computed ONCE (Eq. 11-12, Table 1) ---
        self.K, P = paper_gain(self.get_parameter('q_weight').value,
                               self.get_parameter('r_weight').value)
        self.get_logger().info(
            f'LQR: Ts = {self.Ts} s, K = diag({self.K[0, 0]:.4f}, {self.K[1, 1]:.4f}, '
            f'{self.K[2, 2]:.4f}), P = diag({P[0, 0]:.4f}, ...), '
            f'velocity gain K/Ts = {self.K[0, 0] / self.Ts:.2f} 1/s')

        # --- State: latest pose and current goal (None until known) ---
        self.pose = None
        self.goal = None
        self.goal_reached = False
        if self.get_parameter('start_with_goal').value:
            self.set_goal((self.get_parameter('goal_x').value,
                           self.get_parameter('goal_y').value,
                           self.get_parameter('goal_yaw').value))

        # --- Subscribers ---
        self.create_subscription(Odometry, 'ground_truth/odom', self.pose_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # --- Publishers and the control timer (one tick every Ts) ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, 'lqr_error', 10)
        self.create_timer(self.Ts, self.control_step)
        self.create_timer(1.0, self.print_status)   # human-readable log every 1 s
        self.last_error = None

        # The speed limits may change while the node runs (Step 7: slow closing),
        # e.g.  ros2 param set /leader/lqr_controller max_linear_speed 0.05
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'max_linear_speed':
                self.max_linear = parameter.value
                self.get_logger().info(f'max_linear_speed = {self.max_linear} m/s')
            elif parameter.name == 'max_angular_speed':
                self.max_angular = parameter.value
                self.get_logger().info(f'max_angular_speed = {self.max_angular} rad/s')
            elif parameter.name == 'active':
                self.active = parameter.value
                self.get_logger().info('LQR ACTIVE' if self.active else 'LQR switched OFF')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def pose_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def goal_callback(self, msg):
        if msg.header.frame_id not in ('world', ''):
            self.get_logger().warn(
                f'Goal in frame "{msg.header.frame_id}" ignored: it must be in "world" '
                '(set the RViz Fixed Frame to world)')
            return
        q = msg.pose.orientation
        self.set_goal((msg.pose.position.x, msg.pose.position.y,
                       yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def set_goal(self, goal):
        self.goal = goal
        self.goal_reached = False
        self.get_logger().info(
            f'New goal: x = {goal[0]:.3f} m, y = {goal[1]:.3f} m, yaw = {goal[2]:.3f} rad')

    # ------------------------------------------------------------ control loop
    def control_step(self):
        if not self.active:
            return  # switched off (Step 8): another controller drives this robot
        if self.pose is None or self.goal is None:
            return  # nothing to do yet: the robot stays free for teleoperation

        # Eq. 10: v = K (q* - q)  [pose change per sample, global frame]
        vx_step, vy_step, vtheta_step = lqr_step(self.K, self.pose, self.goal)

        # Eq. 7-8: the linear model moves v per sample, so the velocity is v / Ts
        vx = vx_step / self.Ts
        vy = vy_step / self.Ts
        wz = vtheta_step / self.Ts

        # Speed limits of the real robot (see the step guide)
        vx, vy, wz = saturate(vx, vy, wz, self.max_linear, self.max_angular)

        # Global frame {G} -> robot frame {R}: the controller expects a body twist
        vx_robot, vy_robot = global_to_robot(vx, vy, self.pose[2])

        cmd = Twist()
        cmd.linear.x = vx_robot
        cmd.linear.y = vy_robot
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        # Error goal - pose, for plotting (rqt_plot /lqr_error/vector/x ...)
        ex, ey, etheta, distance = pose_error(self.goal, self.pose)
        self.last_error = (ex, ey, etheta, distance)
        error_msg = Vector3Stamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'world'
        error_msg.vector.x = ex
        error_msg.vector.y = ey
        error_msg.vector.z = etheta
        self.error_pub.publish(error_msg)

        if (not self.goal_reached and distance < self.position_tolerance
                and abs(etheta) < self.yaw_tolerance):
            self.goal_reached = True
            self.get_logger().info(
                f'Goal reached: distance = {distance:.4f} m, heading = {etheta:+.4f} rad')

    def print_status(self):
        if self.pose is None:
            self.get_logger().info('Waiting for ground_truth/odom ...')
        elif self.goal is None:
            self.get_logger().info('Waiting for a goal on goal_pose (RViz "2D Goal Pose") ...')
        elif self.last_error is not None and not self.goal_reached:
            ex, ey, etheta, distance = self.last_error
            self.get_logger().info(
                f'to goal: distance = {distance:.3f} m, heading = {math.degrees(etheta):+.1f} deg')


def main():
    rclpy.init()
    node = LqrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
