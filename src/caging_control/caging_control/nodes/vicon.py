"""
Virtual Vicon with measurement noise (Step 11).

Gazebo gives PERFECT poses (/<body>/ground_truth/odom). A real motion-capture
system has small measurement errors. This node copies each ground-truth pose,
adds Gaussian noise (core/noise.py) and publishes it as the "Vicon measurement".

    /<body>/ground_truth/odom  --(+ noise)-->  /<body>/vicon/odom      body = leader, follower, object

Who uses what (Step 11):
  - the CONTROLLERS (LQR, Leader-Follower, approach manager) read /<body>/vicon/odom
    when the launch argument vicon_noise_xy or vicon_noise_yaw is > 0
    (their parameter pose_source = 'vicon');
  - the EVALUATION (CSV of trajectory_tracker, task_manager, safety_supervisor)
    keeps reading the ground truth, so the reported errors are the TRUE errors.

Parameters:
  bodies     (['leader', 'follower', 'object'])
  noise_xy   standard deviation of x and y [m]      (e.g. 0.002)
  noise_yaw  standard deviation of theta   [rad]    (e.g. 0.005)
  seed       random seed: same seed = same noise = the run can be repeated exactly

Run:
  ros2 run caging_control vicon --ros-args -p noise_xy:=0.002 -p noise_yaw:=0.005 -p seed:=1
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from caging_control.core.geometry import yaw_from_quaternion
from caging_control.core.noise import make_generator, noisy_pose


class Vicon(Node):

    def __init__(self):
        super().__init__('vicon')

        # --- Parameters ---
        self.declare_parameter('bodies', ['leader', 'follower', 'object'])
        self.declare_parameter('noise_xy', 0.0)
        self.declare_parameter('noise_yaw', 0.0)
        self.declare_parameter('seed', 0)
        self.noise_xy = self.get_parameter('noise_xy').value
        self.noise_yaw = self.get_parameter('noise_yaw').value
        seed = self.get_parameter('seed').value
        self.generator = make_generator(seed)   # ONE generator for all bodies

        # --- One subscriber and one publisher per body ---
        self.vicon_pubs = {}
        for body in self.get_parameter('bodies').value:
            self.vicon_pubs[body] = self.create_publisher(Odometry, f'/{body}/vicon/odom', 10)
            self.create_subscription(Odometry, f'/{body}/ground_truth/odom',
                                     lambda msg, body=body: self.measure(body, msg), 10)

        self.get_logger().info(
            f'virtual Vicon: noise sigma_xy = {self.noise_xy * 1000:.1f} mm, '
            f'sigma_yaw = {math.degrees(self.noise_yaw):.2f} deg, seed = {seed}')

    def measure(self, body, msg):
        """Ground-truth message -> the same message with a noisy pose."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        true_pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        x, y, theta = noisy_pose(true_pose, self.noise_xy, self.noise_yaw, self.generator)

        # Same header, frames and twist; only the pose changes
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)   # rotation theta about z
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        self.vicon_pubs[body].publish(msg)


def main():
    rclpy.init()
    node = Vicon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
