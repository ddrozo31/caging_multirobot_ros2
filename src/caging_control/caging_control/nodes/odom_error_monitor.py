"""
Odometry error monitor (Step 3).

Compares what the robot BELIEVES (wheel odometry) with what is TRUE
(ground truth from Gazebo, our "virtual Vicon").

Subscribes:
  /mecanum_controller/odom  (nav_msgs/Odometry)  pose in the "odom" frame
  /ground_truth/odom        (nav_msgs/Odometry)  pose in the "world" frame
Publishes:
  /odom_error  (geometry_msgs/Vector3Stamped)   x = ex [m], y = ey [m], z = etheta [rad]

Parameters:
  origin_x, origin_y, origin_yaw: pose of the "odom" frame in "world"
                                  (= the spawn pose of the robot)

Run:
  ros2 run caging_control odom_error_monitor
  ros2 run caging_control odom_error_monitor --ros-args -p origin_x:=0.5 -p origin_y:=-2.5 -p origin_yaw:=1.5708
"""
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from caging_control.core.geometry import pose_error, transform_pose, yaw_from_quaternion


def pose_from_odometry(msg):
    """nav_msgs/Odometry -> plain tuple (x, y, theta)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


class OdomErrorMonitor(Node):

    def __init__(self):
        super().__init__('odom_error_monitor')

        # --- Parameters ---
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)
        self.odom_origin = (self.get_parameter('origin_x').value,
                            self.get_parameter('origin_y').value,
                            self.get_parameter('origin_yaw').value)

        # --- Latest poses received (None until the first message) ---
        self.odom_pose = None    # wheel odometry, in "odom"
        self.true_pose = None    # ground truth,   in "world"

        # --- Subscribers: they only store the newest pose ---
        self.create_subscription(Odometry, '/mecanum_controller/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.truth_callback, 10)

        # --- Publisher and timers ---
        self.error_pub = self.create_publisher(Vector3Stamped, '/odom_error', 10)
        self.create_timer(0.1, self.publish_error)   # 10 Hz
        self.create_timer(2.0, self.print_error)     # human-readable log every 2 s
        self.last_error = None

    def odom_callback(self, msg):
        self.odom_pose = pose_from_odometry(msg)

    def truth_callback(self, msg):
        self.true_pose = pose_from_odometry(msg)

    def publish_error(self):
        if self.odom_pose is None or self.true_pose is None:
            return  # wait until both poses have arrived

        # The two poses are in DIFFERENT frames: bring the odometry into "world" first
        odom_in_world = transform_pose(self.odom_pose, self.odom_origin)
        self.last_error = pose_error(odom_in_world, self.true_pose)
        ex, ey, etheta, _ = self.last_error

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.vector.x = ex
        msg.vector.y = ey
        msg.vector.z = etheta
        self.error_pub.publish(msg)

    def print_error(self):
        if self.last_error is None:
            self.get_logger().info('Waiting for /mecanum_controller/odom and /ground_truth/odom ...')
            return
        ex, ey, etheta, distance = self.last_error
        self.get_logger().info(
            f'odometry error: distance = {distance:.3f} m '
            f'(ex = {ex:+.3f}, ey = {ey:+.3f}), heading = {etheta:+.3f} rad')


def main():
    rclpy.init()
    node = OdomErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
