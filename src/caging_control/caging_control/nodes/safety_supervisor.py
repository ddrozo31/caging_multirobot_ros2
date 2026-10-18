"""
Safety supervisor: if something goes wrong, stop BOTH robots (Step 10).

Each mecanum controller already stops its own robot when its commands stop
(Step 5, 0.5 s timeout). This node adds the rule "if one robot has a problem,
both must stop", because the object is held by the two of them.

While the task is running (/task_state not IDLE, DONE or ABORTED) it checks:
  (a) every ground-truth pose (leader, follower, object) is newer than pose_timeout
  (b) in TRANSPORT and TRAJECTORY (both robots driven all the time): the command
      of each robot (/<robot>/mecanum_controller/cmd_vel) is newer than cmd_timeout
  (c) in TRANSPORT and TRAJECTORY: the object is closer than cage_tolerance to the
      midpoint of the robots (the cage holds)
  (d) in TRANSPORT and TRAJECTORY: |e_d| (/formation_error) is smaller than distance_tolerance

If a check fails -> ABORT:
  - switch off every command source: both LQRs and the Leader-Follower (active = false)
  - with no commands, the 0.5 s timeout of each mecanum controller stops each robot
  - publish /safety/abort = true (task_manager -> ABORTED) and print the reason

Subscribes: /task_state, ground truth of the 3 bodies, both cmd_vel, /formation_error
Publishes:  /safety/abort (std_msgs/Bool, latched)

Run (started by task.launch.py):
  ros2 run caging_control safety_supervisor
"""
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

ROBOTS = ('leader', 'follower')
DRIVEN_STAGES = ('TRANSPORT', 'TRAJECTORY')   # both robots receive commands all the time


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        defaults = {'pose_timeout': 0.5, 'cmd_timeout': 0.5,
                    'cage_tolerance': 0.05, 'distance_tolerance': 0.03}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.abort_pub = self.create_publisher(Bool, '/safety/abort', latched)
        self.abort_pub.publish(Bool(data=False))

        self.state = 'IDLE'
        self.create_subscription(String, '/task_state', lambda m: setattr(self, 'state', m.data), latched)

        self.poses, self.pose_time = {}, {}
        for name in ('leader', 'follower', 'object'):
            self.create_subscription(Odometry, f'/{name}/ground_truth/odom',
                                     lambda m, name=name: self.pose_callback(name, m), 10)
        self.cmd_time = {}
        for robot in ROBOTS:
            self.create_subscription(Twist, f'/{robot}/mecanum_controller/cmd_vel',
                                     lambda m, robot=robot: self.cmd_time.__setitem__(robot, self.now()), 10)
        self.e_d = 0.0
        self.create_subscription(Vector3Stamped, '/formation_error',
                                 lambda m: setattr(self, 'e_d', m.vector.x), 10)

        # Clients to switch off the command sources
        self.param_clients = {name: self.create_client(SetParameters, f'{name}/set_parameters')
                        for name in ('/leader/lqr_controller', '/follower/lqr_controller', '/leader_follower')}
        self.aborted = False
        self.create_timer(0.1, self.check)

    def now(self):
        return self.get_clock().now()

    def age(self, stamp):
        return math.inf if stamp is None else (self.now() - stamp).nanoseconds * 1e-9

    def pose_callback(self, name, msg):
        p = msg.pose.pose.position
        self.poses[name] = (p.x, p.y)
        self.pose_time[name] = self.now()

    # ------------------------------------------------------------ checks
    def check(self):
        if self.aborted or self.state in ('IDLE', 'DONE', 'ABORTED'):
            return
        for name in ('leader', 'follower', 'object'):                          # (a)
            if self.age(self.pose_time.get(name)) > self.p['pose_timeout']:
                return self.abort(f'no ground truth of the {name} for {self.p["pose_timeout"]} s')
        if self.state not in DRIVEN_STAGES:
            return
        for robot in ROBOTS:                                                   # (b)
            if self.age(self.cmd_time.get(robot)) > self.p['cmd_timeout']:
                return self.abort(f'no command for the {robot} for {self.p["cmd_timeout"]} s')
        leader, follower, obj = self.poses['leader'], self.poses['follower'], self.poses['object']
        mid = ((leader[0] + follower[0]) / 2, (leader[1] + follower[1]) / 2)
        cage_error = math.dist(obj, mid)
        if cage_error > self.p['cage_tolerance']:                              # (c)
            return self.abort(f'object {cage_error:.3f} m from the midpoint: cage lost')
        if abs(self.e_d) > self.p['distance_tolerance']:                       # (d)
            return self.abort(f'formation distance error {self.e_d:.3f} m')

    def abort(self, reason):
        self.aborted = True
        self.get_logger().error(f'ABORT: {reason}. Stopping BOTH robots.')
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
        for client in self.param_clients.values():
            client.call_async(SetParameters.Request(parameters=[Parameter(name='active', value=value)]))
        self.abort_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
