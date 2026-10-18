"""
Task state machine: the whole experiment of the paper with ONE command (Step 10, §2.1, §3).

    IDLE ──/start_task──► APPROACH ──► CLOSURE ──► TRANSPORT ──► TRAJECTORY ──► DONE
                             │            │            │              │
                             └────────────┴─── failure or /safety/abort ──► ABORTED

It does not move the robots itself: it calls the services of the nodes of
Steps 6-9 and listens to their topics.

    stage           how we enter it                         how we leave it
    APPROACH        call /start_approach                    /approach/stage = closure
    CLOSURE         (approach_manager closes the object)    /object_closed = true
    TRANSPORT       (follower in Leader-Follower)           after transport_wait s: call /start_trajectory
    TRAJECTORY      transfer + one lap                      /trajectory/finished = true
    DONE            the object was carried around the circle

Subscribes:
  /approach/stage (String), /object_closed (Bool), /trajectory/finished (Bool), /safety/abort (Bool)
Publishes:
  /task_state (std_msgs/String, latched)
Service:
  /start_task (std_srvs/Trigger)

Parameters:
  auto_start       true: start by itself when everything is ready      false
  ready_delay      wait this long after everything is ready [s]          3.0
  transport_wait   time in TRANSPORT before the trajectory starts [s]    2.0

Run:
  ros2 launch caging_bringup task.launch.py
  ros2 service call /start_task std_srvs/srv/Trigger
  ros2 topic echo /task_state
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('ready_delay', 3.0)
        self.declare_parameter('transport_wait', 2.0)
        self.auto_start = self.get_parameter('auto_start').value
        self.ready_delay = self.get_parameter('ready_delay').value
        self.transport_wait = self.get_parameter('transport_wait').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/task_state', latched)

        # What the other nodes tell us
        self.approach_stage = ''
        self.closed = False
        self.finished = False
        self.create_subscription(String, '/approach/stage', lambda m: setattr(self, 'approach_stage', m.data), latched)
        self.create_subscription(Bool, '/object_closed', lambda m: setattr(self, 'closed', m.data), latched)
        self.create_subscription(Bool, '/trajectory/finished', lambda m: setattr(self, 'finished', m.data), latched)
        self.create_subscription(Bool, '/safety/abort', self.abort_callback, latched)

        # "Ready" = the ground truth of the three bodies arrives
        self.seen = set()
        for name in ('leader', 'follower', 'object'):
            self.create_subscription(Odometry, f'/{name}/ground_truth/odom',
                                     lambda m, name=name: self.seen.add(name), 10)

        self.approach_client = self.create_client(Trigger, '/start_approach')
        self.trajectory_client = self.create_client(Trigger, '/start_trajectory')
        self.create_service(Trigger, '/start_task', self.start_callback)

        self.state = None
        self.set_state('IDLE')
        self.ready_since = None
        self.create_timer(0.1, self.step)

    # ------------------------------------------------------------ helpers
    def set_state(self, state, reason=''):
        if state == self.state:
            return
        self.state = state
        self.state_since = self.get_clock().now()
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f'TASK STATE: {state}' + (f'  ({reason})' if reason else ''))

    def seconds_in_state(self):
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9

    def ready(self):
        return (len(self.seen) == 3 and self.approach_client.service_is_ready()
                and self.trajectory_client.service_is_ready())

    def abort_callback(self, msg):
        if msg.data and self.state not in ('IDLE', 'DONE', 'ABORTED'):
            self.set_state('ABORTED', 'safety supervisor')

    # -------------------------------------------------------------- start
    def start_callback(self, request, response):
        if self.state != 'IDLE':
            response.success, response.message = False, f'Task already started (state {self.state})'
        elif not self.ready():
            response.success, response.message = False, 'Not ready yet (ground truth or services missing)'
        else:
            self.start_task()
            response.success, response.message = True, 'Task started'
        return response

    def start_task(self):
        self.approach_client.call_async(Trigger.Request())
        self.set_state('APPROACH')

    # ----------------------------------------------------- state machine
    def step(self):
        if self.state == 'IDLE' and self.auto_start:
            if self.ready():
                if self.ready_since is None:
                    self.ready_since = self.get_clock().now()
                elif (self.get_clock().now() - self.ready_since).nanoseconds * 1e-9 > self.ready_delay:
                    self.get_logger().info('auto_start: everything is ready')
                    self.start_task()

        elif self.state == 'APPROACH':
            if self.approach_stage == 'closure':
                self.set_state('CLOSURE')
            elif self.approach_stage == 'failed':
                self.set_state('ABORTED', 'approach failed')

        elif self.state == 'CLOSURE':
            if self.closed:
                self.set_state('TRANSPORT', 'object closed')
            elif self.approach_stage == 'failed':
                self.set_state('ABORTED', 'closure failed')

        elif self.state == 'TRANSPORT':
            if self.seconds_in_state() > self.transport_wait:
                self.trajectory_client.call_async(Trigger.Request())
                self.set_state('TRAJECTORY')

        elif self.state == 'TRAJECTORY':
            if self.finished:
                self.set_state('DONE', 'lap finished')


def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
