"""
Step 5: two robots (leader and follower) and one object, at the initial poses of the
paper (Table 1, Fig. 9).

Starts:
  1. Gazebo + bridge + RViz              (caging_gazebo/world.launch.py)
  2. robot "leader"   at q_L = [ 0.5, -2.5, pi/2], red   (caging_gazebo/spawn_robot.launch.py)
  3. robot "follower" at q_F = [-0.5, -2.5, pi/2], blue
  4. the object       at q_O = [ 0.0,  0.0, pi  ]   box, hemisphere or pyramid,
     and object_marker, which draws it in RViz
  5. per robot: teleoperation (joy_node + teleop_node + twist_mux) and the LQR
     controller of Step 4, all inside the robot namespace

Each robot is controlled on its own:
  - joystick: leader = /dev/input/js0, follower = /dev/input/js1
  - keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/leader -r cmd_vel:=cmd_vel_key
  - LQR goal: RViz buttons "Leader goal" / "Follower goal", or
              ros2 topic pub --once /leader/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 0.5, y: -1.0}, orientation: {w: 1.0}}}"

Run:
  ros2 launch caging_bringup two_robots.launch.py
  ros2 launch caging_bringup two_robots.launch.py object:=hemisphere
  ros2 launch caging_bringup two_robots.launch.py object:=pyramid
  ros2 launch caging_bringup two_robots.launch.py object:=none
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Initial poses of the paper (Table 1): (x [m], y [m], yaw [rad])
ROBOTS = {
    'leader':   {'pose': (0.5, -2.5, 1.5708), 'joy_device': '0', 'color': 'red'},
    'follower': {'pose': (-0.5, -2.5, 1.5708), 'joy_device': '1', 'color': 'blue'},
}
OBJECT_POSE = (0.0, 0.0, 3.1416)


def launch_setup(context):
    gazebo_share = get_package_share_directory('caging_gazebo')
    bringup_share = get_package_share_directory('caging_bringup')
    description_share = get_package_share_directory('caging_description')

    object_name = LaunchConfiguration('object').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    world = LaunchConfiguration('world').perform(context)

    # --- 1. World: Gazebo + bridge + RViz ---
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'world.launch.py')),
        launch_arguments={'rviz': rviz, 'world': world}.items(),
    )]

    for name, robot in ROBOTS.items():
        x, y, yaw = robot['pose']

        # --- 2-3. Spawn the robot in its namespace ---
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'name': name, 'color': robot['color'],
                              'x': str(x), 'y': str(y), 'yaw': str(yaw)}.items(),
        ))

        # --- 5. Teleoperation of this robot ---
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'teleop.launch.py')),
            launch_arguments={'namespace': name, 'joy_device': robot['joy_device']}.items(),
        ))

        # --- 5. LQR controller of this robot (waits for a goal on /<name>/goal_pose) ---
        # PushRosNamespace puts every node of the group inside /<name>.
        actions.append(GroupAction([
            PushRosNamespace(name),
            Node(
                package='caging_control',
                executable='lqr_controller',
                output='screen',
                parameters=[os.path.join(bringup_share, 'config', 'lqr.yaml'),
                            {'use_sim_time': True}],
            ),
        ]))

    # --- 4. The object, named "object" in Gazebo (topics /model/object/...) ---
    if object_name != 'none':
        x, y, yaw = OBJECT_POSE
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'object',
                '-file', os.path.join(description_share, 'objects', object_name + '.sdf'),
                '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw),
            ],
        ))
        # RViz cannot draw the SDF: a Marker with the same shape follows the TF "object"
        actions.append(Node(
            package='caging_control',
            executable='object_marker',
            output='screen',
            parameters=[{'object': object_name, 'use_sim_time': True}],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='box',
                              description='box, hemisphere, pyramid or none'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('caging_gazebo'), 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        OpaqueFunction(function=launch_setup),
    ])
