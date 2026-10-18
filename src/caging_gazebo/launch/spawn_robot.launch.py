"""
Spawn ONE ROSMASTER X3 robot with a name (namespace) in a running Gazebo (Step 5).
Include it once per robot: name:=leader, name:=follower.

Everything of the robot lives under its name:
  nodes   /leader/robot_state_publisher, /leader/controller_manager, ...
  topics  /leader/robot_description, /leader/joint_states,
          /leader/mecanum_controller/cmd_vel, /leader/mecanum_controller/odom, ...
  frames  world -> leader/base_link -> leader/wheel_0_link ...   (base_link placed by the
          ground truth = "Vicon", as in the paper)
          world -> leader/odom   (only the frame of the odometry messages, red arrows in RViz)
  Gazebo  model "leader": /model/leader/cmd_vel, /model/leader/odometry, ...

Starts:
  1. robot_state_publisher in the namespace (caging_description/rsp.launch.py)
  2. ros_gz_sim "create"  -> spawns the model "<name>" from /<name>/robot_description
  3. controller spawners  -> joint_state_broadcaster and mecanum_controller of
                             /<name>/controller_manager; the mecanum_controller also loads
                             config/multi_robot_controllers.yaml (no odometry TF)
  4. static TF world -> <name>/odom (the odometry starts at the spawn pose)

The bridge entries of each robot are in config/topic_bridge.yaml (started by world.launch.py).

Run (Gazebo must be running: world.launch.py):
  ros2 launch caging_gazebo spawn_robot.launch.py name:=leader color:=red x:=0.5 y:=-2.5 yaw:=1.5708
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    # Arguments as plain Python strings
    name = LaunchConfiguration('name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    color = LaunchConfiguration('color').perform(context)

    # --- 1. robot_state_publisher in the namespace <name> ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('caging_description'), 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'namespace': name, 'color': color}.items(),
    )

    # --- 2. Spawn the model "<name>" ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', x, '-y', y, '-z', '0.01', '-Y', yaw,
        ],
    )

    # --- 3. Controllers of THIS robot: tell the spawner which controller_manager ---
    controller_manager = f'/{name}/controller_manager'
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', controller_manager],
    )
    # --param-file: extra parameters on top of ros2_controllers.yaml (publish_tf: false)
    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['mecanum_controller', '--controller-manager', controller_manager,
                   '--param-file', os.path.join(get_package_share_directory('caging_gazebo'),
                                                'config', 'multi_robot_controllers.yaml')],
    )
    spawners = [joint_state_spawner, mecanum_spawner]

    # --- 4. Static TF world -> <name>/odom ---
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', x, '--y', y, '--yaw', yaw,
                   '--frame-id', 'world', '--child-frame-id', f'{name}/odom'],
        parameters=[{'use_sim_time': True}],
    )

    return [rsp, spawn, *spawners, world_to_odom]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='leader',
                              description='Robot name = namespace = Gazebo model name'),
        DeclareLaunchArgument('color', default_value='white',
                              description='Chassis color: white, red, blue, ...'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw [rad]'),
        OpaqueFunction(function=launch_setup),
    ])
