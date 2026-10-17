"""
Start Gazebo Harmonic and spawn ONE ROSMASTER X3 robot.

Starts:
  1. Gazebo (gz sim) with a world file
  2. robot_state_publisher (from caging_description/rsp.launch.py)
  3. ros_gz_sim "create"   -> reads /robot_description and spawns the robot
  4. ros_gz_bridge         -> copies topics between Gazebo and ROS 2
                              (list of topics in config/topic_bridge.yaml)
  5. controller spawners   -> start joint_state_broadcaster and mecanum_controller
  6. static TF world -> odom (the odometry starts where the robot is spawned)
  7. RViz (optional)       -> robot model (odometry) + ground truth

Drive the robot (body twist in the robot frame):
  ros2 topic pub -r 10 /mecanum_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"

Run:
  ros2 launch caging_gazebo sim.launch.py
  ros2 launch caging_gazebo sim.launch.py x:=0.5 y:=-2.5 yaw:=1.5708   # initial pose
  ros2 launch caging_gazebo sim.launch.py rviz:=false
  ros2 launch caging_gazebo sim.launch.py world:=/full/path/to/other_world.sdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory('caging_gazebo')
    description_share = get_package_share_directory('caging_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # --- Launch arguments ---
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')       # initial pose of the robot in the world [m]
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')   # [rad]
    rviz = LaunchConfiguration('rviz')

    # --- 1. Gazebo ---
    # gz_args: -r = start running (not paused), -v 4 = verbose messages
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'true',   # closing Gazebo also stops this launch
        }.items(),
    )

    # --- 2. Robot State Publisher, using the Gazebo clock ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # --- 3. Spawn the robot from the /robot_description topic ---
    # base_link is on the ground, so the robot is spawned at z = 0.01 m
    # (1 cm above the floor) and simply drops onto it.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', '0.01', '-Y', yaw,
        ],
    )

    # --- 4. ROS 2 <-> Gazebo bridge ---
    # All the topics that cross over are listed in ONE file: config/topic_bridge.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(gazebo_share, 'config', 'topic_bridge.yaml'),
        }],
    )

    # --- 5. Start the controllers ---
    # gz_ros2_control runs a controller_manager inside Gazebo. The "spawner"
    # waits for it, then loads, configures and activates each controller.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['mecanum_controller'],
    )

    # --- 6. Static TF: world -> odom ---
    # The wheel odometry starts at (0, 0, 0) in the "odom" frame. Placing "odom"
    # at the spawn pose makes odometry and ground truth start at the same point.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', x, '--y', y, '--yaw', yaw,
                   '--frame-id', 'world', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # --- 7. RViz: fixed frame "world" ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(gazebo_share, 'rviz', 'sim.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),   # only if rviz:=true
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_share, 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw [rad]'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        gazebo,
        rsp,
        spawn,
        bridge,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        world_to_odom,
        rviz_node,
    ])
