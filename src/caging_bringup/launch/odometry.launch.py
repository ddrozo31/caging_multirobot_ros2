"""
Step 3: simulation + wheel odometry vs. ground truth.

Starts:
  1. the simulation (caging_gazebo/sim.launch.py) at the initial pose (x, y, yaw)
  2. odom_error_monitor (caging_control): compares wheel odometry with ground truth

Both receive the SAME initial pose, so odometry and ground truth start together.

Run:
  ros2 launch caging_bringup odometry.launch.py
  ros2 launch caging_bringup odometry.launch.py x:=0.5 y:=-2.5 yaw:=1.5708
Then drive the robot (teleop.launch.py or ros2 topic pub) and watch the error.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    rviz = LaunchConfiguration('rviz')

    # --- 1. Simulation, passing our arguments down ---
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('caging_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={'x': x, 'y': y, 'yaw': yaw, 'rviz': rviz}.items(),
    )

    # --- 2. Odometry error monitor ---
    # ParameterValue(..., value_type=float): launch arguments are text; this makes
    # sure "1" is sent as the number 1.0 (the node declares float parameters).
    monitor = Node(
        package='caging_control',
        executable='odom_error_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'origin_x': ParameterValue(x, value_type=float),
            'origin_y': ParameterValue(y, value_type=float),
            'origin_yaw': ParameterValue(yaw, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw [rad]'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        sim,
        monitor,
    ])
