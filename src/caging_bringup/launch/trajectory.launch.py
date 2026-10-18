"""
Step 9: transport the box along the circular trajectory of the paper (§2.5, Eq. 26-27).

Starts:
  1. Step 8 (caging.launch.py leader_follower:=true): scene with grippers, approach,
     closure, Leader-Follower
  2. trajectory_tracker with config/trajectory.yaml

Then:
  ros2 service call /start_approach std_srvs/srv/Trigger      # wait for "TRANSPORT mode"
  ros2 service call /start_trajectory std_srvs/srv/Trigger    # transfer + one 30 s lap

Arguments:
  orientation  static (experiment 1), tangent (experiment 2, default), radial
  reference    smooth (default, continuous goal) or waypoints (paper: 1 s steps)
  object_yaw   theta_O [rad] of the box at the start (Table 1: pi)

Run:
  ros2 launch caging_bringup trajectory.launch.py
  ros2 launch caging_bringup trajectory.launch.py orientation:=static
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory('caging_bringup')

    # --- 1. Steps 5-8 ---
    transport = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'caging.launch.py')),
        launch_arguments={
            'leader_follower': 'true',
            'object_yaw': LaunchConfiguration('object_yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # --- 2. Trajectory tracker (the orientation argument overrides the YAML) ---
    tracker = Node(
        package='caging_control',
        executable='trajectory_tracker',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'trajectory.yaml'),
                    {'use_sim_time': True, 'orientation': LaunchConfiguration('orientation'),
                     'reference': LaunchConfiguration('reference')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('orientation', default_value='tangent',
                              description='static (experiment 1), tangent (experiment 2), radial'),
        DeclareLaunchArgument('reference', default_value='smooth',
                              description='smooth (continuous goal) or waypoints (paper)'),
        DeclareLaunchArgument('object_yaw', default_value='3.1416',
                              description='Object orientation theta_O [rad] (Table 1: pi)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('caging_gazebo'), 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        transport,
        tracker,
    ])
