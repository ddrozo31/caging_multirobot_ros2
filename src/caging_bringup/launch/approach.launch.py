"""
Step 6: object approach with A* (paper §2.3, Fig. 7, Eq. 13).

Starts:
  1. the Step 5 scene (two_robots.launch.py): leader, follower, object, LQR per robot
  2. approach_manager (caging_control) with config/approach.yaml

Then start the approach (leader first, then follower):
  ros2 service call /start_approach std_srvs/srv/Trigger

Arguments:
  object      box (default), hemisphere, pyramid
  object_yaw  theta_O of the object [rad]. Table 1: pi (default).
              Try 1.5708: the follower must go AROUND the object (A* is needed).

Run:
  ros2 launch caging_bringup approach.launch.py
  ros2 launch caging_bringup approach.launch.py object:=hemisphere object_yaw:=1.5708
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

    # --- 1. Step 5 scene ---
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'two_robots.launch.py')),
        launch_arguments={
            'object': LaunchConfiguration('object'),
            'object_yaw': LaunchConfiguration('object_yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # --- 2. Approach manager ---
    manager = Node(
        package='caging_control',
        executable='approach_manager',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'approach.yaml'),
                    {'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='box',
                              description='box, hemisphere or pyramid'),
        DeclareLaunchArgument('object_yaw', default_value='3.1416',
                              description='Object orientation theta_O [rad] (Table 1: pi)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('caging_gazebo'), 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        scene,
        manager,
    ])
