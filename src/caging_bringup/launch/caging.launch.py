"""
Step 7: approach (Step 6) + object closure with the soft grippers (paper §2.1.2).

Starts:
  1. the Step 5 scene with grippers (two_robots.launch.py gripper:=true)
  2. approach_manager with config/approach.yaml + config/caging.yaml
     (bigger robot footprint, do_closure: true)

Then:
  ros2 service call /start_approach std_srvs/srv/Trigger
  ros2 topic echo /object_closed          # becomes true when the box is caged

Arguments:
  object_yaw  theta_O [rad]: pi (Table 1, default) or e.g. 1.5708

Run:
  ros2 launch caging_bringup caging.launch.py
  ros2 launch caging_bringup caging.launch.py object_yaw:=1.5708
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

    # --- 1. Step 5 scene, robots WITH grippers, the box ---
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'two_robots.launch.py')),
        launch_arguments={
            'object': 'box',
            'object_yaw': LaunchConfiguration('object_yaw'),
            'gripper': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # --- 2. Approach + closure: caging.yaml is loaded last, so its values win ---
    manager = Node(
        package='caging_control',
        executable='approach_manager',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'approach.yaml'),
                    os.path.join(bringup_share, 'config', 'caging.yaml'),
                    {'use_sim_time': True}],
    )

    return LaunchDescription([
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
