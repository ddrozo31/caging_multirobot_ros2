"""
Step 7: approach (Step 6) + object closure with the soft grippers (paper §2.1.2).
Step 8: with leader_follower:=true, after the closure the follower switches to the
Leader-Follower controller (paper §2.4) and the formation follows the leader.

Starts:
  1. the Step 5 scene with grippers (two_robots.launch.py gripper:=true)
  2. approach_manager with config/approach.yaml + config/caging.yaml
     (bigger robot footprint, do_closure: true)

Then:
  ros2 service call /start_approach std_srvs/srv/Trigger
  ros2 topic echo /object_closed          # becomes true when the box is caged

Arguments:
  object_yaw       theta_O [rad]: pi (Table 1, default) or e.g. 1.5708
  leader_follower  true (Step 8): start the Leader-Follower controller after the closure

Run:
  ros2 launch caging_bringup caging.launch.py
  ros2 launch caging_bringup caging.launch.py object_yaw:=1.5708
  ros2 launch caging_bringup caging.launch.py leader_follower:=true      # Step 8
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    # --- 2. Approach + closure (+ transport): later YAML files win ---
    def manager_and_formation(context):
        params = [os.path.join(bringup_share, 'config', 'approach.yaml'),
                  os.path.join(bringup_share, 'config', 'caging.yaml')]
        nodes = []
        if LaunchConfiguration('leader_follower').perform(context) == 'true':
            params.append(os.path.join(bringup_share, 'config', 'transport.yaml'))
            # --- 3. Leader-Follower controller of the follower (starts inactive) ---
            nodes.append(Node(
                package='caging_control',
                executable='leader_follower',
                output='screen',
                parameters=[os.path.join(bringup_share, 'config', 'leader_follower.yaml'),
                            {'use_sim_time': True}],
            ))
        nodes.append(Node(
            package='caging_control',
            executable='approach_manager',
            output='screen',
            parameters=params + [{'use_sim_time': True}],
        ))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('object_yaw', default_value='3.1416',
                              description='Object orientation theta_O [rad] (Table 1: pi)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('caging_gazebo'), 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        DeclareLaunchArgument('leader_follower', default_value='false',
                              description='Step 8: Leader-Follower after the closure'),
        scene,
        OpaqueFunction(function=manager_and_formation),
    ])
