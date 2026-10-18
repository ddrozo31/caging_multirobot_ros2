"""
Start Gazebo Harmonic with a world, the ROS 2 <-> Gazebo bridge and RViz.
No robots: add them with spawn_robot.launch.py (Step 5, several robots).

Starts:
  1. Gazebo (gz sim) with a world file
  2. ros_gz_bridge  -> copies topics between Gazebo and ROS 2 (config/topic_bridge.yaml)
  3. RViz (optional) with the configuration given in "rviz_config"

Gazebo must find the object meshes ("model://caging_description/meshes/..."), so
GZ_SIM_RESOURCE_PATH gets the folder that contains the caging_description package.

Run (normally included by caging_bringup/two_robots.launch.py):
  ros2 launch caging_gazebo world.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory('caging_gazebo')
    description_share = get_package_share_directory('caging_description')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # "model://caging_description/meshes/x.stl" -> <this folder>/caging_description/meshes/x.stl
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share))

    # --- 1. Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --- 2. Bridge: ONE node for every robot and object (all topics in the YAML) ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(gazebo_share, 'config', 'topic_bridge.yaml'),
        }],
    )

    # --- 3. RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_share, 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(gazebo_share, 'rviz', 'two_robots.rviz'),
            description='RViz configuration file'),
        resource_path,
        gazebo,
        bridge,
        rviz_node,
    ])
