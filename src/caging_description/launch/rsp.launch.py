"""
Robot State Publisher (RSP) launch file.

What it does:
  1. Converts the xacro model into a URDF (XML text).
  2. Starts robot_state_publisher, which:
       - publishes the URDF on the topic /robot_description
       - publishes the TF tree of the robot (base_link -> chassis_link, wheels, ...)

It is reused by other launch files (display.launch.py, and the Gazebo launch file).

Run alone:
  ros2 launch caging_description rsp.launch.py
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch arguments ---
    # use_sim_time = true  -> use the clock of Gazebo (/clock topic)
    # use_sim_time = false -> use the clock of the computer
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Robot description: xacro -> URDF (XML string) ---
    xacro_file = os.path.join(
        get_package_share_directory('caging_description'), 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # --- robot_state_publisher node ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation (Gazebo) clock if true'),
        rsp,
    ])
