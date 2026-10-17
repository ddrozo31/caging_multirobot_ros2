"""
Display the robot model in RViz (no simulation).

Starts:
  1. robot_state_publisher      (from rsp.launch.py)   -> TF of the robot
  2. joint_state_publisher_gui                         -> sliders for the wheel joints
  3. rviz2                                             -> 3D view

Use it to check the model: dimensions, frames, and the direction in which
each wheel turns when you move its slider.

Run:
  ros2 launch caging_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('caging_description')

    # --- 1. Robot State Publisher (computer clock, no Gazebo here) ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # --- 2. Sliders to move the joints (publishes /joint_states) ---
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # --- 3. RViz with a saved configuration (RobotModel + TF + Grid) ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        rsp,
        jsp_gui,
        rviz,
    ])
