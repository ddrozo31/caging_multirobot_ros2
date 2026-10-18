"""
Robot State Publisher (RSP) launch file.

What it does:
  1. Converts the xacro model into a URDF (XML text).
  2. Starts robot_state_publisher, which:
       - publishes the URDF on the topic robot_description
       - publishes the TF tree of the robot (base_link -> chassis_link, wheels, ...)

It is reused by other launch files (display.launch.py, and the Gazebo launch files).

Arguments:
  use_sim_time  true = use the Gazebo clock
  namespace     "" (default) = one robot, as in Steps 1-4
                "leader"     = one of several robots (Step 5):
                               topics  -> /leader/robot_description, /leader/joint_states
                               frames  -> leader/base_link, leader/wheel_0_link, ...
  color         chassis color: white (default), red, blue, ...  (Step 5)

Run alone:
  ros2 launch caging_description rsp.launch.py
  ros2 launch caging_description rsp.launch.py namespace:=leader color:=red
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    # OpaqueFunction gives us the arguments as plain Python strings,
    # so the rest of this function is normal Python.
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    color = LaunchConfiguration('color').perform(context)

    # --- Robot description: xacro -> URDF (XML string) ---
    # namespace and color go to xacro as arguments (namespace: Gazebo plugins; color: chassis).
    xacro_file = os.path.join(
        get_package_share_directory('caging_description'), 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file, mappings={'namespace': namespace, 'color': color}).toxml()

    # frame_prefix: "leader/" adds the robot name to every TF frame of the robot.
    # One robot: no prefix, frames stay "base_link", "wheel_0_link", ...
    frame_prefix = namespace + '/' if namespace else ''

    # --- robot_state_publisher node ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,   # "" = no namespace
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix,
        }],
    )
    return [rsp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot name when there are several robots (e.g. leader); empty = one robot'),
        DeclareLaunchArgument(
            'color',
            default_value='white',
            description='Chassis color (a material of properties.xacro: white, red, blue, ...)'),
        OpaqueFunction(function=launch_setup),
    ])
