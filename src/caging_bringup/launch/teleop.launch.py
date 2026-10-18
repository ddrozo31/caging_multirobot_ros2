"""
Teleoperation of the robot with a joystick and/or the keyboard.

Starts:
  1. joy_node     -> reads the joystick, publishes joy
  2. teleop_node  -> converts joy into a Twist on cmd_vel_joy
  3. twist_mux    -> picks the input with the highest priority and sends it
                     to mecanum_controller/cmd_vel

All parameters are in config/teleop.yaml.

Arguments:
  namespace   "" (default) = one robot (Steps 2-4): topics /cmd_vel_joy, /mecanum_controller/cmd_vel
              "leader"     = robot "leader" (Step 5): /leader/cmd_vel_joy, /leader/mecanum_controller/cmd_vel
  joy_device  joystick number: 0 = /dev/input/js0, 1 = /dev/input/js1 (one joystick per robot)

Run (with the simulation already running):
  ros2 launch caging_bringup teleop.launch.py
  ros2 launch caging_bringup teleop.launch.py namespace:=follower joy_device:=1

Keyboard (needs its own terminal, because it reads the keys you type):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/leader -r cmd_vel:=cmd_vel_key
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    teleop_params = os.path.join(
        get_package_share_directory('caging_bringup'), 'config', 'teleop.yaml')
    namespace = LaunchConfiguration('namespace')
    joy_device = LaunchConfiguration('joy_device')

    # --- 1. Joystick driver ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',   # must match the name used in teleop.yaml
        namespace=namespace,
        parameters=[teleop_params, {
            'use_sim_time': True,
            'device_id': ParameterValue(joy_device, value_type=int),
        }],
    )

    # --- 2. Joystick -> Twist ---
    # teleop_node publishes on "cmd_vel"; we rename (remap) it to "cmd_vel_joy"
    # so that twist_mux knows the command comes from the joystick.
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        namespace=namespace,
        parameters=[teleop_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_joy')],
    )

    # --- 3. Multiplexer: several inputs -> one output ---
    # twist_mux publishes on "cmd_vel_out"; we remap it to the controller input.
    # Relative name (no "/" at the start): it gets the namespace, e.g.
    # /leader/mecanum_controller/cmd_vel, so each robot has its own twist_mux.
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace=namespace,
        parameters=[teleop_params, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'mecanum_controller/cmd_vel')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot name (leader, follower); empty = one robot'),
        DeclareLaunchArgument('joy_device', default_value='0',
                              description='Joystick number (/dev/input/jsN)'),
        joy_node,
        teleop_node,
        twist_mux_node,
    ])
