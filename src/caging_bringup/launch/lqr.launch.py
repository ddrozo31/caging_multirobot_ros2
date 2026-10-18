"""
Step 4: drive ONE robot to a goal pose with the LQR controller.

Starts:
  1. the simulation (caging_gazebo/sim.launch.py) at the initial pose (x, y, yaw)
  2. teleoperation (teleop.launch.py): its twist_mux carries /cmd_vel_nav to the
     controller, and the joystick / keyboard can still take over at any time
  3. lqr_controller (caging_control) with config/lqr.yaml

Two ways to give the goal:
  - automatic test: start_with_goal:=true goal_x:=... goal_y:=... goal_yaw:=...
  - manual: RViz "2D Goal Pose" button (top toolbar), click and drag on the floor

Run:
  ros2 launch caging_bringup lqr.launch.py
  ros2 launch caging_bringup lqr.launch.py x:=0.5 y:=-2.5 yaw:=1.5708 \
      start_with_goal:=true goal_x:=0.0 goal_y:=-1.0 goal_yaw:=3.1416
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

    bringup_share = get_package_share_directory('caging_bringup')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    rviz = LaunchConfiguration('rviz')
    start_with_goal = LaunchConfiguration('start_with_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    # --- 1. Simulation at the initial pose ---
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('caging_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={'x': x, 'y': y, 'yaw': yaw, 'rviz': rviz}.items(),
    )

    # --- 2. Teleoperation + twist_mux (needed: it forwards /cmd_vel_nav) ---
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'teleop.launch.py')),
    )

    # --- 3. LQR controller: tuning from the YAML, goal from the arguments ---
    # ParameterValue(..., value_type=...): launch arguments are text; this sends
    # them with the type the node declares (float / bool).
    lqr = Node(
        package='caging_control',
        executable='lqr_controller',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'lqr.yaml'),
            {
                'use_sim_time': True,
                'start_with_goal': ParameterValue(start_with_goal, value_type=bool),
                'goal_x': ParameterValue(goal_x, value_type=float),
                'goal_y': ParameterValue(goal_y, value_type=float),
                'goal_yaw': ParameterValue(goal_yaw, value_type=float),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw [rad]'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('start_with_goal', default_value='false',
                              description='true: go to (goal_x, goal_y, goal_yaw) at start'),
        DeclareLaunchArgument('goal_x', default_value='0.0', description='Goal x [m]'),
        DeclareLaunchArgument('goal_y', default_value='0.0', description='Goal y [m]'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0', description='Goal yaw [rad]'),
        sim,
        teleop,
        lqr,
    ])
