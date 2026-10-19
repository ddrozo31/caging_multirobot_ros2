"""
Step 10: the whole experiment of the paper with ONE command.

Starts:
  1. Step 9 (trajectory.launch.py): scene, approach, closure, Leader-Follower, trajectory
  2. task_manager       state machine IDLE -> APPROACH -> CLOSURE -> TRANSPORT -> TRAJECTORY -> DONE
  3. safety_supervisor  stops BOTH robots if a pose or a command is lost, or the cage opens

Run:
  ros2 launch caging_bringup task.launch.py                         # then:
  ros2 service call /start_task std_srvs/srv/Trigger
  ros2 launch caging_bringup task.launch.py auto_start:=true        # starts by itself
  ros2 topic echo /task_state

Arguments: object (box), orientation (static | tangent | radial), reference (smooth | waypoints),
           auto_start (false), results_dir (results), rviz, world,
           Step 11: vicon_noise_xy (0.0 m), vicon_noise_yaw (0.0 rad), seed (0).
  ros2 launch caging_bringup task.launch.py auto_start:=true vicon_noise_xy:=0.002 vicon_noise_yaw:=0.005 seed:=3
The noise arguments are declared here and read by the included launch files
(two_robots.launch.py, caging.launch.py): a launch argument is visible to every file it includes.
After the lap: ros2 run caging_control plot_results results/<file>.csv
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
    arguments = ['object', 'orientation', 'reference', 'results_dir', 'rviz', 'world']

    # --- 1. Steps 5-9 ---
    experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'trajectory.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in arguments}.items(),
    )

    # --- 2. State machine ---
    task_manager = Node(
        package='caging_control',
        executable='task_manager',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)}],
    )

    # --- 3. Safety supervisor ---
    supervisor = Node(
        package='caging_control',
        executable='safety_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='box', description='box (tested), hemisphere, pyramid'),
        DeclareLaunchArgument('orientation', default_value='static',
                              description='static (experiment 1), tangent (experiment 2), radial'),
        DeclareLaunchArgument('reference', default_value='smooth', description='smooth or waypoints (paper)'),
        DeclareLaunchArgument('auto_start', default_value='false', description='start the task by itself'),
        DeclareLaunchArgument('results_dir', default_value='results', description='folder for the CSV files'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('vicon_noise_xy', default_value='0.0',
                              description='Step 11: Vicon noise, std of x and y [m] (0 = perfect poses)'),
        DeclareLaunchArgument('vicon_noise_yaw', default_value='0.0',
                              description='Step 11: Vicon noise, std of theta [rad]'),
        DeclareLaunchArgument('seed', default_value='0', description='Step 11: random seed of the noise'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('caging_gazebo'), 'worlds', 'empty_world.sdf'),
            description='Full path to the world (SDF) file'),
        experiment,
        task_manager,
        supervisor,
    ])
