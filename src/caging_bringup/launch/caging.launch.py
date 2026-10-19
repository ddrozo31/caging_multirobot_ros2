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

# Grasp of each object (Step 10). The closing distance d* makes the object just touch
# both V grippers (120 deg V, apex at l_g = 0.12 m), see docs/steps/step_10_task_and_results.md:
#   box:        corner in the apex, 0.141 m from the center   -> d* = 2 (0.141 + 0.12) = 0.523 (Table 1: 0.52)
#   hemisphere: sphere tangent to both fingers, 0.1/sin(60)   -> d* = 0.471
#   pyramid:    follower on a corner, leader on a face        -> d* = 0.413
# Only the box has been tested so far.
OBJECTS = {
    'box':        {'grasp_offset': 0.7854, 'closing_distance': 0.52},
    'hemisphere': {'grasp_offset': 0.0,    'closing_distance': 0.471},
    'pyramid':    {'grasp_offset': 0.0,    'closing_distance': 0.413},
}


def generate_launch_description():

    bringup_share = get_package_share_directory('caging_bringup')

    # --- 1. Step 5 scene, robots WITH grippers, the object (box by default) ---
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'two_robots.launch.py')),
        launch_arguments={
            'object': LaunchConfiguration('object'),
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
        # grasp of this object (overrides caging.yaml)
        params.append(OBJECTS[LaunchConfiguration('object').perform(context)])
        # Step 11: same rule as two_robots.launch.py: with Vicon noise, read the vicon/odom poses
        noisy = (float(LaunchConfiguration('vicon_noise_xy', default='0.0').perform(context)) > 0 or
                 float(LaunchConfiguration('vicon_noise_yaw', default='0.0').perform(context)) > 0)
        pose_source = {'pose_source': 'vicon' if noisy else 'ground_truth'}
        nodes = []
        if LaunchConfiguration('leader_follower').perform(context) == 'true':
            params.append(os.path.join(bringup_share, 'config', 'transport.yaml'))
            # --- 3. Leader-Follower controller of the follower (starts inactive) ---
            nodes.append(Node(
                package='caging_control',
                executable='leader_follower',
                output='screen',
                parameters=[os.path.join(bringup_share, 'config', 'leader_follower.yaml'),
                            {'use_sim_time': True}, pose_source],
            ))
        nodes.append(Node(
            package='caging_control',
            executable='approach_manager',
            output='screen',
            parameters=params + [{'use_sim_time': True}, pose_source],
        ))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='box',
                              description='box (tested), hemisphere, pyramid'),
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
