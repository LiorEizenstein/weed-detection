"""
sim.launch.py — Gazebo + RViz simulation of SO-ARM101 scanning for weeds.

Launches (all timing managed here to avoid the inner launch's race conditions):
  1. Gazebo Sim with simple_field.sdf (ground + 4 green weed cylinders)
  2. robot_state_publisher — publishes /robot_description and TF from SO-ARM101 URDF
  3. gz_spawn (5 s delay) — creates robot model inside Gazebo
  4. ros2_control spawners (joint_state_broadcaster, arm_controller, gripper_controller)
  5. camera_bridge (10 s) — bridges Gazebo /camera/image_raw → ROS 2
  6. weed_detection-2.py, sim_arm_controller, RViz (15 s)

Usage:
    ros2 launch real_simulation_ur5 sim.launch.py
    ros2 launch real_simulation_ur5 sim.launch.py model_path:=/home/lior/best.pt
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_WEED_DETECT_PY = os.path.expanduser('~/ros2_ws/weed_detection-2.py')


def generate_launch_description():
    pkg_sim   = get_package_share_directory('real_simulation_ur5')
    pkg_so101 = get_package_share_directory('so_arm_101_description')

    world_file      = os.path.join(pkg_sim,   'worlds', 'simple_field.sdf')
    xacro_file      = os.path.join(pkg_so101, 'urdf',   'so_101.urdf.xacro')
    controllers_yaml = os.path.join(pkg_so101, 'config', 'controllers.yaml')
    params_file     = os.path.join(pkg_sim,   'config', 'sim_params.yaml')
    rviz_config     = os.path.join(pkg_sim,   'config', 'sim.rviz')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/lior/best.pt',
        description='Path to YOLO .pt weights; detection runs in stub mode if missing')
    model_path = LaunchConfiguration('model_path')

    # ── 0. Env vars Gazebo needs (not set by ROS 2 Jazzy on this machine) ────
    # GZ_SIM_SYSTEM_PLUGIN_PATH: allows Gazebo to find libgz_ros2_control-system.so
    # GZ_SIM_RESOURCE_PATH: allows Gazebo to resolve package:// mesh URIs in URDF
    gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/lib',
    )
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/home/lior/ros2_ws/install/so_arm_101_description/share',
    )

    # ── 1. Gazebo Sim with the weed field world ───────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── 2. robot_state_publisher — publishes /robot_description + TF ─────────
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_backend:=gazebo']),
        value_type=str,
    )
    # Republish /joint_states with current wall-time header stamp so that
    # robot_state_publisher can timestamp TF correctly.  Gazebo's
    # joint_state_broadcaster sets header.stamp = sim time (stuck at 0 on
    # WSL2), which makes every TF transform appear 30+ seconds stale in RViz.
    joint_state_restamper = Node(
        package='real_simulation_ur5',
        executable='joint_state_restamper',
        name='joint_state_restamper',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 50.0,
        }],
        # Subscribe to the wall-time-stamped joint states, not the raw ones
        remappings=[('joint_states', 'joint_states_stamped')],
        output='screen',
    )

    # ── 3. Spawn robot in Gazebo (5 s delay so Gazebo has time to start) ─────
    gz_spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'so_101',
                '-z', '0.001',
            ],
            output='screen',
        )
    ])

    # ── 4. Controller spawners (wait for Gazebo's embedded controller_manager) ─
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )

    # ── 5. Camera bridge (10 s — robot must be spawned and sensor active) ────
    camera_bridge = TimerAction(period=10.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen',
        )
    ])

    # ── 6. Detection, arm controller, RViz (15 s for controllers to be ready) ─
    sim_nodes = TimerAction(period=15.0, actions=[
        ExecuteProcess(
            cmd=[sys.executable, _WEED_DETECT_PY,
                 '--topic', '/camera/image_raw',
                 '--model', model_path,
                 '--confidence', '0.7',
                 '--no-display'],
            name='weed_detection',
            output='screen',
        ),
        Node(
            package='real_simulation_ur5',
            executable='sim_arm_controller',
            name='sim_arm_controller',
            # use_sim_time must be False — Gazebo's sim clock stalls at t=0
            # on WSL2, so timers created with use_sim_time:true never fire.
            parameters=[params_file, {'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config],
            output='log',
        ),
    ])

    return LaunchDescription([
        model_path_arg,
        gz_plugin_path,
        gz_resource_path,
        gz_sim,
        joint_state_restamper,
        robot_state_publisher,
        gz_spawn,
        spawner_jsb,
        spawner_arm,
        camera_bridge,
        sim_nodes,
    ])
