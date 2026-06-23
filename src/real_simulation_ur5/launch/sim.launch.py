"""
sim.launch.py — Gazebo + RViz simulation of UR5 scanning for weeds.

Launches:
  1. ur_simulation_gz  — Gazebo Sim 8 with UR5, ros2_control, joint controllers
  2. camera_bridge     — Gazebo camera topic → ROS 2 Image / CameraInfo
  3. sim_detection_node — YOLO (or stub) weed detector
  4. sim_arm_controller — simplified scan → detect → continue state machine
  5. RViz2             — robot model + weed markers + camera feed

Usage:
    ros2 launch real_simulation_ur5 sim.launch.py
    ros2 launch real_simulation_ur5 sim.launch.py model_path:=/home/lior/best.pt
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim   = get_package_share_directory('real_simulation_ur5')

    world_file       = os.path.join(pkg_sim, 'worlds', 'simple_field.sdf')
    description_file = os.path.join(pkg_sim, 'urdf',   'ur5_with_d435.urdf.xacro')
    params_file      = os.path.join(pkg_sim,  'config',  'sim_params.yaml')
    rviz_config      = os.path.join(pkg_sim,  'config',  'sim.rviz')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/lior/best.pt',
        description='Path to YOLO .pt weights; leave default to use stub mode')

    model_path = LaunchConfiguration('model_path')

    # ── 1. Gazebo simulation + ros2_control + joint controllers ──────────────
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gz'), '/launch/ur_sim_control.launch.py'
        ]),
        launch_arguments={
            'ur_type':          'ur5',
            'world_file':       world_file,
            'description_file': description_file,
            'launch_rviz':      'false',
        }.items(),
    )

    # ── 2. Camera bridge: Gazebo topic → ROS 2 ───────────────────────────────
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # ── 3. Sim nodes + RViz (delayed 10 s for Gz + controllers to be ready) ──
    sim_nodes = TimerAction(period=10.0, actions=[
        Node(
            package='real_simulation_ur5',
            executable='sim_detection_node',
            name='sim_detection_node',
            parameters=[params_file, {'model_path': model_path}],
            output='screen',
        ),
        Node(
            package='real_simulation_ur5',
            executable='sim_arm_controller',
            name='sim_arm_controller',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ),
    ])

    return LaunchDescription([
        model_path_arg,
        ur_sim,
        camera_bridge,
        sim_nodes,
    ])
