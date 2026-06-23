"""
demo_camera_only.launch.py — real RealSense camera + YOLO detection, shown in RViz.

No robot arm needed. Plug in the RealSense D435 via USB and run:

    ros2 launch watermelon_demo demo_camera_only.launch.py
    ros2 launch watermelon_demo demo_camera_only.launch.py use_real_model:=false  # HSV stub

RViz opens two image panels:
  - Raw Camera  → /camera/camera/color/image_raw  (what the sensor sees)
  - Detection Results → /detection_image          (YOLO annotations overlaid)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')
    params_file = os.path.join(pkg, 'config', 'real_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'camera_only.rviz')

    use_real_model_arg = DeclareLaunchArgument(
        'use_real_model', default_value='true',
        description='true = YOLO model; false = HSV colour stub')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/lior/best.pt',
        description='Path to YOLO .pt weights file')

    use_real_model = LaunchConfiguration('use_real_model')
    model_path = LaunchConfiguration('model_path')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'initial_reset': 'false',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_color': 'true',
            'rgb_camera.color_profile': '640x480x30',
        }.items(),
    )

    detection = Node(
        package='watermelon_demo',
        executable='detection_node',
        name='detection_node',
        parameters=[params_file, {
            'use_real_model': use_real_model,
            'model_path': model_path,
        }],
        remappings=[('/camera/image_raw', '/camera/camera/color/image_raw')],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
    )

    return LaunchDescription([
        use_real_model_arg,
        model_path_arg,
        realsense,
        detection,
        rviz,
    ])
