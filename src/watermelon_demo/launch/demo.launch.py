import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')

    world_file      = os.path.join(pkg, 'worlds', 'watermelon_field.sdf')
    description_file = os.path.join(pkg, 'urdf', 'ur5_with_sensors.urdf.xacro')
    params_file     = os.path.join(pkg, 'config', 'demo_params.yaml')
    rviz_config     = os.path.join(pkg, 'config', 'demo_rviz.rviz')

    # ── 1. UR5 Gazebo simulation + ros2_control ──────────────────────────────
    # /clock bridge is already inside ur_sim_control.launch.py — do not add it
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

    # ── 2. Camera bridge: Gazebo publishes → ROS2 subscribes ─────────────────
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

    # ── 3. Demo nodes + RViz (delayed 8 s for Gz + controllers to be ready) ──
    # laser_link TF is published by robot_state_publisher via the URDF — no
    # static_transform_publisher needed.
    demo_nodes = TimerAction(period=8.0, actions=[
        Node(
            package='watermelon_demo', executable='detection_node',
            name='detection_node', parameters=[params_file], output='screen',
        ),
        Node(
            package='watermelon_demo', executable='arm_controller_node',
            name='arm_controller_node', parameters=[params_file], output='screen',
        ),
        Node(
            package='watermelon_demo', executable='laser_effect_node',
            name='laser_effect_node', parameters=[params_file], output='screen',
        ),
        Node(
            package='watermelon_demo', executable='field_manager_node',
            name='field_manager_node', parameters=[params_file], output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2', arguments=['-d', rviz_config], output='log',
        ),
    ])

    return LaunchDescription([
        ur_sim,
        camera_bridge,
        demo_nodes,
    ])
