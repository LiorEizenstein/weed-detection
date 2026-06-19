"""
demo_real.launch.py — launch watermelon weed-detection on physical UR5 + RealSense D435.

Usage:
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=true
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=false

Before running:
    1. Measure the camera mount offset from tool0 and fill in the CAMERA_* constants below.
    2. Ensure ur_robot_driver, realsense2_camera, and image_view are installed.
    3. Set use_real_model: true and provide best.pt at /home/lior/best.pt.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Robot IP ────────────────────────────────────────────────────────────────
DEFAULT_ROBOT_IP = '192.168.1.100'

# ── Camera mount geometry — loaded from config/camera_params.yaml ────────────
# Edit that file (not this one) before running on the physical robot.
_cam_cfg_path = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'camera_params.yaml')
with open(_cam_cfg_path) as _f:
    _cam = yaml.safe_load(_f)['mount']

CAMERA_X     = float(_cam['x'])
CAMERA_Y     = float(_cam['y'])
CAMERA_Z     = float(_cam['z'])
CAMERA_ROLL  = float(_cam['roll'])
CAMERA_PITCH = float(_cam['pitch'])
CAMERA_YAW   = float(_cam['yaw'])


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')

    params_file = os.path.join(pkg, 'config', 'real_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'demo_rviz.rviz')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value=DEFAULT_ROBOT_IP,
        description='IP address of the UR5 controller')

    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='true = detection-only (no laser signal); false = full pipeline')

    robot_ip  = LaunchConfiguration('robot_ip')
    dry_run   = LaunchConfiguration('dry_run')

    # ── 1. UR5 driver ────────────────────────────────────────────────────────
    ur_driver = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        name='ur_ros2_control_node',
        parameters=[{
            'robot_ip': robot_ip,
            'ur_type': 'ur5',
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── 2. Joint state broadcaster + arm controller (ros2_control) ───────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scaled_joint_trajectory_controller'],
        output='screen',
    )

    # ── 3. RealSense D435 camera ─────────────────────────────────────────────
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30.0,
        }],
        output='screen',
    )

    # ── 4. Static TF: tool0 → camera_link ───────────────────────────────────
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '--x',     str(CAMERA_X),
            '--y',     str(CAMERA_Y),
            '--z',     str(CAMERA_Z),
            '--roll',  str(CAMERA_ROLL),
            '--pitch', str(CAMERA_PITCH),
            '--yaw',   str(CAMERA_YAW),
            '--frame-id',       'tool0',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 5. Demo nodes (delayed 5 s for driver + camera to be ready) ──────────
    demo_nodes = TimerAction(period=5.0, actions=[
        Node(
            package='watermelon_demo',
            executable='detection_node',
            name='detection_node',
            parameters=[params_file],
            remappings=[('/camera/image_raw', '/camera/color/image_raw')],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='arm_controller_node',
            name='arm_controller_node',
            parameters=[params_file, {'dry_run': dry_run}],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='laser_effect_node',
            name='laser_effect_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='field_manager_node',
            name='field_manager_node',
            parameters=[params_file],
            output='screen',
        ),
        # Live annotated camera feed for audience
        Node(
            package='image_view',
            executable='image_view',
            name='detection_display',
            remappings=[('image', '/detection_image')],
            output='log',
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
        robot_ip_arg,
        dry_run_arg,
        ur_driver,
        joint_state_broadcaster,
        trajectory_controller,
        realsense,
        camera_tf,
        demo_nodes,
    ])
