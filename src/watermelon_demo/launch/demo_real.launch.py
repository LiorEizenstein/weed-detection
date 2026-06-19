"""
demo_real.launch.py — launch watermelon weed-detection on physical UR5 + RealSense D435.

Usage:
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=true
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=false
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 use_real_model:=false
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 \
        model_path:=/path/to/best.pt

Before running:
    1. Run easy_handeye2 calibration and paste results into config/camera_params.yaml,
       OR fill in x/y/z + roll/pitch/yaw from manual measurement (less accurate).
    2. Ensure ur_robot_driver, realsense2_camera, and image_view are installed.
    3. Place best.pt at /home/lior/best.pt, or pass use_real_model:=false to fall
       back to the HSV colour stub.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Robot IP ────────────────────────────────────────────────────────────────
DEFAULT_ROBOT_IP = '192.168.1.100'

# ── Camera mount geometry ────────────────────────────────────────────────────
# Loaded from config/camera_params.yaml at import time so that module-level
# constants are available to tests and the static TF args are defined once.
# Edit that file (not this one) before running on the physical robot.
_cam_cfg_path = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'camera_params.yaml')
try:
    with open(_cam_cfg_path) as _f:
        _cam_cfg = yaml.safe_load(_f)
    _cam = _cam_cfg['mount']
except (FileNotFoundError, KeyError) as _exc:
    raise RuntimeError(
        f"Cannot read camera_params.yaml ({_exc}). "
        "Run 'python3 scripts/check_camera_params.py' to validate it."
    ) from _exc

CAMERA_X     = float(_cam['x'])
CAMERA_Y     = float(_cam['y'])
CAMERA_Z     = float(_cam['z'])
CAMERA_ROLL  = float(_cam['roll'])
CAMERA_PITCH = float(_cam['pitch'])
CAMERA_YAW   = float(_cam['yaw'])

# When calibrated: true, use quaternion (more accurate, from easy_handeye2).
# When calibrated: false, fall back to roll/pitch/yaw from manual measurement.
_calibrated = bool(_cam_cfg.get('calibrated', False))
if _calibrated:
    _rotation_args = [
        '--qx', str(float(_cam['qx'])),
        '--qy', str(float(_cam['qy'])),
        '--qz', str(float(_cam['qz'])),
        '--qw', str(float(_cam['qw'])),
    ]
else:
    _rotation_args = [
        '--roll',  str(CAMERA_ROLL),
        '--pitch', str(CAMERA_PITCH),
        '--yaw',   str(CAMERA_YAW),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')

    params_file = os.path.join(pkg, 'config', 'real_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'demo_rviz.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value=DEFAULT_ROBOT_IP,
        description='IP address of the UR5 controller')

    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='true = detection-only (no laser signal); false = full pipeline')

    use_real_model_arg = DeclareLaunchArgument(
        'use_real_model', default_value='true',
        description='true = YOLO model; false = HSV colour stub (no .pt file needed)')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/lior/best.pt',
        description='Path to the YOLO .pt weights file (only used when use_real_model=true)')

    robot_ip       = LaunchConfiguration('robot_ip')
    dry_run        = LaunchConfiguration('dry_run')
    use_real_model = LaunchConfiguration('use_real_model')
    model_path     = LaunchConfiguration('model_path')

    # ── 1. UR5 driver (includes robot_state_publisher + controller spawners) ──
    # ur_control.launch.py handles: hardware interface, robot_state_publisher,
    # joint_state_broadcaster, and scaled_joint_trajectory_controller spawners.
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ur_robot_driver'),
                'launch', 'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type':     'ur5',
            'robot_ip':    robot_ip,
            'launch_rviz': 'false',
        }.items(),
    )

    # ── 2. RealSense D435 camera ─────────────────────────────────────────────
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

    # ── 3. Static TF: tool0 → camera_link ───────────────────────────────────
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '--x', str(CAMERA_X),
            '--y', str(CAMERA_Y),
            '--z', str(CAMERA_Z),
            *_rotation_args,
            '--frame-id',       'tool0',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 4. Static TF: tool0 → laser_link ────────────────────────────────────
    # Laser fires through the camera aperture — treat as coincident with camera.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=[
            '--x', str(CAMERA_X),
            '--y', str(CAMERA_Y),
            '--z', str(CAMERA_Z),
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id',       'tool0',
            '--child-frame-id', 'laser_link',
        ],
    )

    # ── 5. Demo nodes (delayed 15 s — UR driver + controllers need ~10 s) ────
    demo_nodes = TimerAction(period=15.0, actions=[
        Node(
            package='watermelon_demo',
            executable='detection_node',
            name='detection_node',
            parameters=[params_file, {
                'use_real_model': use_real_model,
                'model_path':     model_path,
            }],
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
        use_real_model_arg,
        model_path_arg,
        ur_driver,
        realsense,
        camera_tf,
        laser_tf,
        demo_nodes,
    ])
