"""
demo_ur5_camera.launch.py — minimal UR5 + RealSense sanity-check launch.

No YOLO, no detection, no arm controller — just verify the robot driver
connects and the camera stream is live before running the full pipeline.

Usage:
    ros2 launch watermelon_demo demo_ur5_camera.launch.py robot_ip:=192.168.1.113
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

DEFAULT_ROBOT_IP = '192.168.1.113'

# Load camera mount geometry (same source as demo_real.launch.py)
_cam_cfg_path = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'camera_params.yaml')
try:
    with open(_cam_cfg_path) as _f:
        _cam_cfg = yaml.safe_load(_f)
    _cam = _cam_cfg['mount']
except (FileNotFoundError, KeyError) as _exc:
    raise RuntimeError(
        f"Cannot read camera_params.yaml ({_exc}). "
        "Fill in config/camera_params.yaml before launching."
    ) from _exc

CAMERA_X     = float(_cam['x'])
CAMERA_Y     = float(_cam['y'])
CAMERA_Z     = float(_cam['z'])

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
        '--roll',  str(float(_cam['roll'])),
        '--pitch', str(float(_cam['pitch'])),
        '--yaw',   str(float(_cam['yaw'])),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')
    rviz_config = os.path.join(pkg, 'config', 'ur5_camera_basic.rviz')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value=DEFAULT_ROBOT_IP,
        description='IP address of the UR5 controller')

    robot_ip = LaunchConfiguration('robot_ip')

    # 1. UR5 driver — brings up robot_state_publisher, joint_state_broadcaster,
    #    and scaled_joint_trajectory_controller
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch', 'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type':                'ur5',
            'robot_ip':               robot_ip,
            'launch_rviz':            'false',
            'kinematics_params_file': os.path.expanduser('~/ur5_calibration.yaml'),
        }.items(),
    )

    # 2. RealSense D435 — colour stream only
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'initial_reset':              'false',
            'enable_color':               'true',
            'enable_depth':               'false',
            'enable_infra1':              'false',
            'enable_infra2':              'false',
            'rgb_camera.color_profile':   '640x480x30',
        }.items(),
    )

    # 3. Static TF: tool0 → camera_link
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

    # 4. RViz — robot model + raw camera image
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_driver,
        realsense,
        camera_tf,
        rviz,
    ])
