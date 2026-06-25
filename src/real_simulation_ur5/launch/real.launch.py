"""
real.launch.py — SO-ARM101 real hardware launch.

What's different from sim.launch.py:
  - No Gazebo / ros_gz_bridge (RViz shows the real arm via TF instead)
  - sim_backend:=hardware in the URDF (removes the Gazebo plugin)
  - USB camera driver instead of the Gazebo camera bridge
  - dry_run arg: arm moves and detects but takes no action on weeds
  - use_fake_joints arg (default true): publishes zero joint states so RViz
    shows the robot model before the Feetech hardware driver is ready.
    Set false once the hardware driver provides real /joint_states.

TODO — blocked on Feetech hardware driver:
  When the driver is ready, add here:
    - ros2_control controller_manager node (loaded with so_101 hardware plugin)
    - spawner for joint_state_broadcaster  (replaces fake_joint_publisher)
    - spawner for arm_controller           (provides the action server)
    - spawner for gripper_controller
  Then launch with: use_fake_joints:=false

Usage:
    ros2 launch real_simulation_ur5 real.launch.py
    ros2 launch real_simulation_ur5 real.launch.py camera_device:=/dev/video2
    ros2 launch real_simulation_ur5 real.launch.py dry_run:=true
    ros2 launch real_simulation_ur5 real.launch.py model_path:=/home/lior/best.pt
    ros2 launch real_simulation_ur5 real.launch.py use_fake_joints:=false   # with hardware driver
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_WEED_DETECT_PY = os.path.expanduser('~/ros2_ws/weed_detection-2.py')


def generate_launch_description():
    pkg_sim   = get_package_share_directory('real_simulation_ur5')
    pkg_so101 = get_package_share_directory('so_arm_101_description')

    xacro_file  = os.path.join(pkg_so101, 'urdf',   'so_101.urdf.xacro')
    params_file = os.path.join(pkg_sim,   'config', 'sim_params.yaml')
    rviz_config = os.path.join(pkg_sim,   'config', 'sim.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/lior/best.pt',
        description='Path to YOLO .pt weights; stub mode if file is missing')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='USB camera device (find yours: ls /dev/video* or v4l2-ctl --list-devices)')

    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='If true, arm moves and detects but takes no action on weeds')

    use_fake_joints_arg = DeclareLaunchArgument(
        'use_fake_joints', default_value='true',
        description='Publish zero joint states for RViz (disable when hardware driver runs)')

    model_path      = LaunchConfiguration('model_path')
    camera_device   = LaunchConfiguration('camera_device')
    dry_run         = LaunchConfiguration('dry_run')
    use_fake_joints = LaunchConfiguration('use_fake_joints')

    # ── 1. Robot description + TF ─────────────────────────────────────────────
    # sim_backend:=hardware strips the Gazebo plugin so the URDF loads without
    # Gazebo.  The hardware plugin entry in the URDF is currently a placeholder
    # (ros2_control_demo_hardware) — replace with the real Feetech plugin once
    # the driver is ready.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_backend:=hardware']),
        value_type=str,
    )

    # Re-stamp /joint_states with wall time so RSP timestamps TF correctly.
    # Needed even on real hardware in case the driver has the same t=0 issue.
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
        remappings=[('joint_states', 'joint_states_stamped')],
        output='screen',
    )

    # ── 2. Fake joint states (remove when hardware driver is ready) ───────────
    # Publishes all joints at zero so the robot model is visible in RViz
    # before the real arm is connected.
    fake_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        condition=IfCondition(use_fake_joints),
        output='screen',
    )

    # ── 3. USB / UVC camera ───────────────────────────────────────────────────
    # Install: sudo apt install ros-jazzy-usb-cam
    # Check device: ls /dev/video*   or   v4l2-ctl --list-devices
    usb_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device':  camera_device,
            'image_width':   640,
            'image_height':  480,
            'framerate':     10.0,
            'pixel_format':  'yuyv',
            'camera_name':   'camera',
            'frame_id':      'camera_color_optical_frame',
        }],
        remappings=[
            ('image_raw',   '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    # ── 4. Weed detection ─────────────────────────────────────────────────────
    # Identical to simulation — subscribes to /camera/image_raw, no changes needed.
    weed_detection = ExecuteProcess(
        cmd=[sys.executable, _WEED_DETECT_PY,
             '--topic', '/camera/image_raw',
             '--model', model_path,
             '--confidence', '0.7'],
        name='weed_detection',
        output='screen',
    )

    # ── 5. Arm controller ─────────────────────────────────────────────────────
    # Identical to simulation — talks only to /arm_controller/follow_joint_trajectory.
    # Will log "action server not found after 30s" until the hardware driver runs.
    sim_arm_controller = Node(
        package='real_simulation_ur5',
        executable='sim_arm_controller',
        name='sim_arm_controller',
        parameters=[params_file, {
            'use_sim_time': False,
            'dry_run':      dry_run,
        }],
        output='screen',
    )

    # ── 6. RViz ───────────────────────────────────────────────────────────────
    # Same config as simulation — shows robot model, TF, weed markers, camera feed.
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config],
        output='log',
    )

    return LaunchDescription([
        model_path_arg,
        camera_device_arg,
        dry_run_arg,
        use_fake_joints_arg,
        joint_state_restamper,
        robot_state_publisher,
        fake_joint_publisher,
        usb_camera,
        weed_detection,
        sim_arm_controller,
        rviz2,
    ])
