"""Unified simulator launch file for the SO-ARM-101 robot.

Usage:
    ros2 launch so_arm_101_description sim.launch.py sim:=gazebo
    ros2 launch so_arm_101_description sim.launch.py sim:=mujoco
    ros2 launch so_arm_101_description sim.launch.py sim:=webots
    ros2 launch so_arm_101_description sim.launch.py sim:=coppeliasim
    ros2 launch so_arm_101_description sim.launch.py sim:=isaac
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Webots utilities (only available when webots_ros2_driver is installed)
try:
    from webots_ros2_driver.webots_controller import WebotsController
    from webots_ros2_driver.webots_launcher import WebotsLauncher
    HAS_WEBOTS = True
except ImportError:
    HAS_WEBOTS = False


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('so_arm_101_description')

    # --- Arguments ---
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='gazebo',
        choices=['gazebo', 'mujoco', 'webots', 'coppeliasim', 'isaac'],
        description='Simulator backend to use',
    )
    sim = LaunchConfiguration('sim')

    # --- Robot description ---
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'so_101.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_backend:=', sim]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # --- Conditions ---
    is_gazebo = PythonExpression(["'", sim, "' == 'gazebo'"])
    is_webots = PythonExpression(["'", sim, "' == 'webots'"])
    # ros2_control_node is needed for backends where the simulator doesn't
    # embed its own controller manager (i.e. everything except Gazebo and Webots)
    needs_control_node = PythonExpression(
        ["'", sim, "' not in ('gazebo', 'webots')"]
    )

    # --- Common nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # ros2_control_node — for MuJoCo, CoppeliaSim, Isaac
    # The MuJoCo plugin reads the URDF from the robot_description parameter.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True},
            controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(needs_control_node),
    )

    # --- Controller spawners ---
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

    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )

    # For Gazebo: spawners start after robot_state_publisher
    # (Gazebo plugin embeds the controller manager)
    spawn_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawner_jsb, spawner_arm, spawner_gripper],
        ),
        condition=IfCondition(is_gazebo),
    )

    # For MuJoCo / CoppeliaSim / Isaac: spawners start after ros2_control_node
    spawn_after_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[spawner_jsb, spawner_arm, spawner_gripper],
        ),
        condition=IfCondition(needs_control_node),
    )

    # --- Gazebo ---
    gz_world = PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': ['-r ', gz_world]}.items(),
        condition=IfCondition(is_gazebo),
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'so_101',
            '-z', '0.001',
        ],
        condition=IfCondition(is_gazebo),
    )

    # --- MuJoCo ---
    # mujoco_ros2_control loads the MuJoCo model from the
    # /mujoco_robot_description topic (URDF or MJCF XML string).
    # Publish the URDF so the MuJoCo plugin can parse it.
    is_mujoco = PythonExpression(["'", sim, "' == 'mujoco'"])
    mujoco_model_pub = Node(
        package='so_arm_101_description',
        executable='publish_mujoco_description',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(is_mujoco),
        output='screen',
    )

    # --- Webots ---
    webots_actions = []
    if HAS_WEBOTS:
        webots_world = PathJoinSubstitution(
            [pkg_share, 'worlds', 'webots_sim.wbt']
        )

        webots_launcher = WebotsLauncher(
            world=webots_world,
            mode='realtime',
            condition=IfCondition(is_webots),
        )

        # WebotsController passes params via CLI args, so robot_description
        # must be a file path (not inline URDF). Use the pre-expanded URDF
        # generated at Docker build time (xacro with sim_backend:=webots).
        webots_urdf_path = PathJoinSubstitution(
            [pkg_share, 'urdf', 'so_101_webots.urdf']
        )
        # WebotsController only accepts plain str for --params-file.
        # Resolve the controllers YAML path at import time.
        from ament_index_python.packages import get_package_share_directory
        _pkg = get_package_share_directory('so_arm_101_description')
        webots_controllers_path = os.path.join(_pkg, 'config', 'controllers.yaml')

        webots_robot = WebotsController(
            robot_name='so_101',
            parameters=[
                {
                    'robot_description': webots_urdf_path,
                    'use_sim_time': False,
                    'update_rate': 250,
                },
                webots_controllers_path,
            ],
            condition=IfCondition(is_webots),
        )

        # Spawn controllers after the Webots driver starts
        spawn_after_webots = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=webots_robot,
                on_start=[spawner_jsb, spawner_arm, spawner_gripper],
            ),
            condition=IfCondition(is_webots),
        )

        # Shut down launch when Webots exits
        webots_shutdown = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots_launcher,
                on_exit=[Shutdown()],
            ),
            condition=IfCondition(is_webots),
        )

        webots_actions = [
            webots_launcher,
            webots_robot,
            spawn_after_webots,
            webots_shutdown,
        ]

    # --- CoppeliaSim / Isaac Sim ---
    # These use topic_based_ros2_control. The user starts the simulator
    # externally and loads the URDF via its built-in importer.
    # ros2_control_node + TopicBasedSystem handles the ROS2 side.

    return LaunchDescription([
        sim_arg,
        robot_state_publisher,
        control_node,
        mujoco_model_pub,
        spawn_after_rsp,
        spawn_after_control,
        gz_sim,
        gz_spawn,
        *webots_actions,
    ])
