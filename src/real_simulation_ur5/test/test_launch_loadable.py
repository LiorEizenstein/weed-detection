"""
Verifies that sim.launch.py can be loaded and all referenced paths exist.
Does NOT start Gazebo — purely checks installed share files and executables.
"""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory


def test_sim_params_yaml_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'config', 'sim_params.yaml')
    assert os.path.isfile(path), f'Missing: {path}'


def test_sim_rviz_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'config', 'sim.rviz')
    assert os.path.isfile(path), f'Missing: {path}'


def test_world_file_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'worlds', 'simple_field.sdf')
    assert os.path.isfile(path), f'Missing: {path}'


def test_d435_urdf_xacro_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'urdf', 'ur5_with_d435.urdf.xacro')
    assert os.path.isfile(path), f'Missing: {path}'


def test_ur_simulation_gz_launch_exists():
    pkg = get_package_share_directory('ur_simulation_gz')
    path = os.path.join(pkg, 'launch', 'ur_sim_control.launch.py')
    assert os.path.isfile(path), f'Missing: {path}'


def test_sim_arm_controller_executable_registered():
    result = subprocess.run(
        ['ros2', 'pkg', 'executables', 'real_simulation_ur5'],
        capture_output=True, text=True, timeout=10)
    assert 'sim_arm_controller' in result.stdout


def test_sim_detection_node_executable_registered():
    result = subprocess.run(
        ['ros2', 'pkg', 'executables', 'real_simulation_ur5'],
        capture_output=True, text=True, timeout=10)
    assert 'sim_detection_node' in result.stdout


def test_ros_gz_bridge_package_available():
    result = subprocess.run(
        ['ros2', 'pkg', 'list'],
        capture_output=True, text=True, timeout=10)
    assert 'ros_gz_bridge' in result.stdout


def test_sim_launch_file_parseable():
    import ast
    pkg = get_package_share_directory('real_simulation_ur5')
    launch_src = os.path.join(
        os.path.dirname(pkg),  # share/real_simulation_ur5 → share
        '..', '..', '..', 'src', 'real_simulation_ur5', 'launch', 'sim.launch.py')
    # Fall back to installed location
    launch_installed = os.path.join(
        get_package_share_directory('real_simulation_ur5'), '..', 'launch', 'sim.launch.py')
    for candidate in [
        os.path.expanduser('~/ros2_ws/src/real_simulation_ur5/launch/sim.launch.py'),
        os.path.join(get_package_share_directory('real_simulation_ur5'), 'launch', 'sim.launch.py'),
    ]:
        if os.path.isfile(candidate):
            ast.parse(open(candidate).read())
            return
    raise FileNotFoundError('sim.launch.py not found in src or install')
