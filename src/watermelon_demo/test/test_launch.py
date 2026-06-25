"""
Tests for Step 6 — launch file.
Checks syntax, file references, and bridge arguments without actually launching.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_launch.py -v
"""

import ast
import pytest
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

LAUNCH_DIR = Path(__file__).parent.parent / "launch"
LAUNCH_FILE = LAUNCH_DIR / "demo.launch.py"
PKG_SHARE = Path(get_package_share_directory('watermelon_demo'))


class TestLaunchFile:

    def test_launch_file_exists(self):
        assert LAUNCH_FILE.exists(), f"demo.launch.py not found at {LAUNCH_FILE}"

    def test_launch_file_valid_python(self):
        try:
            ast.parse(LAUNCH_FILE.read_text())
        except SyntaxError as e:
            pytest.fail(f"demo.launch.py has a Python syntax error: {e}")

    def test_generate_launch_description_defined(self):
        content = LAUNCH_FILE.read_text()
        assert 'def generate_launch_description' in content, \
            "demo.launch.py must define generate_launch_description()"

    def test_no_clock_bridge_duplicated(self):
        content = LAUNCH_FILE.read_text()
        assert content.count('rosgraph_msgs/msg/Clock') == 0, \
            "/clock bridge is already inside ur_sim_control.launch.py — do not add it again"

    def test_camera_bridge_topics_present(self):
        content = LAUNCH_FILE.read_text()
        assert '/camera/image_raw' in content, \
            "Launch file must bridge /camera/image_raw"
        assert '/camera/camera_info' in content, \
            "Launch file must bridge /camera/camera_info"

    def test_camera_bridge_direction_gz_to_ros(self):
        content = LAUNCH_FILE.read_text()
        assert 'gz.msgs.Image' in content, \
            "Camera bridge must use gz.msgs.Image (Gz→ROS direction)"

    def test_laser_tf_via_urdf(self):
        """Laser is on tool0 in the URDF — robot_state_publisher handles the TF, no static publisher needed."""
        content = LAUNCH_FILE.read_text()
        assert 'laser_mount_frame' not in content, \
            "laser_mount_frame static TF should be removed — laser_link TF comes from the URDF"
        assert 'description_file' in content, \
            "URDF description_file must be passed so robot_state_publisher publishes laser_link TF"

    def test_ur_sim_launch_included(self):
        content = LAUNCH_FILE.read_text()
        assert 'ur_sim_control.launch.py' in content, \
            "Launch file must include ur_sim_control.launch.py"

    def test_ur_type_is_ur5(self):
        content = LAUNCH_FILE.read_text()
        assert "'ur5'" in content or '"ur5"' in content, \
            "ur_type must be set to 'ur5'"

    def test_launch_rviz_false(self):
        content = LAUNCH_FILE.read_text()
        assert "'false'" in content or '"false"' in content, \
            "launch_rviz must be 'false' (we launch our own RViz)"

    def test_all_four_nodes_launched(self):
        content = LAUNCH_FILE.read_text()
        for node in ('detection_node', 'arm_controller_node',
                     'laser_effect_node', 'field_manager_node'):
            assert node in content, f"{node} not found in launch file"

    def test_rviz_launched_with_config(self):
        content = LAUNCH_FILE.read_text()
        assert 'rviz2' in content, "RViz2 not launched"
        assert 'demo_rviz.rviz' in content, "RViz not using demo_rviz.rviz config"

    def test_demo_nodes_use_params_file(self):
        content = LAUNCH_FILE.read_text()
        assert 'demo_params.yaml' in content, \
            "Demo nodes must load demo_params.yaml"

    def test_timer_delay_present(self):
        content = LAUNCH_FILE.read_text()
        assert 'TimerAction' in content, \
            "Demo nodes must be delayed with TimerAction to wait for Gz+controllers"


class TestInstalledPaths:

    @classmethod
    def setup_class(cls):
        import sys
        # test_camera_tf mocks ament_index_python at module level; remove the
        # mock so we get the real get_package_share_directory.
        for mod_name in ['ament_index_python', 'ament_index_python.packages']:
            if mod_name in sys.modules:
                del sys.modules[mod_name]
        from ament_index_python.packages import get_package_share_directory
        cls._pkg_share = Path(get_package_share_directory('watermelon_demo'))

    def test_world_file_installed(self):
        path = self._pkg_share / 'worlds' / 'watermelon_field.sdf'
        assert path.exists(), f"Installed world file not found at {path}"

    def test_urdf_file_installed(self):
        path = self._pkg_share / 'urdf' / 'ur5_with_sensors.urdf.xacro'
        assert path.exists(), f"Installed URDF not found at {path}"

    def test_params_file_installed(self):
        path = self._pkg_share / 'config' / 'demo_params.yaml'
        assert path.exists(), f"Installed params not found at {path}"

    def test_rviz_config_installed(self):
        path = self._pkg_share / 'config' / 'demo_rviz.rviz'
        assert path.exists(), f"Installed RViz config not found at {path}"

    def test_launch_file_installed(self):
        path = self._pkg_share / 'launch' / 'demo.launch.py'
        assert path.exists(), f"Installed launch file not found at {path}"
