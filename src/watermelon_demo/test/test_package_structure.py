"""
Tests for Step 1 — ROS2 package structure.
Verifies package files exist and are correctly configured.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_package_structure.py -v
"""

import xml.etree.ElementTree as ET
import ast
import pytest
from pathlib import Path

PACKAGE_DIR = Path(__file__).parent.parent

REQUIRED_DEPS = [
    "rclpy",
    "sensor_msgs",
    "geometry_msgs",
    "visualization_msgs",
    "std_msgs",
    "control_msgs",
    "trajectory_msgs",
    "vision_msgs",
    "cv_bridge",
    "tf2_ros",
    "tf2_geometry_msgs",
    "ros_gz_bridge",
    "ur_simulation_gz",
    "ur_moveit_config",
]

REQUIRED_ENTRY_POINTS = [
    "detection_node",
    "arm_controller_node",
    "laser_effect_node",
    "field_manager_node",
]

REQUIRED_DIRECTORIES = [
    "watermelon_demo",
    "worlds",
    "urdf",
    "config",
    "launch",
    "resource",
    "test",
]


class TestPackageFiles:

    def test_package_xml_exists(self):
        assert (PACKAGE_DIR / "package.xml").exists(), "package.xml missing"

    def test_setup_py_exists(self):
        assert (PACKAGE_DIR / "setup.py").exists(), "setup.py missing"

    def test_setup_cfg_exists(self):
        assert (PACKAGE_DIR / "setup.cfg").exists(), "setup.cfg missing"

    def test_resource_marker_exists(self):
        marker = PACKAGE_DIR / "resource" / "watermelon_demo"
        assert marker.exists(), f"ament resource marker missing at {marker}"

    def test_python_package_init_exists(self):
        init = PACKAGE_DIR / "watermelon_demo" / "__init__.py"
        assert init.exists(), f"__init__.py missing at {init}"

    def test_required_directories_exist(self):
        missing = [
            d for d in REQUIRED_DIRECTORIES
            if not (PACKAGE_DIR / d).is_dir()
        ]
        assert not missing, f"Missing directories: {missing}"


class TestPackageXml:

    @pytest.fixture(scope="class")
    def xml_root(self):
        tree = ET.parse(PACKAGE_DIR / "package.xml")
        return tree.getroot()

    def test_package_name(self, xml_root):
        name = xml_root.findtext("name")
        assert name == "watermelon_demo", f"Wrong package name: {name!r}"

    def test_build_type_is_ament_python(self, xml_root):
        build_type = xml_root.findtext(".//build_type")
        assert build_type == "ament_python", f"Expected ament_python, got: {build_type!r}"

    def test_required_dependencies(self, xml_root):
        declared = {el.text for el in xml_root.findall("exec_depend")}
        missing = [dep for dep in REQUIRED_DEPS if dep not in declared]
        assert not missing, (
            f"Missing exec_depend entries in package.xml: {missing}"
        )


class TestSetupPy:

    @pytest.fixture(scope="class")
    def setup_content(self):
        return (PACKAGE_DIR / "setup.py").read_text()

    def test_entry_points_defined(self, setup_content):
        missing = [ep for ep in REQUIRED_ENTRY_POINTS if ep not in setup_content]
        assert not missing, (
            f"Missing entry points in setup.py: {missing}\n"
            f"Each node needs a console_scripts entry so 'ros2 run' can find it."
        )

    def test_all_four_nodes_present(self, setup_content):
        count = sum(1 for ep in REQUIRED_ENTRY_POINTS if ep in setup_content)
        assert count == 4, f"Expected 4 entry points, found {count}"
