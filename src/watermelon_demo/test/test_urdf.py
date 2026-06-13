"""
Tests for Step 3 — UR5 URDF with camera and laser links.
Verifies link structure, sensor config, and joint offsets.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_urdf.py -v
"""

import subprocess
import xml.etree.ElementTree as ET
import math
import pytest
from pathlib import Path

URDF_DIR = Path(__file__).parent.parent / "urdf"
XACRO_FILE = URDF_DIR / "ur5_with_sensors.urdf.xacro"
XACRO_ARGS = ["ur_type:=ur5"]


@pytest.fixture(scope="module")
def urdf_root():
    """Generate URDF from xacro and return the parsed root element."""
    result = subprocess.run(
        ["xacro", str(XACRO_FILE)] + XACRO_ARGS,
        capture_output=True, text=True
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stderr}"
    return ET.fromstring(result.stdout)


def get_link_names(root) -> set[str]:
    return {link.get("name") for link in root.findall("link")}


def get_joint(root, name: str):
    return next((j for j in root.findall("joint") if j.get("name") == name), None)


class TestLinkStructure:

    def test_camera_link_exists(self, urdf_root):
        assert "camera_link" in get_link_names(urdf_root), \
            "camera_link is missing from URDF"

    def test_laser_link_exists(self, urdf_root):
        """laser_link must be in the URDF — the laser is fixed to the arm base."""
        assert "laser_link" in get_link_names(urdf_root), \
            "laser_link is missing from URDF"

    def test_tool0_exists(self, urdf_root):
        assert "tool0" in get_link_names(urdf_root), \
            "tool0 end-effector link missing — upstream xacro may not have been included"

    def test_camera_is_child_of_tool0(self, urdf_root):
        joint = get_joint(urdf_root, "tool0_to_camera")
        assert joint is not None, "Joint 'tool0_to_camera' not found"
        parent = joint.findtext("parent[@link]") or joint.find("parent").get("link")
        child  = joint.findtext("child[@link]")  or joint.find("child").get("link")
        assert parent == "tool0",      f"tool0_to_camera parent should be 'tool0', got '{parent}'"
        assert child  == "camera_link", f"tool0_to_camera child should be 'camera_link', got '{child}'"

    def test_laser_is_child_of_tool0(self, urdf_root):
        """The laser is fixed to tool0 — it moves with the camera and arm."""
        joint = get_joint(urdf_root, "tool0_to_laser")
        assert joint is not None, "Joint 'tool0_to_laser' not found"
        parent = joint.find("parent").get("link")
        child  = joint.find("child").get("link")
        assert parent == "tool0",      f"tool0_to_laser parent should be 'tool0', got '{parent}'"
        assert child  == "laser_link", f"tool0_to_laser child should be 'laser_link', got '{child}'"

    def test_laser_not_child_of_base_link(self, urdf_root):
        """base_to_laser joint must NOT exist — laser is now on tool0, not the static base."""
        joint = get_joint(urdf_root, "base_to_laser")
        assert joint is None, \
            "base_to_laser joint should not exist; laser is fixed to tool0"

    def test_sensor_joints_are_fixed(self, urdf_root):
        for joint_name in ("tool0_to_camera", "tool0_to_laser"):
            joint = get_joint(urdf_root, joint_name)
            assert joint is not None, f"Joint '{joint_name}' not found"
            assert joint.get("type") == "fixed", \
                f"Joint '{joint_name}' should be type='fixed', got '{joint.get('type')}'"


class TestJointOffsets:

    def test_camera_joint_xyz(self, urdf_root):
        joint = get_joint(urdf_root, "tool0_to_camera")
        origin = joint.find("origin")
        assert origin is not None, "tool0_to_camera has no origin element"
        xyz = [float(v) for v in origin.get("xyz").split()]
        assert xyz == pytest.approx([0.02, 0.0, 0.05], abs=0.001), \
            f"Camera joint xyz expected [0.02, 0.0, 0.05], got {xyz}"

    def test_laser_joint_xyz(self, urdf_root):
        joint = get_joint(urdf_root, "tool0_to_laser")
        assert joint is not None, "tool0_to_laser joint not found"
        origin = joint.find("origin")
        assert origin is not None, "tool0_to_laser has no origin element"
        xyz = [float(v) for v in origin.get("xyz").split()]
        assert xyz == pytest.approx([-0.02, 0.0, 0.05], abs=0.001), \
            f"Laser joint xyz expected [-0.02, 0.0, 0.05], got {xyz}"

    def test_camera_joint_rpy_points_down(self, urdf_root):
        """Camera rpy pitch -90° around Y aligns the gz optical axis (+X) with
        tool0 +Z, so the camera looks DOWN at the field (not back at the arm)."""
        joint = get_joint(urdf_root, "tool0_to_camera")
        origin = joint.find("origin")
        rpy = [float(v) for v in origin.get("rpy").split()]
        # rpy[1] (pitch) should be ~-pi/2 (-1.5708 rad)
        assert rpy[1] == pytest.approx(-math.pi / 2, abs=0.01), \
            f"Camera pitch should be ~-1.5708 rad (-pi/2) to point downward, got {rpy[1]:.4f}"


class TestCameraSensor:

    def test_gazebo_sensor_block_present(self, urdf_root):
        gazebo_refs = urdf_root.findall("gazebo")
        camera_gazebo = next(
            (g for g in gazebo_refs if g.get("reference") == "camera_link"), None
        )
        assert camera_gazebo is not None, \
            "No <gazebo reference='camera_link'> block found — camera sensor won't render"

    def test_camera_topic(self, urdf_root):
        gazebo_refs = urdf_root.findall("gazebo")
        camera_gazebo = next(
            (g for g in gazebo_refs if g.get("reference") == "camera_link"), None
        )
        assert camera_gazebo is not None
        topic = camera_gazebo.findtext(".//topic")
        assert topic == "/camera/image_raw", \
            f"Camera topic should be '/camera/image_raw', got '{topic}'"

    def test_camera_resolution(self, urdf_root):
        gazebo_refs = urdf_root.findall("gazebo")
        camera_gazebo = next(
            (g for g in gazebo_refs if g.get("reference") == "camera_link"), None
        )
        assert camera_gazebo is not None
        width  = camera_gazebo.findtext(".//image/width")
        height = camera_gazebo.findtext(".//image/height")
        assert width  == "640", f"Camera width should be 640, got {width}"
        assert height == "480", f"Camera height should be 480, got {height}"

    def test_camera_update_rate(self, urdf_root):
        gazebo_refs = urdf_root.findall("gazebo")
        camera_gazebo = next(
            (g for g in gazebo_refs if g.get("reference") == "camera_link"), None
        )
        assert camera_gazebo is not None
        rate = camera_gazebo.findtext(".//update_rate")
        assert float(rate) == pytest.approx(10.0), \
            f"Camera update_rate should be 10Hz, got {rate}"
