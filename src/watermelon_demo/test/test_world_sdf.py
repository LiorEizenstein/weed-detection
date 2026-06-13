"""
Tests for Step 2 — Gazebo world SDF.
Verifies plant models, plugins, and reachability constraints.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_world_sdf.py -v
"""

import math
import xml.etree.ElementTree as ET
import pytest
from pathlib import Path

WORLDS_DIR = Path(__file__).parent.parent / "worlds"
SDF_FILE = WORLDS_DIR / "watermelon_field.sdf"

UR5_MAX_REACH = 0.85  # metres

EXPECTED_WATERMELONS = {
    "watermelon_1": (0.5,  0.2,  0.08),
    "watermelon_2": (0.5, -0.2,  0.08),
    "watermelon_3": (0.6,  0.0,  0.08),
    "watermelon_4": (0.4,  0.4,  0.08),
}

EXPECTED_WEEDS = {
    "weed_1": (0.55,  0.35, 0.03),
    "weed_2": (0.45, -0.35, 0.03),
    "weed_3": (0.65, -0.1,  0.03),
}

REQUIRED_PLUGINS = [
    "gz::sim::systems::Physics",
    "gz::sim::systems::SceneBroadcaster",
    "gz::sim::systems::Sensors",   # required for camera rendering
    "gz::sim::systems::Contact",
]


@pytest.fixture(scope="module")
def sdf_root():
    assert SDF_FILE.exists(), f"World file not found: {SDF_FILE}"
    tree = ET.parse(SDF_FILE)
    return tree.getroot().find("world")


def parse_pose(pose_text: str) -> tuple[float, float, float]:
    """Parse 'x y z roll pitch yaw' pose string, return (x, y, z)."""
    parts = [float(v) for v in pose_text.strip().split()]
    return parts[0], parts[1], parts[2]


def xy_distance(x, y) -> float:
    return math.sqrt(x ** 2 + y ** 2)


class TestSDFFile:

    def test_sdf_file_exists(self):
        assert SDF_FILE.exists(), f"World SDF not found at {SDF_FILE}"

    def test_sdf_is_valid_xml(self):
        try:
            ET.parse(SDF_FILE)
        except ET.ParseError as e:
            pytest.fail(f"SDF is not valid XML: {e}")

    def test_sdf_version(self):
        root = ET.parse(SDF_FILE).getroot()
        version = root.get("version")
        assert version is not None, "SDF has no version attribute"


class TestPlugins:

    def test_required_plugins_present(self, sdf_root):
        plugin_names = {p.get("name") for p in sdf_root.findall("plugin")}
        missing = [p for p in REQUIRED_PLUGINS if p not in plugin_names]
        assert not missing, (
            f"Missing plugins: {missing}\n"
            f"Without gz::sim::systems::Sensors the camera will silently publish nothing."
        )

    def test_sensors_plugin_uses_ogre2(self, sdf_root):
        sensors_plugin = next(
            (p for p in sdf_root.findall("plugin")
             if p.get("name") == "gz::sim::systems::Sensors"),
            None
        )
        assert sensors_plugin is not None, "Sensors plugin not found"
        engine = sensors_plugin.findtext("render_engine")
        assert engine == "ogre2", (
            f"render_engine should be 'ogre2', got: {engine!r}. "
            f"ogre2 is required for gz-sim 8 camera rendering."
        )


class TestWatermelonModels:

    def test_all_watermelons_present(self, sdf_root):
        model_names = {m.get("name") for m in sdf_root.findall("model")}
        missing = [name for name in EXPECTED_WATERMELONS if name not in model_names]
        assert not missing, f"Missing watermelon models: {missing}"

    def test_watermelons_are_static(self, sdf_root):
        for name in EXPECTED_WATERMELONS:
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            static = model.findtext("static")
            assert static == "true", f"{name} must be static=true"

    def test_watermelon_poses(self, sdf_root):
        for name, expected_xyz in EXPECTED_WATERMELONS.items():
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            pose_text = model.findtext("pose")
            assert pose_text is not None, f"{name} has no pose element"
            x, y, z = parse_pose(pose_text)
            assert (x, y, z) == pytest.approx(expected_xyz, abs=0.001), (
                f"{name} pose mismatch. Expected {expected_xyz}, got ({x}, {y}, {z})"
            )

    def test_watermelons_use_sphere_geometry(self, sdf_root):
        for name in EXPECTED_WATERMELONS:
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            sphere = model.find(".//visual//sphere")
            assert sphere is not None, f"{name} visual should use sphere geometry"

    def test_watermelons_within_ur5_reach(self, sdf_root):
        for name, (x, y, z) in EXPECTED_WATERMELONS.items():
            dist = xy_distance(x, y)
            assert dist <= UR5_MAX_REACH, (
                f"{name} at ({x}, {y}) is {dist:.3f}m from base — "
                f"exceeds UR5 max reach of {UR5_MAX_REACH}m"
            )


class TestWeedModels:

    def test_all_weeds_present(self, sdf_root):
        model_names = {m.get("name") for m in sdf_root.findall("model")}
        missing = [name for name in EXPECTED_WEEDS if name not in model_names]
        assert not missing, f"Missing weed models: {missing}"

    def test_weeds_are_static(self, sdf_root):
        for name in EXPECTED_WEEDS:
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            static = model.findtext("static")
            assert static == "true", f"{name} must be static=true"

    def test_weed_poses(self, sdf_root):
        for name, expected_xyz in EXPECTED_WEEDS.items():
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            pose_text = model.findtext("pose")
            assert pose_text is not None, f"{name} has no pose element"
            x, y, z = parse_pose(pose_text)
            assert (x, y, z) == pytest.approx(expected_xyz, abs=0.001), (
                f"{name} pose mismatch. Expected {expected_xyz}, got ({x}, {y}, {z})"
            )

    def test_weeds_use_cylinder_geometry(self, sdf_root):
        for name in EXPECTED_WEEDS:
            model = next(m for m in sdf_root.findall("model") if m.get("name") == name)
            cylinder = model.find(".//visual//cylinder")
            assert cylinder is not None, f"{name} visual should use cylinder geometry"

    def test_weeds_within_ur5_reach(self, sdf_root):
        for name, (x, y, z) in EXPECTED_WEEDS.items():
            dist = xy_distance(x, y)
            assert dist <= UR5_MAX_REACH, (
                f"{name} at ({x}, {y}) is {dist:.3f}m from base — "
                f"exceeds UR5 max reach of {UR5_MAX_REACH}m"
            )


class TestLaserNotInWorld:

    def test_no_laser_mount_model(self, sdf_root):
        """The laser is now laser_link fixed to base_link, not a standalone world model."""
        names = {m.get("name") for m in sdf_root.findall("model")}
        assert "laser_mount" not in names, \
            "laser_mount should not be a world model; it is now laser_link on base_link"


class TestModelCount:

    def test_total_model_count(self, sdf_root):
        models = sdf_root.findall("model")
        names = [m.get("name") for m in models]
        assert len(models) == 8, (
            f"Expected 8 models (ground_plane + 4 watermelons + 3 weeds), "
            f"got {len(models)}: {names}"
        )

    def test_ground_plane_present(self, sdf_root):
        names = {m.get("name") for m in sdf_root.findall("model")}
        assert "ground_plane" in names, "ground_plane model missing"
