"""
Tests for Step 5 — config files.
Verifies demo_params.yaml structure and demo_rviz.rviz presence.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_config.py -v
"""

import yaml
import pytest
from pathlib import Path

CONFIG_DIR = Path(__file__).parent.parent / "config"
PARAMS_FILE = CONFIG_DIR / "demo_params.yaml"
RVIZ_FILE   = CONFIG_DIR / "demo_rviz.rviz"

EXPECTED_NODES = [
    "arm_controller_node",
    "detection_node",
    "laser_effect_node",
    "field_manager_node",
]


@pytest.fixture(scope="module")
def params():
    assert PARAMS_FILE.exists(), f"demo_params.yaml not found at {PARAMS_FILE}"
    return yaml.safe_load(PARAMS_FILE.read_text())


class TestDemoParams:

    def test_file_exists(self):
        assert PARAMS_FILE.exists(), "config/demo_params.yaml missing"

    def test_is_valid_yaml(self):
        try:
            yaml.safe_load(PARAMS_FILE.read_text())
        except yaml.YAMLError as e:
            pytest.fail(f"demo_params.yaml is not valid YAML: {e}")

    def test_all_nodes_present(self, params):
        missing = [n for n in EXPECTED_NODES if n not in params]
        assert not missing, f"Missing node sections in demo_params.yaml: {missing}"

    def test_ros_parameters_key(self, params):
        for node in EXPECTED_NODES:
            assert "ros__parameters" in params[node], \
                f"{node} is missing 'ros__parameters' key"

    def test_use_sim_time_set(self, params):
        for node in EXPECTED_NODES:
            val = params[node]["ros__parameters"].get("use_sim_time")
            assert val is True, \
                f"{node}.ros__parameters.use_sim_time must be true (got {val!r})"

    def test_detection_model_path(self, params):
        path = params["detection_node"]["ros__parameters"].get("model_path")
        assert path == "/home/lior/best.pt", \
            f"detection_node.model_path should be '/home/lior/best.pt', got {path!r}"

    def test_scan_dwell_time_positive(self, params):
        val = params["arm_controller_node"]["ros__parameters"]["scan_dwell_time"]
        assert val > 0, f"scan_dwell_time must be > 0, got {val}"

    def test_laser_fire_duration_positive(self, params):
        val = params["arm_controller_node"]["ros__parameters"]["laser_fire_duration"]
        assert val > 0, f"laser_fire_duration must be > 0, got {val}"


class TestRvizConfig:

    def test_rviz_file_exists(self):
        assert RVIZ_FILE.exists(), "config/demo_rviz.rviz missing"

    def test_rviz_is_valid_yaml(self):
        try:
            yaml.safe_load(RVIZ_FILE.read_text())
        except yaml.YAMLError as e:
            pytest.fail(f"demo_rviz.rviz is not valid YAML: {e}")

    def test_fixed_frame_is_world(self):
        data = yaml.safe_load(RVIZ_FILE.read_text())
        frame = (data.get("Visualization Manager", {})
                     .get("Global Options", {})
                     .get("Fixed Frame"))
        assert frame == "world", \
            f"RViz Fixed Frame should be 'world', got {frame!r}"

    def test_required_displays_present(self):
        data = yaml.safe_load(RVIZ_FILE.read_text())
        displays = data.get("Visualization Manager", {}).get("Displays", [])
        names = {d.get("Name", "") for d in displays}
        required = {"RobotModel", "TF", "Laser Beam", "Field Plants"}
        missing = required - names
        assert not missing, f"RViz config missing displays: {missing}"

    def test_laser_beam_topic(self):
        data = yaml.safe_load(RVIZ_FILE.read_text())
        displays = data.get("Visualization Manager", {}).get("Displays", [])
        laser = next((d for d in displays if d.get("Name") == "Laser Beam"), None)
        assert laser is not None
        topic = laser.get("Topic", {}).get("Value")
        assert topic == "/visualization_marker", \
            f"Laser Beam display should subscribe to /visualization_marker, got {topic!r}"

    def test_field_plants_topic(self):
        data = yaml.safe_load(RVIZ_FILE.read_text())
        displays = data.get("Visualization Manager", {}).get("Displays", [])
        plants = next((d for d in displays if d.get("Name") == "Field Plants"), None)
        assert plants is not None
        topic = plants.get("Topic", {}).get("Value")
        assert topic == "/field_markers", \
            f"Field Plants display should subscribe to /field_markers, got {topic!r}"
