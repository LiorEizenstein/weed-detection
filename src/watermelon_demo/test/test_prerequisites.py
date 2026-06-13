"""
Tests for project prerequisites.
Verifies that all required Python packages are installed correctly.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_prerequisites.py -v
"""

import subprocess
import sys
import pytest


class TestPythonDependencies:

    def test_ultralytics_importable(self):
        """ultralytics must be importable."""
        import importlib
        spec = importlib.util.find_spec("ultralytics")
        assert spec is not None, (
            "ultralytics is not installed. Run: pip3 install ultralytics --break-system-packages"
        )

    def test_yolo_loads_model(self):
        """YOLO must load best.pt without errors."""
        from pathlib import Path
        model_path = Path.home() / "best.pt"
        assert model_path.exists(), f"Model not found at {model_path}"

        from ultralytics import YOLO
        model = YOLO(str(model_path))
        assert model is not None

    def test_yolo_class_names(self):
        """best.pt must have exactly the expected 3 classes."""
        from pathlib import Path
        from ultralytics import YOLO

        model = YOLO(str(Path.home() / "best.pt"))
        expected = {0: "watermelon", 1: "weed_side", 2: "weed_top"}
        assert model.names == expected, (
            f"Unexpected class mapping.\n"
            f"  Expected: {expected}\n"
            f"  Got:      {model.names}\n"
            f"  If class names differ, update WEED_CLASSES and aim point logic in detection_node.py"
        )

    def test_cv2_importable(self):
        """OpenCV (cv2) must be importable."""
        import importlib
        spec = importlib.util.find_spec("cv2")
        assert spec is not None, "cv2 not available. Install via apt or pip."

    def test_numpy_version_below_2(self):
        """NumPy must be < 2.0 to avoid matplotlib/cv2 compatibility issues."""
        import numpy as np
        major = int(np.__version__.split(".")[0])
        assert major < 2, (
            f"NumPy {np.__version__} is >= 2.0 which breaks system matplotlib and cv2. "
            f"Run: pip3 install 'numpy<2' --break-system-packages"
        )


class TestROSPackages:

    def test_cv_bridge_available(self):
        """cv_bridge ROS package must be installed."""
        result = subprocess.run(
            ["ros2", "pkg", "list"],
            capture_output=True, text=True
        )
        assert "cv_bridge" in result.stdout, (
            "cv_bridge not found. Run: sudo apt install ros-jazzy-cv-bridge"
        )

    def test_vision_msgs_available(self):
        """vision_msgs ROS package must be installed."""
        result = subprocess.run(
            ["ros2", "pkg", "list"],
            capture_output=True, text=True
        )
        assert "vision_msgs" in result.stdout, (
            "vision_msgs not found. Run: sudo apt install ros-jazzy-vision-msgs"
        )

    def test_ur_packages_available(self):
        """Core UR packages must be installed."""
        result = subprocess.run(
            ["ros2", "pkg", "list"],
            capture_output=True, text=True
        )
        required = [
            "ur_description",
            "ur_simulation_gz",
            "ur_moveit_config",
            "ur_robot_driver",
        ]
        missing = [pkg for pkg in required if pkg not in result.stdout]
        assert not missing, f"Missing UR packages: {missing}"
