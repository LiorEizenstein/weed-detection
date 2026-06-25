"""
Tests for Step 4 — ROS2 Python nodes.
Checks importability, structure, and key constants without spinning ROS.

Run with:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_nodes.py -v
"""

import importlib
import math
import pytest
from pathlib import Path

NODE_PKG = 'watermelon_demo'


def import_node(name):
    return importlib.import_module(f'{NODE_PKG}.{name}')


class TestDetectionNode:

    @classmethod
    def setup_class(cls):
        import sys
        # Other test files mock vision_msgs at module level; remove those mocks
        # so detection_node is imported with the real vision_msgs classes.
        for mod_name in ['vision_msgs', 'vision_msgs.msg']:
            if mod_name in sys.modules:
                del sys.modules[mod_name]
        if 'watermelon_demo.detection_node' in sys.modules:
            del sys.modules['watermelon_demo.detection_node']

    def test_module_imports(self):
        mod = import_node('detection_node')
        assert hasattr(mod, 'DetectionNode')

    def test_weed_class_ids_correct(self):
        mod = import_node('detection_node')
        assert mod.WEED_CLASSES == {1, 2}, \
            "WEED_CLASSES should be {1, 2} (weed_side, weed_top)"

    def test_confidence_threshold(self):
        mod = import_node('detection_node')
        assert 0.5 <= mod.CONFIDENCE_THRESHOLD <= 1.0

    def test_make_detection_static(self):
        mod = import_node('detection_node')
        det = mod.DetectionNode._make_detection(1, 0.9, 100, 200, 50, 60)
        assert det.bbox.center.position.x == pytest.approx(100.0)
        assert det.bbox.center.position.y == pytest.approx(200.0)
        assert det.results[0].hypothesis.class_id == '1'
        assert det.results[0].hypothesis.score == pytest.approx(0.9)


class TestArmControllerNode:

    def test_module_imports(self):
        mod = import_node('arm_controller_node')
        assert hasattr(mod, 'ArmControllerNode')

    def test_scan_poses_sweep_wide_pan(self):
        """shoulder_pan (joint 0) must sweep a wide arc so the arm scans left to right."""
        mod = import_node('arm_controller_node')
        pans = [pose[0] for pose in mod.SCAN_POSES]
        span = max(pans) - min(pans)
        assert span >= 2.5, \
            f"Pan sweep should span >= 2.5 rad (~140 deg) left-right, got {span:.2f} rad"

    def test_scan_poses_small_steps(self):
        """Consecutive scan poses must differ by a small pan step for a smooth sweep."""
        mod = import_node('arm_controller_node')
        pans = [pose[0] for pose in mod.SCAN_POSES]
        max_step = max(abs(a - b) for a, b in zip(pans, pans[1:]))
        assert max_step <= 0.2, \
            f"Pan step between poses should be small (<=0.2 rad), got {max_step:.2f} rad"
        assert len(mod.SCAN_POSES) >= 12, \
            f"Small steps over the arc should give many poses, got {len(mod.SCAN_POSES)}"

    def test_scan_poses_joint_count(self):
        mod = import_node('arm_controller_node')
        for pose in mod.SCAN_POSES:
            assert len(pose) == 6, f"Each scan pose needs 6 joint values, got {len(pose)}"

    def test_joint_names_count(self):
        mod = import_node('arm_controller_node')
        assert len(mod.JOINT_NAMES) == 6

    def test_weed_class_ids_are_strings(self):
        mod = import_node('arm_controller_node')
        for cid in mod.WEED_CLASS_IDS:
            assert isinstance(cid, str), \
                "WEED_CLASS_IDS must be strings to match Detection2D class_id"

    def test_state_constants_exist(self):
        mod = import_node('arm_controller_node')
        for attr in ('INIT', 'SCAN_MOVE', 'WAITING', 'MOVE_TO_WEED',
                     'FIRE_LASER', 'RETURN_HOME'):
            assert hasattr(mod.State, attr), f"State.{attr} missing"


class TestLaserEffectNode:

    def test_module_imports(self):
        mod = import_node('laser_effect_node')
        assert hasattr(mod, 'LaserEffectNode')

    def test_laser_position_is_tf_based(self):
        """Laser moves with the arm (on tool0), so world position comes from TF, not constants."""
        mod = import_node('laser_effect_node')
        assert not hasattr(mod, 'LASER_X'), \
            "LASER_X must not exist — laser world position is looked up via TF (laser_link frame)"

    def test_camera_intrinsics_consistent(self):
        mod = import_node('laser_effect_node')
        expected_fx = (mod.IMG_W / 2) / math.tan(mod.HFOV / 2)
        assert mod.FX == pytest.approx(expected_fx, rel=0.01), \
            "FX must be derived from IMG_W and HFOV"

    def test_quat_to_matrix_identity(self):
        mod = import_node('laser_effect_node')
        import numpy as np
        R = mod.LaserEffectNode._quat_to_matrix(0, 0, 0, 1)
        assert np.allclose(R, np.eye(3), atol=1e-6), \
            "Identity quaternion should produce identity rotation matrix"

    def test_weed_ground_z(self):
        mod = import_node('laser_effect_node')
        assert mod.WEED_GROUND_Z == pytest.approx(0.03), \
            "WEED_GROUND_Z must match weed height in SDF"


class TestFieldManagerNode:

    def test_module_imports(self):
        mod = import_node('field_manager_node')
        assert hasattr(mod, 'FieldManagerNode')

    def test_watermelon_count(self):
        mod = import_node('field_manager_node')
        assert len(mod.WATERMELONS) == 4, "Must match 4 watermelons in SDF"

    def test_weed_count(self):
        mod = import_node('field_manager_node')
        assert len(mod.WEEDS) == 3, "Must match 3 weeds in SDF"

    def test_weed_positions_match_sdf(self):
        mod = import_node('field_manager_node')
        expected = {
            (0.55,  0.35, 0.03),
            (0.45, -0.35, 0.03),
            (0.65, -0.1,  0.03),
        }
        actual = {(x, y, z) for (x, y, z) in mod.WEEDS.values()}
        assert actual == expected, \
            f"Weed positions do not match SDF.\nExpected: {expected}\nGot: {actual}"

    def test_no_laser_mount_marker(self):
        """The laser is shown via the robot model now, not a field marker."""
        mod = import_node('field_manager_node')
        assert not hasattr(mod, 'LASER_MOUNT'), \
            "field_manager should not publish a laser marker; laser_link shows via RobotModel"

    def test_no_duplicate_marker_ids(self):
        mod = import_node('field_manager_node')
        all_ids = (list(mod.WATERMELONS.keys()) +
                   list(mod.WEEDS.keys()))
        assert len(all_ids) == len(set(all_ids)), \
            "All marker IDs must be unique to avoid RViz overwrites"
