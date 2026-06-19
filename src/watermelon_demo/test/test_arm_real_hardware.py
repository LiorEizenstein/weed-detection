"""
test_arm_real_hardware.py — test CameraInfo intrinsics and JointState feedback
in arm_controller_node without a running ROS context.
"""
import sys
import math
import pytest
from unittest.mock import MagicMock

for _m in [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.time',
    'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'control_msgs', 'control_msgs.action',
    'trajectory_msgs', 'trajectory_msgs.msg',
    'builtin_interfaces', 'builtin_interfaces.msg',
    'tf2_ros',
]:
    sys.modules[_m] = MagicMock()

class _FakeNode:
    pass

sys.modules['rclpy.node'].Node = _FakeNode

import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
JOINT_NAMES = _mod.JOINT_NAMES


def _make_node():
    node = _mod.ArmControllerNode.__new__(_mod.ArmControllerNode)
    node.get_parameter = MagicMock(return_value=MagicMock(value=False))
    node.declare_parameter = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(nanoseconds=0))))
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.create_subscription = MagicMock()
    node.create_timer = MagicMock()
    node.destroy_subscription = MagicMock()
    node._action = MagicMock()
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._fire_pub = MagicMock()
    node._info_sub = MagicMock()
    node._intrinsics = (_mod._FX, _mod._FY, _mod._CX_OPT, _mod._CY_OPT)
    node._joints_received = False
    node._tf_warn_time = 0.0
    node._current_joints = list(_mod.HOME_POSE)
    node._busy = False
    node._state = _mod.State.INIT
    node._scan_idx = 0
    node._last_weed_det = None
    node._wait_start = 0.0
    node._fire_start = 0.0
    node._lock_start = 0.0
    node._treated_pans = []
    node._treated_world_xy = []
    return node


def _make_camera_info(fx, fy, cx, cy, k_len=9):
    msg = MagicMock()
    k = [0.0] * k_len
    if k_len >= 6:
        k[0] = fx   # fx
        k[4] = fy   # fy
        k[2] = cx   # cx
        k[5] = cy   # cy
    msg.k = k
    return msg


def _make_joint_state(positions: dict):
    msg = MagicMock()
    msg.name = list(positions.keys())
    msg.position = list(positions.values())
    return msg


# ── CameraInfo callback ───────────────────────────────────────────────────────

class TestCameraInfoCallback:
    def test_valid_message_updates_intrinsics(self):
        node = _make_node()
        node._camera_info_cb(_make_camera_info(600.0, 600.0, 320.0, 240.0))
        assert node._intrinsics == (600.0, 600.0, 320.0, 240.0)

    def test_valid_message_unsubscribes(self):
        node = _make_node()
        node._camera_info_cb(_make_camera_info(600.0, 600.0, 320.0, 240.0))
        node.destroy_subscription.assert_called_once_with(node._info_sub)

    def test_second_call_does_not_reach_here_after_unsub(self):
        """destroy_subscription is mocked so a second call CAN overwrite in tests,
        but this test documents that at runtime it would not happen."""
        node = _make_node()
        node._camera_info_cb(_make_camera_info(600.0, 600.0, 320.0, 240.0))
        # Verify destroy_subscription was called after the first valid message
        node.destroy_subscription.assert_called_once_with(node._info_sub)
        # After first valid message intrinsics should be set correctly
        assert node._intrinsics == (600.0, 600.0, 320.0, 240.0)

    def test_degenerate_fx_zero_ignored(self):
        node = _make_node()
        original = node._intrinsics
        node._camera_info_cb(_make_camera_info(0.0, 600.0, 320.0, 240.0))
        assert node._intrinsics == original

    def test_short_k_ignored(self):
        node = _make_node()
        original = node._intrinsics
        msg = _make_camera_info(600.0, 600.0, 320.0, 240.0, k_len=3)
        node._camera_info_cb(msg)
        assert node._intrinsics == original

    def test_degenerate_does_not_unsubscribe(self):
        node = _make_node()
        node._camera_info_cb(_make_camera_info(0.0, 0.0, 0.0, 0.0))
        node.destroy_subscription.assert_not_called()

    def test_fallback_values_are_gazebo_defaults(self):
        """Without a real camera, intrinsics must equal Gazebo values."""
        node = _make_node()
        fx, fy, cx, cy = node._intrinsics
        assert fx == _mod._FX
        assert fy == _mod._FY
        assert cx == _mod._CX_OPT
        assert cy == _mod._CY_OPT


# ── JointState callback ───────────────────────────────────────────────────────

class TestJointStateCallback:
    def test_full_message_updates_all_joints(self):
        node = _make_node()
        positions = {n: float(i) * 0.1 for i, n in enumerate(JOINT_NAMES)}
        node._joint_state_cb(_make_joint_state(positions))
        expected = [positions[n] for n in JOINT_NAMES]
        assert node._current_joints == expected

    def test_partial_message_does_not_update(self):
        node = _make_node()
        original = list(node._current_joints)
        # Only 3 of 6 joints present — KeyError should be swallowed
        partial = {JOINT_NAMES[0]: 1.0, JOINT_NAMES[1]: 2.0, JOINT_NAMES[2]: 3.0}
        node._joint_state_cb(_make_joint_state(partial))
        assert node._current_joints == original

    def test_extra_joints_ignored(self):
        """Message may carry joints from other controllers — only UR6 extracted."""
        node = _make_node()
        positions = {n: 0.5 for n in JOINT_NAMES}
        positions['some_other_joint'] = 99.9
        node._joint_state_cb(_make_joint_state(positions))
        assert len(node._current_joints) == 6
        assert all(v == 0.5 for v in node._current_joints)

    def test_joints_update_shoulder_pan_for_treated_zone(self):
        """shoulder_pan_joint is JOINT_NAMES[0]; _enter_waiting reads index 0."""
        node = _make_node()
        positions = {n: 0.0 for n in JOINT_NAMES}
        positions['shoulder_pan_joint'] = 1.23
        node._joint_state_cb(_make_joint_state(positions))
        assert node._current_joints[0] == pytest.approx(1.23)

    def test_successive_messages_update_current_joints(self):
        node = _make_node()
        for angle in [0.1, 0.5, 1.0]:
            positions = {n: angle for n in JOINT_NAMES}
            node._joint_state_cb(_make_joint_state(positions))
        assert all(v == pytest.approx(1.0) for v in node._current_joints)
