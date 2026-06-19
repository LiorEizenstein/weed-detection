"""
test_dry_run.py — verify arm_controller_node dry_run parameter behaviour.

Mocks all ROS 2 dependencies so the node class can be instantiated
without a running ROS context (same pattern as test_centering.py).
"""
import sys
import math
from unittest.mock import MagicMock, patch, call

# ── Mock every ROS2 dependency before any import ────────────────────────────
for _m in [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.time',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'control_msgs', 'control_msgs.action',
    'trajectory_msgs', 'trajectory_msgs.msg',
    'builtin_interfaces', 'builtin_interfaces.msg',
    'tf2_ros',
]:
    sys.modules[_m] = MagicMock()

# Node must be a real class so that ArmControllerNode(Node) produces a real
# class rather than a MagicMock attribute — __new__ on a MagicMock class fails.
class _FakeNode:
    pass

sys.modules['rclpy.node'].Node = _FakeNode

import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
State = _mod.State


def _make_node(dry_run: bool):
    """Instantiate ArmControllerNode with mocked ROS infrastructure."""
    node = _mod.ArmControllerNode.__new__(_mod.ArmControllerNode)
    # Mock parameter store
    params = {
        'scan_dwell_time': 2.0,
        'laser_fire_duration': 1.0,
        'dry_run': dry_run,
    }
    node.get_parameter = lambda name: MagicMock(value=params[name])
    node.declare_parameter = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(nanoseconds=0))))
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.create_subscription = MagicMock()
    node.create_timer = MagicMock()
    node._action = MagicMock()
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._fire_pub = MagicMock()
    node._busy = False
    node._state = State.FIRE_LASER
    node._scan_idx = 0
    node._current_joints = [0.0] * 6
    node._last_weed_det = (320, 240)
    node._wait_start = 0.0
    node._fire_start = 0.0
    node._lock_start = 0.0
    node._treated_pans = []
    node._treated_world_xy = []
    return node


class TestDryRunSkipsFire:
    def test_state_transitions_to_scan_move_not_firing(self):
        """dry_run=True: FIRE_LASER → SCAN_MOVE, never enters FIRING."""
        node = _make_node(dry_run=True)
        node._tick()
        assert node._state == State.SCAN_MOVE, (
            f"Expected SCAN_MOVE, got {node._state}")

    def test_laser_fire_not_published(self):
        """dry_run=True: /laser_fire True is never published."""
        node = _make_node(dry_run=True)
        node._tick()
        node._fire_pub.publish.assert_not_called()

    def test_treated_lists_remain_empty(self):
        """dry_run=True: no pan or world position is blacklisted."""
        node = _make_node(dry_run=True)
        node._tick()
        assert node._treated_pans == []
        assert node._treated_world_xy == []


class TestDryRunFalseFiresNormally:
    def test_state_transitions_to_firing(self):
        """dry_run=False: FIRE_LASER → FIRING."""
        node = _make_node(dry_run=False)
        node._tick()
        assert node._state == State.FIRING, (
            f"Expected FIRING, got {node._state}")

    def test_laser_fire_true_published(self):
        """dry_run=False: /laser_fire True is published."""
        from std_msgs.msg import Bool as _Bool
        node = _make_node(dry_run=False)
        node._tick()
        node._fire_pub.publish.assert_called_once()
