"""
test_dry_run_integration.py — end-to-end check that dry_run=true prevents
any /laser_fire publication even when a weed detection is injected.

Uses the same mock-ROS approach as test_dry_run.py; no live ROS context needed.
"""
import sys
from unittest.mock import MagicMock

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

# Make Node a real class so __new__ works
class _FakeNode:
    pass
sys.modules['rclpy.node'].Node = _FakeNode

import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
State = _mod.State


def _make_node(dry_run: bool):
    node = _mod.ArmControllerNode.__new__(_mod.ArmControllerNode)
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
        now=MagicMock(return_value=MagicMock(nanoseconds=int(5e9)))))
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.create_subscription = MagicMock()
    node.create_timer = MagicMock()
    node._action = MagicMock()
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._fire_pub = MagicMock()
    node._busy = False
    node._state = State.WAITING
    node._scan_idx = 0
    node._current_joints = [0.0] * 6
    node._last_weed_det = (320, 380)   # weed below centre
    node._wait_start = 0.0             # _now() >> scan_dwell_time -> moves to MOVE_TO_WEED
    node._fire_start = 0.0
    node._lock_start = 0.0
    node._treated_pans = []
    node._treated_world_xy = []
    return node


class TestDryRunEndToEnd:
    def test_no_laser_fire_published_through_full_cycle(self):
        """Simulate WAITING -> MOVE_TO_WEED -> (centred) -> FIRE_LASER with dry_run=True.
        /laser_fire must never be published."""
        node = _make_node(dry_run=True)

        # Tick 1: WAITING — weed detected, TF unavailable -> goes to MOVE_TO_WEED
        node._pixel_to_world_xy = MagicMock(return_value=None)
        node._tick()
        assert node._state == State.MOVE_TO_WEED

        # Drive MOVE_TO_WEED -> FIRE_LASER: inject weed already centred
        node._last_weed_det = (320, 240)   # dead centre -> err_x=0, err_y=0
        node._state = State.MOVE_TO_WEED
        node._move_to = MagicMock()
        node._tick()
        assert node._state == State.FIRE_LASER

        # Tick: FIRE_LASER with dry_run=True -> SCAN_MOVE, no publish
        node._tick()
        assert node._state == State.SCAN_MOVE
        node._fire_pub.publish.assert_not_called()

    def test_with_dry_run_false_fire_is_published(self):
        """Sanity check: dry_run=False still publishes /laser_fire."""
        node = _make_node(dry_run=False)
        node._state = State.FIRE_LASER
        node._tick()
        assert node._state == State.FIRING
        node._fire_pub.publish.assert_called_once()
