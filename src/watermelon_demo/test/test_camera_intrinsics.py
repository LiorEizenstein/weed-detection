"""
test_camera_intrinsics.py — verify laser_effect_node updates its ray-cast
intrinsics from CameraInfo messages and falls back to hardcoded values when
no CameraInfo has been received.
"""
import sys, math
from unittest.mock import MagicMock

_mods = [
    'rclpy', 'rclpy.node', 'rclpy.time',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'visualization_msgs', 'visualization_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'tf2_ros',
    'builtin_interfaces', 'builtin_interfaces.msg',
]
for _m in _mods:
    sys.modules[_m] = MagicMock()

# Make Node a real class so __new__ works
class _FakeNode:
    pass
sys.modules['rclpy.node'].Node = _FakeNode

import importlib
_laser_mod = importlib.import_module('watermelon_demo.laser_effect_node')
LaserEffectNode = _laser_mod.LaserEffectNode


def _make_laser_node():
    node = LaserEffectNode.__new__(LaserEffectNode)
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.create_subscription = MagicMock()
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.destroy_subscription = MagicMock()
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(to_msg=MagicMock()))))
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._marker_pub = MagicMock()
    node._last_weed_pixel = None
    node._info_sub = MagicMock()
    # Initialise intrinsics tuple to hardcoded fallback values
    node._intrinsics = (_laser_mod.FX, _laser_mod.FY, _laser_mod.CX, _laser_mod.CY)
    return node


def _make_camera_info(fx, fy, cx, cy):
    """Build a minimal CameraInfo mock with a K matrix."""
    info = MagicMock()
    # K is row-major 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


class TestCameraInfoUpdatesIntrinsics:
    def test_fx_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=600.0, cx=320.0, cy=240.0))
        fx, fy, cx, cy = node._intrinsics
        assert fx == 600.0, f"Expected fx=600.0, got {fx}"

    def test_fy_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=605.0, cx=320.0, cy=240.0))
        fx, fy, cx, cy = node._intrinsics
        assert fy == 605.0, f"Expected fy=605.0, got {fy}"

    def test_cx_cy_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=600.0, cx=321.5, cy=241.5))
        fx, fy, cx, cy = node._intrinsics
        assert cx == 321.5
        assert cy == 241.5

    def test_degenerate_camera_info_ignored(self):
        """CameraInfo with fx=0 must be ignored (guards against bad calibration)."""
        node = _make_laser_node()
        original = node._intrinsics
        bad_info = _make_camera_info(fx=0.0, fy=0.0, cx=320.0, cy=240.0)
        node._camera_info_cb(bad_info)
        assert node._intrinsics == original, "Degenerate CameraInfo should not update intrinsics"


class TestFallbackIntrinsics:
    def test_fallback_values_are_hardcoded_defaults(self):
        """Before any CameraInfo arrives, intrinsics equal the hardcoded Gazebo values."""
        node = _make_laser_node()
        fx, fy, cx, cy = node._intrinsics
        assert abs(fx - _laser_mod.FX) < 1e-6, (
            f"Fallback fx should be {_laser_mod.FX}, got {fx}")

    def test_pixel_to_world_uses_intrinsics_tuple(self):
        """_pixel_to_world must unpack self._intrinsics, not use module-level constants."""
        import inspect
        src = inspect.getsource(LaserEffectNode._pixel_to_world)
        assert 'self._intrinsics' in src, (
            "_pixel_to_world must use self._intrinsics, not module-level FX/CX")
