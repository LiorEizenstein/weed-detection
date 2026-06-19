"""
test_camera_tf.py — verify the static TF constants in demo_real.launch.py
are present and non-trivially defined (not all zeros from a forgotten measurement).

This is an offline test: it imports the launch module and reads the
CAMERA_* constants without starting any ROS processes.
"""
import sys, os, importlib, importlib.util, types
import unittest.mock as mock

# Stub launch dependencies so we can import the module without ROS
for _m in ['launch', 'launch.actions', 'launch.launch_description_sources',
           'launch_ros', 'launch_ros.actions', 'launch_ros.substitutions',
           'ament_index_python', 'ament_index_python.packages']:
    sys.modules[_m] = mock.MagicMock()

# Add the launch file's directory to path
_launch_dir = os.path.join(os.path.dirname(__file__), '..', 'launch')
_launch_file = os.path.join(os.path.abspath(_launch_dir), 'demo_real.launch.py')

# Load the .launch.py file by path (filename contains two dots, normal import won't work)
_spec = importlib.util.spec_from_file_location('demo_real', _launch_file)
_launch_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_launch_mod)


class TestCameraTFConstants:
    def test_camera_z_positive(self):
        """Camera must be mounted some distance from tool0 (z > 0)."""
        assert _launch_mod.CAMERA_Z > 0, (
            f"CAMERA_Z={_launch_mod.CAMERA_Z} — measure the tool0→camera distance")

    def test_all_constants_defined(self):
        for attr in ['CAMERA_X', 'CAMERA_Y', 'CAMERA_Z',
                     'CAMERA_ROLL', 'CAMERA_PITCH', 'CAMERA_YAW']:
            assert hasattr(_launch_mod, attr), f"{attr} not defined in demo_real.launch.py"
            assert isinstance(getattr(_launch_mod, attr), float), (
                f"{attr} must be a float")

    def test_robot_ip_default_defined(self):
        assert hasattr(_launch_mod, 'DEFAULT_ROBOT_IP'), (
            "DEFAULT_ROBOT_IP must be defined in demo_real.launch.py")
        assert _launch_mod.DEFAULT_ROBOT_IP != '', "DEFAULT_ROBOT_IP must not be empty"
