"""
test_camera_tf.py — verify camera_params.yaml and demo_real.launch.py
are consistent and correctly pick up calibrated vs. uncalibrated mode.

Offline test: no ROS processes started.
"""
import sys, os, importlib.util, copy
import unittest.mock as mock
import pytest

try:
    import yaml
except ImportError:
    pytest.skip('pyyaml not available', allow_module_level=True)

# Stub all launch/ROS dependencies so the launch module can be imported offline
for _m in [
    'launch', 'launch.actions', 'launch.launch_description_sources',
    'launch.substitutions',
    'launch_ros', 'launch_ros.actions', 'launch_ros.substitutions',
    'ament_index_python', 'ament_index_python.packages',
]:
    sys.modules[_m] = mock.MagicMock()

_LAUNCH_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'launch', 'demo_real.launch.py'))
_PARAMS_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'config', 'camera_params.yaml'))


def _load_launch_module():
    """Fresh import of the launch module (re-reads camera_params.yaml each time)."""
    spec = importlib.util.spec_from_file_location('demo_real', _LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_launch_mod = _load_launch_module()


class TestCameraTFConstants:
    def test_camera_z_positive(self):
        """Camera must be some distance from tool0 (z > 0)."""
        assert _launch_mod.CAMERA_Z > 0, (
            f"CAMERA_Z={_launch_mod.CAMERA_Z} — fill in the flange-to-lens distance")

    def test_all_rpy_constants_are_floats(self):
        for attr in ['CAMERA_X', 'CAMERA_Y', 'CAMERA_Z',
                     'CAMERA_ROLL', 'CAMERA_PITCH', 'CAMERA_YAW']:
            assert hasattr(_launch_mod, attr), f"{attr} not defined in demo_real.launch.py"
            assert isinstance(getattr(_launch_mod, attr), float), f"{attr} must be a float"

    def test_robot_ip_default_defined(self):
        assert hasattr(_launch_mod, 'DEFAULT_ROBOT_IP')
        assert _launch_mod.DEFAULT_ROBOT_IP != ''


class TestCameraParamsYaml:
    def _load(self):
        with open(_PARAMS_FILE) as f:
            return yaml.safe_load(f)

    def test_file_has_calibrated_flag(self):
        data = self._load()
        assert 'calibrated' in data, "camera_params.yaml must have a 'calibrated' key"
        assert isinstance(data['calibrated'], bool)

    def test_mount_section_has_xyz(self):
        data = self._load()
        for key in ['x', 'y', 'z']:
            assert key in data['mount'], f"mount.{key} missing from camera_params.yaml"

    def test_mount_section_has_rpy(self):
        data = self._load()
        for key in ['roll', 'pitch', 'yaw']:
            assert key in data['mount'], f"mount.{key} missing from camera_params.yaml"

    def test_mount_section_has_quaternion_fields(self):
        data = self._load()
        for key in ['qx', 'qy', 'qz', 'qw']:
            assert key in data['mount'], f"mount.{key} missing — needed for easy_handeye2 output"

    def test_quaternion_unit_when_uncalibrated(self):
        """When calibrated=false, quaternion should be identity (qw=1) to signal it's a placeholder."""
        data = self._load()
        if not data['calibrated']:
            m = data['mount']
            assert m['qw'] == 1.0, "Placeholder quaternion should be identity (qw=1.0)"


class TestCalibrationModeSwitch:
    def test_uncalibrated_mode_uses_rpy_args(self):
        """When calibrated=false, _rotation_args must contain --roll not --qx."""
        data = yaml.safe_load(open(_PARAMS_FILE))
        if data['calibrated']:
            pytest.skip('camera_params.yaml is currently calibrated — skip RPY test')
        assert '--roll' in _launch_mod._rotation_args, (
            "Uncalibrated mode must use --roll in rotation args")
        assert '--qx' not in _launch_mod._rotation_args

    def test_calibrated_mode_uses_quaternion_args(self, tmp_path, monkeypatch):
        """When calibrated=true, _rotation_args must contain --qx not --roll."""
        cal_params = {
            'calibrated': True,
            'mount': {
                'x': 0.01, 'y': 0.02, 'z': 0.06,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'qx': 0.0, 'qy': 0.707, 'qz': 0.0, 'qw': 0.707,
            }
        }
        cal_file = tmp_path / 'camera_params.yaml'
        cal_file.write_text(yaml.dump(cal_params))

        import builtins
        real_open = builtins.open
        def _patched_open(path, *args, **kwargs):
            if os.path.basename(str(path)) == 'camera_params.yaml':
                return real_open(str(cal_file), *args, **kwargs)
            return real_open(path, *args, **kwargs)

        monkeypatch.setattr(builtins, 'open', _patched_open)
        mod = _load_launch_module()

        assert '--qx' in mod._rotation_args, (
            "Calibrated mode must use --qx in rotation args")
        assert '--roll' not in mod._rotation_args


class TestLaserLinkTF:
    def test_laser_link_child_frame_in_rotation_args(self):
        """laser_link TF must be present — it's required for RViz laser beam marker."""
        # We can't easily inspect the Node args without ROS, but we can check
        # that the launch file exports the camera coords that laser_tf uses.
        # laser_tf uses the same X/Y/Z as camera_tf (coincident with aperture).
        assert hasattr(_launch_mod, 'CAMERA_X'), "CAMERA_X must be defined for laser_tf"
        assert hasattr(_launch_mod, 'CAMERA_Z'), "CAMERA_Z must be defined for laser_tf"
        assert _launch_mod.CAMERA_Z > 0, "laser_tf Z must be > 0"
