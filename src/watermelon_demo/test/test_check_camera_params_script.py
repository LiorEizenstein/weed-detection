"""
test_check_camera_params_script.py — dry-run tests for scripts/check_camera_params.py.

Exercises the check() function with synthetic camera_params.yaml files
to verify it catches bad values and accepts good ones.
"""
import os, sys, math, textwrap, pathlib
import pytest

try:
    import yaml
except ImportError:
    pytest.skip('pyyaml not available', allow_module_level=True)

# Import the script as a module
_SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts', 'check_camera_params.py'))
sys.path.insert(0, os.path.dirname(_SCRIPT))
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location('check_camera_params', _SCRIPT)
_script = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_script)
check = _script.check


def _write(tmp_path, data: dict) -> str:
    p = tmp_path / 'camera_params.yaml'
    p.write_text(yaml.dump(data))
    return str(p)


def _good_manual(overrides=None):
    base = {
        'calibrated': False,
        'mount': {'x': 0.01, 'y': 0.0, 'z': 0.06,
                  'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    }
    if overrides:
        base['mount'].update(overrides)
    return base


def _good_calibrated(overrides=None):
    _r2 = 1.0 / math.sqrt(2)   # exact 1/√2 so quaternion norm = 1.0
    base = {
        'calibrated': True,
        'mount': {'x': 0.01, 'y': -0.01, 'z': 0.06,
                  'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                  'qx': 0.0, 'qy': _r2, 'qz': 0.0, 'qw': _r2}
    }
    if overrides:
        base['mount'].update(overrides)
    return base


# ── Happy paths ───────────────────────────────────────────────────────────────

class TestValidInputs:
    def test_good_manual_params_pass(self, tmp_path, capsys):
        check(_write(tmp_path, _good_manual()))
        out = capsys.readouterr().out
        assert '✓ All checks passed' in out

    def test_good_calibrated_params_pass(self, tmp_path, capsys):
        check(_write(tmp_path, _good_calibrated()))
        out = capsys.readouterr().out
        assert '✓ All checks passed' in out

    def test_mode_label_manual(self, tmp_path, capsys):
        check(_write(tmp_path, _good_manual()))
        out = capsys.readouterr().out
        assert 'manual measurement' in out

    def test_mode_label_calibrated(self, tmp_path, capsys):
        check(_write(tmp_path, _good_calibrated()))
        out = capsys.readouterr().out
        assert 'easy_handeye2' in out

    def test_calibrated_shows_rpy_equivalent(self, tmp_path, capsys):
        """Calibrated mode prints equivalent RPY for human readability."""
        check(_write(tmp_path, _good_calibrated()))
        out = capsys.readouterr().out
        assert 'Equivalent RPY' in out


# ── Translation error detection ───────────────────────────────────────────────

class TestTranslationErrors:
    def test_z_zero_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'z': 0.0})))

    def test_z_too_large_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'z': 0.50})))

    def test_z_too_small_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'z': 0.001})))

    def test_x_too_large_warns(self, tmp_path, capsys):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'x': 0.20})))
        out = capsys.readouterr().out
        assert '✗' in out

    def test_y_too_large_warns(self, tmp_path, capsys):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'y': -0.20})))
        out = capsys.readouterr().out
        assert '✗' in out


# ── Quaternion error detection ────────────────────────────────────────────────

class TestQuaternionErrors:
    def test_non_unit_quaternion_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_calibrated(
                {'qx': 1.0, 'qy': 1.0, 'qz': 1.0, 'qw': 1.0})))

    def test_identity_quaternion_with_calibrated_true_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_calibrated(
                {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})))

    def test_qw_near_zero_raises(self, tmp_path):
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_calibrated(
                {'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0})))


# ── RPY angle unit check ──────────────────────────────────────────────────────

class TestRPYAngleUnits:
    def test_angle_in_degrees_raises(self, tmp_path):
        """Angles > π rad are almost certainly degrees — must be caught."""
        with pytest.raises(SystemExit):
            check(_write(tmp_path, _good_manual({'pitch': 45.0})))  # 45 degrees, not radians

    def test_small_valid_angle_passes(self, tmp_path, capsys):
        check(_write(tmp_path, _good_manual({'pitch': 0.3})))  # ~17° downward tilt
        out = capsys.readouterr().out
        assert '✓ All checks passed' in out
