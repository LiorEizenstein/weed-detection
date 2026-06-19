"""
test_manual_calibration.py — validate manual camera mount measurements in
camera_params.yaml are physically plausible before running on the real robot.

Run this after filling in camera_params.yaml with calipers measurements:
    python3 -m pytest src/watermelon_demo/test/test_manual_calibration.py -v

All tests must pass before launching demo_real.launch.py with calibrated: false.
"""
import os, math
import pytest

try:
    import yaml
except ImportError:
    pytest.skip('pyyaml not available', allow_module_level=True)

_PARAMS_FILE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'config', 'camera_params.yaml'))

# Physical sanity bounds for a wrist-mounted RealSense D435 on a UR5
_Z_MIN, _Z_MAX   = 0.01, 0.25   # flange-to-lens: 1 cm to 25 cm
_XY_MAX          = 0.15          # max lateral/vertical offset: 15 cm
_ANGLE_MAX       = math.pi       # angles must be within ±π rad
_QW_MIN          = 1e-6          # valid quaternion must not have qw ≈ 0 (singularity)


def _load():
    with open(_PARAMS_FILE) as f:
        return yaml.safe_load(f)


@pytest.fixture
def mount():
    return _load()['mount']


@pytest.fixture
def cfg():
    return _load()


# ── Translation sanity ────────────────────────────────────────────────────────

class TestTranslationSanity:
    def test_z_filled_in(self, mount):
        """z must not be exactly 0 — that means nobody measured it."""
        assert mount['z'] != 0.0, (
            "mount.z is 0 — measure the flange-to-lens distance with calipers "
            "(typically 4–8 cm for D435 bracket mounts)")

    def test_z_in_physical_range(self, mount):
        z = float(mount['z'])
        assert _Z_MIN <= z <= _Z_MAX, (
            f"mount.z={z:.3f}m is outside [{_Z_MIN}, {_Z_MAX}] m — "
            "check measurement (expected ~0.04–0.08 m for a wrist-mounted D435)")

    def test_x_in_physical_range(self, mount):
        x = float(mount['x'])
        assert abs(x) <= _XY_MAX, (
            f"mount.x={x:.3f}m exceeds ±{_XY_MAX}m — "
            "lateral offset this large is unusual, double-check measurement")

    def test_y_in_physical_range(self, mount):
        y = float(mount['y'])
        assert abs(y) <= _XY_MAX, (
            f"mount.y={y:.3f}m exceeds ±{_XY_MAX}m — "
            "vertical offset this large is unusual, double-check measurement")


# ── Rotation sanity (RPY path) ────────────────────────────────────────────────

class TestRPYSanity:
    def test_roll_in_range(self, mount, cfg):
        if cfg.get('calibrated'):
            pytest.skip('calibrated=true — RPY not used')
        assert abs(float(mount['roll'])) <= _ANGLE_MAX, \
            f"mount.roll={mount['roll']} rad is outside ±π"

    def test_pitch_in_range(self, mount, cfg):
        if cfg.get('calibrated'):
            pytest.skip('calibrated=true — RPY not used')
        assert abs(float(mount['pitch'])) <= _ANGLE_MAX, \
            f"mount.pitch={mount['pitch']} rad is outside ±π"

    def test_yaw_in_range(self, mount, cfg):
        if cfg.get('calibrated'):
            pytest.skip('calibrated=true — RPY not used')
        assert abs(float(mount['yaw'])) <= _ANGLE_MAX, \
            f"mount.yaw={mount['yaw']} rad is outside ±π"

    def test_rpy_not_all_placeholder(self, mount, cfg):
        """Warn if roll/pitch/yaw are all zero — likely untouched placeholders.
        Not a hard failure: all-zero RPY is valid if the camera is mounted straight."""
        if cfg.get('calibrated'):
            pytest.skip('calibrated=true — RPY not used')
        r = float(mount['roll'])
        p = float(mount['pitch'])
        y = float(mount['yaw'])
        if r == 0.0 and p == 0.0 and y == 0.0:
            pytest.xfail(
                "roll/pitch/yaw are all 0 — acceptable only if the camera is mounted "
                "perfectly parallel to tool0 (no tilt). Verify physically before launching.")


# ── Quaternion sanity (calibrated path) ──────────────────────────────────────

class TestQuaternionSanity:
    def test_quaternion_unit_norm(self, mount, cfg):
        """Quaternion from easy_handeye2 must be unit-length."""
        if not cfg.get('calibrated'):
            pytest.skip('calibrated=false — quaternion not used')
        qx, qy, qz, qw = (float(mount[k]) for k in ['qx', 'qy', 'qz', 'qw'])
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert abs(norm - 1.0) < 1e-4, (
            f"Quaternion norm={norm:.6f} — not unit length. "
            "Paste values directly from easy_handeye2 output without rounding.")

    def test_quaternion_not_identity_when_calibrated(self, mount, cfg):
        """After real calibration the quaternion should not be the default identity."""
        if not cfg.get('calibrated'):
            pytest.skip('calibrated=false — quaternion not used')
        qx, qy, qz, qw = (float(mount[k]) for k in ['qx', 'qy', 'qz', 'qw'])
        is_identity = (abs(qx) < 1e-9 and abs(qy) < 1e-9
                       and abs(qz) < 1e-9 and abs(qw - 1.0) < 1e-9)
        assert not is_identity, (
            "Quaternion is still identity (0,0,0,1) but calibrated=true — "
            "paste the actual values from easy_handeye2 output")

    def test_qw_not_near_zero(self, mount, cfg):
        """qw ≈ 0 means ~180° rotation — physically impossible for a wrist-mounted camera."""
        if not cfg.get('calibrated'):
            pytest.skip('calibrated=false — quaternion not used')
        qw = float(mount['qw'])
        assert abs(qw) > _QW_MIN, (
            f"qw={qw:.6f} is near zero (≈180° rotation) — check easy_handeye2 output")


# ── Overall readiness check ───────────────────────────────────────────────────

class TestReadiness:
    def test_calibration_mode_is_explicit(self, cfg):
        """The calibrated flag must be a bool, not a string or None."""
        val = cfg.get('calibrated')
        assert isinstance(val, bool), (
            f"'calibrated' in camera_params.yaml must be true or false (bool), got {val!r}")

    def test_z_is_the_only_nonzero_translation_on_straight_mount(self, mount, cfg):
        """If roll/pitch/yaw are all 0, x and y should also be near 0
        (a camera pointed straight forward sits on the Z axis of tool0).
        This is a warning-level check — fails soft (xfail)."""
        if cfg.get('calibrated'):
            pytest.skip('calibrated=true — skip heuristic check')
        r = float(mount['roll'])
        p = float(mount['pitch'])
        y_angle = float(mount['yaw'])
        x = float(mount['x'])
        y = float(mount['y'])
        if r == 0.0 and p == 0.0 and y_angle == 0.0:
            if abs(x) > 0.02 or abs(y) > 0.02:
                pytest.xfail(
                    f"mount.x={x:.3f}, y={y:.3f} are non-trivial but rotation is 0 — "
                    "verify: is the camera bracket off-centre?")
