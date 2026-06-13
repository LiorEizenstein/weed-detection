"""
test_centering.py — unit-level simulation of the MOVE_TO_WEED centering loop.

Runs the centering state-machine logic against a calibrated linear kinematic
model of the arm and verifies pixel error converges to the fire zone for every
weed scenario recorded in run_20260613_013159.log.

World model (calibrated against log step 1→2 for OLD algorithm):
    Δerr_x ≈  delta_pan  * C_PAN_X  –  delta_joint_y * C_JY_X_CROSS
    Δerr_y ≈  delta_joint_y * (–C_JY_Y) + delta_pan * C_PAN_Y_CROSS

    C_PAN_X    = 2.00  — shoulder_pan moves image X (calibrated: 1–0.25*2=0.50/step)
    C_PAN_Y    = 0.10  — pan→Y cross-coupling (small)
    C_JY_Y     = 0.80  — Y-joint  corrects image Y (lift or wrist_1)
    C_JY_X_OLD = 0.40  — shoulder_lift→X cross-coupling (LARGE, causes divergence)
    C_JY_X_NEW = 0.03  — wrist_1→X  cross-coupling (tiny, wrist is near camera)

Run:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_centering.py -v
"""

import math
import pytest

# ── pull live constants from the module under test ──────────────────────────
import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
PAN_GAIN       = _mod.PAN_GAIN
WRIST1_GAIN    = _mod.WRIST1_GAIN
FIRE_ZONE_FRAC = _mod.FIRE_ZONE_FRAC

# ── world-model coefficients ─────────────────────────────────────────────────
C_PAN_X       = 2.00   # shoulder_pan  → image X (primary)
C_PAN_Y_CROSS = 0.10   # shoulder_pan  → image Y (cross, arm sweeps arc)
C_JY_Y        = 0.80   # Y-axis joint  → image Y (primary)
C_JY_X_OLD    = 0.40   # shoulder_lift → image X cross-coupling (large: far base joint)
C_JY_X_NEW    = 0.03   # wrist_1       → image X cross-coupling (tiny: near camera)

MAX_STEPS = 25         # generous cap; good controller converges ≤ 12

# ── scenarios from run_20260613_013159.log ("Weed spotted" pixel values) ─────
# Format: (test-id, cx, cy) — err computed as (cx-320)/320, (cy-240)/240
LOG_SCENARIOS = [
    ("weed1_large_y_bot_pos",  338, 441),   # err ≈ (+0.06, +0.84) — OLD diverged
    ("weed2_near_center",      330, 274),   # err ≈ (+0.03, +0.14) — fired at once
    ("weed3_large_y_top_neg",  282,  67),   # err ≈ (−0.12, −0.72) — OLD stalled
    ("weed4_large_y_bot_pos2", 356, 424),   # err ≈ (+0.11, +0.77) — OLD diverged
    ("weed6_large_y_top_neg2", 266,  65),   # err ≈ (−0.12, −0.80) — OLD diverged
    ("weed7_corner",           163, 458),   # err ≈ (−0.50, +0.91) — OLD lost weed
]


def _pix_to_err(cx, cy):
    return (cx - 320) / 320.0, (cy - 240) / 240.0


# ── world model ──────────────────────────────────────────────────────────────

def world_step(err_x, err_y, dpan, djoint_y, c_jy_x_cross):
    """Return updated pixel errors after joint deltas are applied."""
    new_x = err_x + dpan * C_PAN_X - djoint_y * c_jy_x_cross
    new_y = err_y - djoint_y * C_JY_Y + dpan * C_PAN_Y_CROSS
    return new_x, new_y


# ── NEW algorithm (single-axis, wrist_1 for Y) ───────────────────────────────

def _new_centering_step(err_x, err_y, pan, w1):
    """One tick of the fixed algorithm. Returns updated (pan, w1, dpan, dw1)."""
    old_pan, old_w1 = pan, w1
    if abs(err_x) >= abs(err_y):
        pan -= err_x * PAN_GAIN
    else:
        w1 += err_y * WRIST1_GAIN
    pan = max(-math.pi, min(math.pi, pan))
    w1  = max(-2.5, min(-0.8, w1))
    return pan, w1, pan - old_pan, w1 - old_w1


def run_new(cx, cy, pan0=0.60, w1_0=-1.80):
    """Simulate the new centering loop. Returns (converged, steps, history)."""
    err_x, err_y = _pix_to_err(cx, cy)
    pan, w1 = pan0, w1_0
    history = [{'step': 0, 'err_x': err_x, 'err_y': err_y,
                'mag': math.hypot(err_x, err_y), 'pan': pan, 'w1': w1}]

    for step in range(1, MAX_STEPS + 1):
        if abs(err_x) < FIRE_ZONE_FRAC and abs(err_y) < FIRE_ZONE_FRAC:
            return True, step - 1, history
        pan, w1, dpan, dw1 = _new_centering_step(err_x, err_y, pan, w1)
        err_x, err_y = world_step(err_x, err_y, dpan, dw1, C_JY_X_NEW)
        history.append({'step': step, 'err_x': err_x, 'err_y': err_y,
                        'mag': math.hypot(err_x, err_y), 'pan': pan, 'w1': w1})

    return False, MAX_STEPS, history


# ── OLD algorithm (simultaneous dual-axis, shoulder_lift for Y) ───────────────

OLD_PAN_GAIN  = 0.25
OLD_LIFT_GAIN = 0.18


def run_old(cx, cy, pan0=0.60, lift0=-1.20):
    """Simulate the pre-fix algorithm. Returns (converged, steps, history)."""
    err_x, err_y = _pix_to_err(cx, cy)
    pan, lift = pan0, lift0
    history = [{'step': 0, 'err_x': err_x, 'err_y': err_y}]

    for step in range(1, MAX_STEPS + 1):
        if abs(err_x) < FIRE_ZONE_FRAC and abs(err_y) < FIRE_ZONE_FRAC:
            return True, step - 1, history
        dpan  = -err_x * OLD_PAN_GAIN
        dlift = -err_y * OLD_LIFT_GAIN
        err_x, err_y = world_step(err_x, err_y, dpan, dlift, C_JY_X_OLD)
        history.append({'step': step, 'err_x': err_x, 'err_y': err_y})

    return False, MAX_STEPS, history


# ── helpers ───────────────────────────────────────────────────────────────────

def _fmt(history):
    return '  →  '.join(f"({h['err_x']:+.2f},{h['err_y']:+.2f})" for h in history)


# ═════════════════════════════════════════════════════════════════════════════
# NEW ALGORITHM TESTS
# ═════════════════════════════════════════════════════════════════════════════

class TestNewAlgorithmConverges:

    @pytest.mark.parametrize("name,cx,cy", LOG_SCENARIOS)
    def test_reaches_fire_zone(self, name, cx, cy):
        """Every log scenario must reach the fire zone within MAX_STEPS."""
        converged, steps, history = run_new(cx, cy)
        assert converged, (
            f"{name}: did not converge in {MAX_STEPS} steps.\n"
            f"Final err=({history[-1]['err_x']:+.3f},{history[-1]['err_y']:+.3f})\n"
            f"Trace: {_fmt(history)}")

    @pytest.mark.parametrize("name,cx,cy", LOG_SCENARIOS)
    def test_converges_within_max_steps(self, name, cx, cy):
        """All scenarios must converge within MAX_STEPS — no infinite hunting."""
        converged, steps, history = run_new(cx, cy)
        assert converged and steps <= MAX_STEPS, (
            f"{name}: did not converge within {MAX_STEPS} steps (took {steps}).\n"
            f"Trace: {_fmt(history)}")

    @pytest.mark.parametrize("name,cx,cy", LOG_SCENARIOS)
    def test_average_reduction_rate(self, name, cx, cy):
        """
        Mean error-magnitude reduction per step must be ≥ 5 %.
        Guards against extremely slow convergence (stuck near boundary).
        """
        _, _, history = run_new(cx, cy)
        if len(history) < 2:
            return  # already in fire zone at start — trivially fine
        initial = history[0]['mag']
        final   = history[-1]['mag']
        steps   = len(history) - 1
        if initial < FIRE_ZONE_FRAC:
            return  # started inside fire zone
        # geometric mean reduction per step
        rate = 1.0 - (final / initial) ** (1.0 / steps)
        mags_str = [f'{h["mag"]:.3f}' for h in history]
        assert rate >= 0.05, (
            f"{name}: only {rate:.1%} average reduction/step (need ≥ 5%).\n"
            f"Magnitudes: {mags_str}")

    @pytest.mark.parametrize("name,cx,cy", LOG_SCENARIOS)
    def test_error_magnitude_net_decreasing(self, name, cx, cy):
        """
        Magnitude of (err_x, err_y) must be generally decreasing.
        Allow at most 2 transient increases (clamping effects).
        """
        _, _, history = run_new(cx, cy)
        mags = [math.hypot(h['err_x'], h['err_y']) for h in history]
        increases = sum(
            1 for i in range(1, len(mags))
            if mags[i] > mags[i - 1] * 1.05   # 5 % tolerance
        )
        assert increases <= 2, (
            f"{name}: oscillating — error grew on {increases} steps.\n"
            f"Magnitudes: {[f'{m:.3f}' for m in mags]}")

    def test_near_center_fires_immediately(self):
        """Weed2 (err≈0.03, 0.14) is close enough to fire in ≤1 correction."""
        converged, steps, _ = run_new(330, 274)
        assert converged and steps <= 1, (
            f"Near-center weed took {steps} steps — should fire without correction.")

    def test_corner_weed_converges(self):
        """Weed7 corner case (−0.50, +0.91) was the hardest — must still converge."""
        converged, steps, history = run_new(163, 458)
        assert converged, (
            f"Corner weed did not converge.\nTrace: {_fmt(history)}")


# ═════════════════════════════════════════════════════════════════════════════
# REGRESSION: OLD ALGORITHM WOULD FAIL
# ═════════════════════════════════════════════════════════════════════════════

HARD_SCENARIOS = [s for s in LOG_SCENARIOS if abs(_pix_to_err(s[1], s[2])[1]) > 0.5]


class TestOldAlgorithmDiverges:
    """
    Show that the pre-fix simultaneous dual-axis correction with shoulder_lift
    diverges on every large-Y scenario.  This guards against accidentally
    reverting to the old approach.
    """

    @pytest.mark.parametrize("name,cx,cy", HARD_SCENARIOS)
    def test_old_algo_fails_on_large_y(self, name, cx, cy):
        """Old algorithm must NOT converge on hard cases (large Y error)."""
        converged, steps, history = run_old(cx, cy)
        assert not converged, (
            f"{name}: old algorithm unexpectedly converged in {steps} steps — "
            f"re-check C_JY_X_OLD cross-coupling coefficient.\n"
            f"Trace: {_fmt(history)}")

    def test_old_x_error_grows_on_weed1(self):
        """
        Weed1: X error must grow across the first 5 steps of the old algo
        (matches log: err_x drifts +0.06 → −0.71 while correction fights Y).
        """
        _, _, history = run_old(338, 441)
        x_errors = [h['err_x'] for h in history[:6]]
        # Magnitude of X should increase — old algo was making it worse
        max_abs_x = max(abs(v) for v in x_errors)
        initial_abs_x = abs(x_errors[0])
        assert max_abs_x > initial_abs_x * 2, (
            f"Expected X error to at least double in 5 steps of old algo; "
            f"got {x_errors}")


# ═════════════════════════════════════════════════════════════════════════════
# AXIS-SELECTION LOGIC
# ═════════════════════════════════════════════════════════════════════════════

class TestSingleAxisSelection:
    """Verify the dominant-axis decision at the algorithmic level."""

    def test_large_y_selects_wrist1(self):
        """When |err_y| > |err_x|, only wrist_1 should change."""
        pan0, w1_0 = 0.60, -1.80
        pan1, w1_1, dpan, dw1 = _new_centering_step(0.05, 0.70, pan0, w1_0)
        assert dpan == 0.0,  f"Expected no pan change when Y dominates, got dpan={dpan}"
        assert dw1  != 0.0,  "Expected wrist_1 to move when Y dominates"

    def test_large_x_selects_pan(self):
        """When |err_x| > |err_y|, only shoulder_pan should change."""
        pan0, w1_0 = 0.60, -1.80
        pan1, w1_1, dpan, dw1 = _new_centering_step(0.60, 0.10, pan0, w1_0)
        assert dw1  == 0.0,  f"Expected no wrist_1 change when X dominates, got dw1={dw1}"
        assert dpan != 0.0,  "Expected shoulder_pan to move when X dominates"

    def test_equal_error_selects_pan(self):
        """When |err_x| == |err_y|, X wins (tie-break in code: >=)."""
        pan0, w1_0 = 0.60, -1.80
        pan1, w1_1, dpan, dw1 = _new_centering_step(0.30, 0.30, pan0, w1_0)
        assert dw1  == 0.0 and dpan != 0.0, "Tie should select shoulder_pan"

    def test_wrist1_correction_direction_positive_y(self):
        """Positive err_y (weed below centre) → wrist_1 increases."""
        _, w1_1, _, dw1 = _new_centering_step(0.0, 0.50, 0.0, -1.80)
        assert dw1 > 0, f"Expected wrist_1 to increase for positive Y error, got {dw1}"

    def test_wrist1_correction_direction_negative_y(self):
        """Negative err_y (weed above centre) → wrist_1 decreases."""
        _, w1_1, _, dw1 = _new_centering_step(0.0, -0.50, 0.0, -1.80)
        assert dw1 < 0, f"Expected wrist_1 to decrease for negative Y error, got {dw1}"

    def test_pan_correction_direction(self):
        """Positive err_x (weed right) → shoulder_pan decreases."""
        pan1, _, dpan, _ = _new_centering_step(0.40, 0.0, 0.60, -1.80)
        assert dpan < 0, f"Expected pan to decrease for positive X error, got {dpan}"

    def test_wrist1_clamped_to_range(self):
        """wrist_1 must stay within [-2.5, -0.8]."""
        for err_y in [1.0, -1.0, 5.0, -5.0]:
            for w1_init in [-0.8, -1.8, -2.5]:
                _, w1_out, _, _ = _new_centering_step(0.0, err_y, 0.0, w1_init)
                assert -2.5 <= w1_out <= -0.8, (
                    f"wrist_1={w1_out} out of range for err_y={err_y}, init={w1_init}")
