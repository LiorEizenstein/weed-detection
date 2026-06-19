"""
test_centering.py — unit-level simulation of the MOVE_TO_WEED centering loop.

Runs the centering state-machine logic against a calibrated linear kinematic
model of the arm and verifies pixel error converges to the fire zone.

World model (X/Y coupling measured from logs; wrist_2 effect is the assumed
perpendicular axis to wrist_1 — to be recalibrated after next demo run):

    Δerr_x ≈  delta_pan * C_PAN_X  – delta_w2 * C_W2_X_CROSS
    Δerr_y ≈ –delta_w2  * C_W2_Y   + delta_pan * C_PAN_Y_CROSS

    C_PAN_X       = 2.00  — shoulder_pan → image X (measured: ~2.65/rad from log)
    C_PAN_Y_CROSS = 0.10  — pan → Y cross-coupling
    C_W2_Y        = 1.50  — wrist_2 → image Y (assumed; wrist_1 measured ≈0 for Y)
    C_W2_X_CROSS  = 0.05  — wrist_2 → X cross-coupling (small, perpendicular axis)
    C_JY_X_OLD    = 0.40  — shoulder_lift → X cross-coupling (OLD algo, kept for regression)

Key finding from run logs: wrist_1 at wrist_2=-π/2 acts as a 2nd pan joint
(moves image X ~2.65/rad but Y ~0/rad). wrist_2 is the correct elevation axis.

Run:
    cd ~/ros2_ws
    pytest src/watermelon_demo/test/test_centering.py -v
"""

import math
import pytest

# ── pull live constants from the module under test ──────────────────────────
import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
WRIST1_GAIN    = _mod.WRIST1_GAIN
WRIST2_GAIN    = _mod.WRIST2_GAIN
FIRE_ZONE_FRAC = _mod.FIRE_ZONE_FRAC

# ── world-model coefficients ─────────────────────────────────────────────────
# Measured from logs (run_20260613_085200):
#   wrist_1 +0.082 rad/step → ΔX ≈ +0.22  (C_W1_X ≈ 2.68), ΔY ≈ +0.01 (tiny)
#   wrist_2 -0.084 rad/step → ΔY ≈ -0.15  (C_W2_Y ≈ 1.79), ΔX ≈  0.00 (none)
# The two joints are nearly orthogonal → simultaneous correction is stable.
C_W1_X        = 2.68   # wrist_1 → image X (measured); dw1>0 → X increases
C_W1_Y_CROSS  = 0.12   # wrist_1 → image Y cross-coupling (small)
C_W2_Y        = 1.79   # wrist_2 → image Y (measured); dw2<0 → Y decreases
C_W2_X_CROSS  = 0.05   # wrist_2 → image X cross-coupling (negligible)

# OLD algorithm (shoulder_lift + pan simultaneously):
C_JY_Y        = 0.80   # shoulder_lift → image Y
C_JY_X_OLD    = 0.40   # shoulder_lift → image X cross-coupling (large → diverges)

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

def world_step(err_x, err_y, dw1, dw2):
    """Dual-wrist world model (new algorithm).
    dw1 < 0 for positive err_x (wrist_1 corrects X); dw2 < 0 for positive err_y."""
    new_x = err_x + dw1 * C_W1_X   + dw2 * C_W2_X_CROSS
    new_y = err_y + dw1 * C_W1_Y_CROSS + dw2 * C_W2_Y
    return new_x, new_y


def world_step_old(err_x, err_y, dpan, dlift, c_lift_x_cross):
    """Shoulder_lift + pan world model (old algorithm).
    Large X cross-coupling from shoulder_lift causes runaway."""
    C_PAN_X       = 2.00
    C_PAN_Y_CROSS = 0.10
    new_x = err_x + dpan * C_PAN_X - dlift * c_lift_x_cross
    new_y = err_y - dlift * C_JY_Y + dpan * C_PAN_Y_CROSS
    return new_x, new_y


# ── NEW algorithm (dual-axis: wrist_1 for X, wrist_2 for Y) ──────────────────

def _new_centering_step(err_x, err_y, w1, w2):
    """One tick. Returns updated (w1, w2, dw1, dw2)."""
    old_w1, old_w2 = w1, w2
    w1 -= err_x * WRIST1_GAIN   # dw1 < 0 for weed-right → X decreases
    w2 -= err_y * WRIST2_GAIN   # dw2 < 0 for weed-below → Y decreases
    w1 = max(-2.5, min(-0.8, w1))
    w2 = max(-2.5, min(-0.5, w2))
    return w1, w2, w1 - old_w1, w2 - old_w2


def run_new(cx, cy, w1_0=-1.80, w2_0=-1.57):
    """Simulate the new centering loop. Returns (converged, steps, history)."""
    err_x, err_y = _pix_to_err(cx, cy)
    w1, w2 = w1_0, w2_0
    history = [{'step': 0, 'err_x': err_x, 'err_y': err_y,
                'mag': math.hypot(err_x, err_y), 'w1': w1, 'w2': w2}]

    for step in range(1, MAX_STEPS + 1):
        if math.hypot(err_x, err_y) < FIRE_ZONE_FRAC:
            return True, step - 1, history
        w1, w2, dw1, dw2 = _new_centering_step(err_x, err_y, w1, w2)
        err_x, err_y = world_step(err_x, err_y, dw1, dw2)
        history.append({'step': step, 'err_x': err_x, 'err_y': err_y,
                        'mag': math.hypot(err_x, err_y), 'w1': w1, 'w2': w2})

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
        if math.hypot(err_x, err_y) < FIRE_ZONE_FRAC:
            return True, step - 1, history
        dpan  = -err_x * OLD_PAN_GAIN
        dlift = -err_y * OLD_LIFT_GAIN
        err_x, err_y = world_step_old(err_x, err_y, dpan, dlift, C_JY_X_OLD)
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

class TestDualAxisCorrection:
    """Verify the dual-axis wrist_1+wrist_2 correction directions and ranges."""

    def test_both_axes_corrected_each_step(self):
        """Both wrist_1 and wrist_2 move every step (dual-axis, not single)."""
        w1, w2, dw1, dw2 = _new_centering_step(0.50, 0.70, -1.80, -1.57)
        assert dw1 != 0.0, "wrist_1 should move when err_x != 0"
        assert dw2 != 0.0, "wrist_2 should move when err_y != 0"

    def test_wrist1_direction_positive_x(self):
        """Positive err_x (weed right) → wrist_1 decreases."""
        w1, _, dw1, _ = _new_centering_step(0.50, 0.0, -1.80, -1.57)
        assert dw1 < 0, f"Expected wrist_1 to decrease for positive X error, got {dw1}"

    def test_wrist1_direction_negative_x(self):
        """Negative err_x (weed left) → wrist_1 increases."""
        w1, _, dw1, _ = _new_centering_step(-0.50, 0.0, -1.80, -1.57)
        assert dw1 > 0, f"Expected wrist_1 to increase for negative X error, got {dw1}"

    def test_wrist2_direction_positive_y(self):
        """Positive err_y (weed below centre) → wrist_2 decreases."""
        _, w2, _, dw2 = _new_centering_step(0.0, 0.70, -1.80, -1.57)
        assert dw2 < 0, f"Expected wrist_2 to decrease for positive Y error, got {dw2}"

    def test_wrist2_direction_negative_y(self):
        """Negative err_y (weed above centre) → wrist_2 increases."""
        _, w2, _, dw2 = _new_centering_step(0.0, -0.70, -1.80, -1.57)
        assert dw2 > 0, f"Expected wrist_2 to increase for negative Y error, got {dw2}"

    def test_wrist1_clamped(self):
        """wrist_1 must stay within [-2.5, -0.8]."""
        for err_x in [1.0, -1.0, 5.0, -5.0]:
            w1_out, _, _, _ = _new_centering_step(err_x, 0.0, -2.5, -1.57)
            assert -2.5 <= w1_out <= -0.8, f"wrist_1={w1_out} out of range"
            w1_out, _, _, _ = _new_centering_step(err_x, 0.0, -0.8, -1.57)
            assert -2.5 <= w1_out <= -0.8, f"wrist_1={w1_out} out of range"

    def test_wrist2_clamped(self):
        """wrist_2 must stay within [-2.5, -0.5]."""
        for err_y in [1.0, -1.0, 5.0, -5.0]:
            _, w2_out, _, _ = _new_centering_step(0.0, err_y, -1.80, -2.5)
            assert -2.5 <= w2_out <= -0.5, f"wrist_2={w2_out} out of range"
            _, w2_out, _, _ = _new_centering_step(0.0, err_y, -1.80, -0.5)
            assert -2.5 <= w2_out <= -0.5, f"wrist_2={w2_out} out of range"
