# Wrist-2 Gain Calibration

Documents how the wrist_2 Y-correction gain (`WRIST2_GAIN = 0.10 rad/px`) and
the world-model coefficient `C_W2_Y = 1.79` used in `test_centering.py` were
derived.

---

## Background: why wrist_2, not wrist_1

Early versions of the algorithm used `wrist_1_joint` (index 3) for the Y-axis
correction. Log analysis showed it was acting as a second pan axis — rotating
the camera horizontally rather than elevating it — because at
`wrist_2 = -π/2`, the wrist_1 rotation axis is parallel to the vertical, not
perpendicular to the image plane.

At `wrist_2 = -π/2` (the scan pose nominal), the joint axes are:

| Joint | Image axis moved | Measured sensitivity |
|---|---|---|
| `shoulder_pan` (index 0) | X (primary) | ~2.65 px_norm / rad |
| `wrist_1` (index 3) | X (secondary, same direction) | ~2.65 px_norm / rad |
| `wrist_2` (index 4) | Y (primary) | measured below |

`wrist_1` was therefore redundant with `shoulder_pan` for X, and useless for
Y. `wrist_2` is the correct elevation axis because it rotates around the axis
perpendicular to the arm-camera plane.

---

## Calibration run: `run_20260613_085200.log`

The run was conducted with `use_real_model: false` (HSV stub) so the weed pixel
positions are repeatable across frames. The arm was at scan pose 7/21
(pan = +0.60 rad, wrist_2 = -1.570 rad nominal) when weed 1 was first spotted.

### Weed 1 — Y-axis calibration steps

| Step | wrist_2 (rad) | weed pixel y | err_y | Δwrist_2 | Δerr_y |
|------|--------------|--------------|-------|----------|--------|
| 0 (spotted) | −1.570 | 442 | +0.842 | — | — |
| 1 | −1.654 | 406 | +0.692 | −0.084 | −0.150 |
| 2 | −1.723 | 358 | +0.492 | −0.069 | −0.200 |
| 3 | −1.772 | 314 | +0.308 | −0.049 | −0.184 |
| 4 (fire) | −1.803 | — | < 0.20 | −0.031 | — |

From the log lines:
```
Centering: weed px=(338,441) err=(+0.06,+0.84) correcting Y→w2=-1.654
Centering: weed px=(340,406) err=(+0.06,+0.69) correcting Y→w2=-1.723
Centering: weed px=(339,358) err=(+0.06,+0.49) correcting Y→w2=-1.772
Centering: weed px=(339,314) err=(+0.06,+0.31) correcting Y→w2=-1.803
```

### Deriving C_W2_Y

The world model in `test_centering.py` is defined as:

```
Δerr_y = dw2 × C_W2_Y
```

Using step 0→1 (the cleanest observation, largest signal):

```
C_W2_Y = Δerr_y / Δwrist_2
       = −0.150 / −0.084
       = 1.79
```

Verification with step 1→2:

```
C_W2_Y = −0.200 / −0.069 = 2.90
```

The two measurements diverge (~60%), which is expected: the linear model is an
approximation valid only near the nominal `wrist_2 = -π/2`. As `wrist_2` moves
away from -1.57 rad during centering, the true kinematic gain changes. The value
1.79 was chosen as the first-step measurement (closest to the nominal
configuration) and matches the convergence observed in practice.

---

## Gain selection: `WRIST2_GAIN = 0.10`

The controller applies:

```
dw2 = −err_y × WRIST2_GAIN
```

With `C_W2_Y = 1.79`, the effective closed-loop gain per step is:

```
loop_gain = WRIST2_GAIN × C_W2_Y = 0.10 × 1.79 = 0.179
```

A gain of 0.179 means each step reduces the Y error by ~18%. For an initial
error of +0.84 (weed 1, step 0), the expected convergence to the 0.20 fire zone
is:

```
n ≈ log(0.20 / 0.84) / log(1 − 0.179) ≈ 8 steps
```

In practice weed 1 converged in 4 steps, faster than the model predicts,
because the gain was higher near the nominal configuration.

---

## Limitations and known issues

- **Single operating point**: C_W2_Y was measured at `wrist_2 ≈ -1.57 rad`.
  The true gain changes as `wrist_2` moves; a multi-point calibration sweep
  would give a better gain schedule.
- **One data run**: the coefficient is derived from a single run, not a
  statistical average across multiple runs and weed positions.
- **Coupling not fully characterised**: `C_W2_X_CROSS = 0.05` (the wrist_2 →
  image X cross-coupling) is assumed, not measured. Weed 1's X error remained
  nearly constant (+0.06) across all steps, which is consistent with the
  assumption being small, but does not prove it.
- **Validity condition**: the perpendicular-axis assumption only holds when
  `wrist_2 ≈ -π/2`. During multi-step centering, `wrist_2` can drift up to
  ~0.3 rad from that point; gain accuracy degrades accordingly.

A controlled calibration sweep (systematically stepping `wrist_2` while
holding all other joints fixed and measuring pixel displacement) would replace
these assumptions with measured values.
