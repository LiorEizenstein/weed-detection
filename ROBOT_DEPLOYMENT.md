# Robot Deployment Checklist — UR5 + RealSense D435

---

## 1. Install ROS packages on the robot PC

```bash
sudo apt install ros-jazzy-ur-robot-driver \
                 ros-jazzy-realsense2-camera \
                 ros-jazzy-image-view \
                 ros-jazzy-aruco-opencv   # needed for easy_handeye2 calibration
```

---

## 2. Copy / build the project

Copy the entire `ros2_ws/src/` directory from your dev machine to the robot PC.
It already includes `watermelon_demo` and the `easy_handeye2` submodule.

```bash
# On the robot PC:
mkdir -p ~/ros2_ws
# (copy src/ from dev machine here)
cd ~/ros2_ws
rosdep install -iyr --from-paths src
colcon build --packages-select watermelon_demo easy_handeye2
source install/setup.bash
```

---

## 3. Camera mount calibration — choose one option

You must tell the system where the RealSense D435 sits relative to the UR5
`tool0` flange. Two options — **Option A is more accurate**.

---

### Option A: easy_handeye2 (recommended — automatic, accurate)

#### A1. Print and place the ArUco marker

Print an ArUco marker (ID 0, dictionary DICT_4X4_50) and place it flat on the
ground in front of the robot where it will be visible from multiple arm poses.

```bash
python3 -c "
import cv2, cv2.aruco as aruco
d = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = aruco.generateImageMarker(d, 0, 200)
cv2.imwrite('/tmp/aruco_marker.png', img)
print('Print: /tmp/aruco_marker.png')
"
```

#### A2. Run the calibration (3 terminals)

```bash
# Terminal 1 — UR5 driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 robot_ip:=<ur5_ip>

# Terminal 2 — RealSense camera
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=false

# Terminal 3 — easy_handeye2 (eye-in-hand: camera on end-effector)
ros2 launch easy_handeye2 calibrate.launch.py \
    eye_on_hand:=true \
    robot_base_frame:=base \
    robot_effector_frame:=tool0 \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=aruco_marker_frame
```

Move the arm to **8–12 different poses** using the GUI, then click **Compute**.

#### A3. Paste result into camera_params.yaml

The output looks like:
```
x: 0.034  y: -0.012  z: 0.058
qx: 0.012  qy: -0.706  qz: 0.008  qw: 0.708
```

Open `src/watermelon_demo/config/camera_params.yaml` and fill in:

```yaml
calibrated: true        # ← set to true

mount:
  x:  0.034             # ← from calibration output
  y: -0.012
  z:  0.058
  roll:  0.0            # ignored when calibrated: true
  pitch: 0.0
  yaw:   0.0
  qx:  0.012            # ← quaternion from easy_handeye2
  qy: -0.706
  qz:  0.008
  qw:  0.708
```

#### A4. Validate and run tests

```bash
cd ~/ros2_ws && source install/setup.bash

# Quick visual check — must print green "✓ All checks passed":
python3 scripts/check_camera_params.py

# Full test suite for calibrated mode (quaternion path):
python3 -m pytest src/watermelon_demo/test/test_manual_calibration.py \
                  src/watermelon_demo/test/test_camera_tf.py -v
```

**Expected results:**
- `TestTranslationSanity` — 4 passed
- `TestRPYSanity` — 4 skipped (calibrated=true, RPY not used)
- `TestQuaternionSanity` — 3 passed (unit-norm, not identity, qw≠0)
- `TestCalibrationModeSwitch::test_calibrated_mode_uses_quaternion_args` — passed
- `TestCameraParamsYaml` — 5 passed

Then rebuild so the launch file picks up the new values:
```bash
colcon build --packages-select watermelon_demo && source install/setup.bash
```

---

### Option B: Manual measurement with calipers (fallback — no calibration tool needed)

Use this if easy_handeye2 can't run (no MoveIt, no marker detection, time pressure).
The centering loop still works correctly. Only the RViz laser beam arrow and the
world-space dead-zone suppression will have small errors proportional to measurement accuracy.

#### B1. What to measure (arm at HOME pose)

Use calipers or a ruler. The UR5 `tool0` frame has **+Z pointing out of the flange face**.

| Value | What to measure | Typical range |
|---|---|---|
| `z` | Distance from flange face centre to camera lens | 0.04 – 0.08 m |
| `x` | Lateral offset (left/right) from flange centre | ±0.05 m |
| `y` | Vertical offset (up/down) from flange centre | ±0.05 m |
| `pitch` | Downward tilt angle of camera (rad) | 0 if mounted straight |
| `roll` | Side tilt (rad) | 0 if mounted straight |
| `yaw` | Left/right rotation (rad) | 0 if mounted straight |

> **Angles are in radians.** To convert: `radians = degrees × π/180`.
> Example: 10° downward tilt = 0.175 rad.

#### B2. Fill in camera_params.yaml

Open `src/watermelon_demo/config/camera_params.yaml`:

```yaml
calibrated: false       # ← keep false for manual measurement

mount:
  x:  0.00    # ← measured lateral offset (m)
  y:  0.00    # ← measured vertical offset (m)
  z:  0.06    # ← measured flange-to-lens distance (m)  ← MUST NOT BE 0
  roll:  0.00 # ← measured (rad), 0 if straight
  pitch: 0.00 # ← measured (rad), 0 if straight
  yaw:   0.00 # ← measured (rad), 0 if straight
  qx: 0.0     # leave as-is (unused when calibrated: false)
  qy: 0.0
  qz: 0.0
  qw: 1.0
```

#### B3. Validate and run tests

```bash
cd ~/ros2_ws && source install/setup.bash

# Quick visual check — must print green "✓ All checks passed":
python3 scripts/check_camera_params.py

# Full test suite for manual mode (RPY path):
python3 -m pytest src/watermelon_demo/test/test_manual_calibration.py \
                  src/watermelon_demo/test/test_camera_tf.py -v
```

**Expected results:**
- `TestTranslationSanity` — 4 passed (z≠0, z in range, x/y in range)
- `TestRPYSanity` — 3 passed + 1 xfail (xfail = warning that RPY is all-zero, safe to ignore if camera is straight)
- `TestQuaternionSanity` — 3 skipped (calibrated=false, quaternion not used)
- `TestCalibrationModeSwitch::test_uncalibrated_mode_uses_rpy_args` — passed
- `TestCameraParamsYaml` — 5 passed

**If `test_z_filled_in` fails:** `z` is still 0 — go back and measure the flange-to-lens distance.

**If `test_rpy_not_all_placeholder` is XFAIL:** all angles are 0. This is fine if the camera is mounted perfectly straight. If it's visibly tilted, measure the tilt angle.

Then rebuild:
```bash
colcon build --packages-select watermelon_demo && source install/setup.bash
```

---

## 4. Run all camera tests at once

After filling in `camera_params.yaml` (either option), run the full camera test suite:

```bash
cd ~/ros2_ws && source install/setup.bash
python3 -m pytest \
    src/watermelon_demo/test/test_camera_tf.py \
    src/watermelon_demo/test/test_manual_calibration.py \
    src/watermelon_demo/test/test_camera_intrinsics.py \
    -v
```

All tests should pass (or skip/xfail as noted above). No hard failures allowed before launching.

---

## 5. YOLO model (optional)

Place `best.pt` at `/home/lior/best.pt`.
Without it, add `use_real_model:=false` to fall back to the HSV colour stub.

---

## 6. Find the robot IP

UR5 teach pendant → Settings → System → Network.
Default in launch file: `192.168.1.100`.

---

## 7. Launch

```bash
# Detection-only (arm centres over weeds, never fires) — safe default:
ros2 launch watermelon_demo demo_real.launch.py \
    robot_ip:=<ur5_ip> dry_run:=true

# Full pipeline (arm centres and publishes /laser_fire signal):
ros2 launch watermelon_demo demo_real.launch.py \
    robot_ip:=<ur5_ip> dry_run:=false

# Without YOLO model:
ros2 launch watermelon_demo demo_real.launch.py \
    robot_ip:=<ur5_ip> dry_run:=true use_real_model:=false
```

A live annotated camera window opens automatically. RViz shows 3D field markers.
