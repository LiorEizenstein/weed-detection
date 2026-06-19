# Robot Deployment Checklist — UR5 + RealSense D435

## 1. Install ROS packages on the robot PC

```bash
sudo apt install ros-jazzy-ur-robot-driver \
                 ros-jazzy-realsense2-camera \
                 ros-jazzy-image-view \
                 ros-jazzy-aruco-opencv   # needed for hand-eye calibration
```

## 2. Copy / build the project

Since you are copying from your dev machine (not cloning from git):

```bash
# On the robot PC:
mkdir -p ~/ros2_ws/src
# Copy the watermelon_demo package from your dev machine, then:
cd ~/ros2_ws
colcon build --packages-select watermelon_demo
source install/setup.bash
```

## 3. Camera calibration (easy_handeye2) — recommended before demo

This gives you the exact `tool0 → camera_link` transform automatically.
Much more accurate than measuring with calipers.

### 3a. Install easy_handeye2

```bash
cd ~/ros2_ws
git clone https://github.com/marcoesposito1988/easy_handeye2 src/easy_handeye2
rosdep install -iyr --from-paths src
colcon build --packages-select easy_handeye2
source install/setup.bash
```

### 3b. Print and place the ArUco marker

Print an ArUco marker (ID 0, dictionary DICT_4X4_50) on A4 paper.
Place it on the ground/table in front of the robot — it must be visible
from multiple arm poses.

```bash
# Generate marker image:
python3 -c "
import cv2, cv2.aruco as aruco
d = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = aruco.generateImageMarker(d, 0, 200)
cv2.imwrite('/tmp/aruco_marker.png', img)
print('Saved to /tmp/aruco_marker.png')
"
```

### 3c. Run the calibration

```bash
# Terminal 1 — start the UR5 driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 robot_ip:=<ur5_ip>

# Terminal 2 — start RealSense camera
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=false

# Terminal 3 — run easy_handeye2 (eye-in-hand: camera on end-effector)
ros2 launch easy_handeye2 calibrate.launch.py \
    eye_on_hand:=true \
    robot_base_frame:=base \
    robot_effector_frame:=tool0 \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=aruco_marker_frame
```

Move the arm to 8–12 different poses (GUI guides you), then click **Compute**.

### 3d. Paste result into camera_params.yaml

The calibration output looks like:
```
x: 0.034  y: -0.012  z: 0.058
qx: 0.012  qy: -0.706  qz: 0.008  qw: 0.708
```

Open `src/watermelon_demo/config/camera_params.yaml` and fill in:

```yaml
calibrated: true        # ← change this to true

mount:
  x:  0.034             # ← paste your values
  y: -0.012
  z:  0.058
  roll:  0.0            # ignored when calibrated: true
  pitch: 0.0
  yaw:   0.0
  qx:  0.012            # ← paste quaternion from easy_handeye2
  qy: -0.706
  qz:  0.008
  qw:  0.708
```

Then rebuild:
```bash
cd ~/ros2_ws && colcon build --packages-select watermelon_demo && source install/setup.bash
```

### 3e. (Alternative) Manual measurement — no calibration tool

If you don't have time for calibration, fill in only x/y/z from calipers
and leave `calibrated: false`. The centering loop still works; only the
RViz beam arrow and world-space dead-zone will be slightly off.

```yaml
calibrated: false
mount:
  x:  0.00   # lateral offset left/right (m)
  y:  0.00   # up/down offset (m)
  z:  0.05   # flange → lens distance, typically 4–8 cm
  roll:  0.0
  pitch: 0.0
  yaw:   0.0
```

## 4. YOLO model (optional)

Place `best.pt` at `/home/lior/best.pt`.
Without it, add `use_real_model:=false` to fall back to the HSV colour stub.

## 5. Find the robot IP

UR5 teach pendant → Settings → System → Network.
Default in launch file: `192.168.1.100`.

## 6. Launch

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

## 7. Verify tests pass after filling in camera_params.yaml

```bash
cd ~/ros2_ws && source install/setup.bash
python3 -m pytest src/watermelon_demo/test/test_camera_tf.py -v
```

All 10 tests should pass. If `test_camera_z_positive` fails, z is still 0 — measure it.
