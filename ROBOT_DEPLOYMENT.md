# Robot Deployment Checklist — UR5 + RealSense D435

## 1. Install missing ROS packages on the robot PC

```bash
sudo apt install ros-jazzy-ur-robot-driver \
                 ros-jazzy-realsense2-camera \
                 ros-jazzy-image-view
```

## 2. Clone and build

```bash
cd ~/ros2_ws
git clone <repo-url> src/watermelon_demo   # or: cd src/watermelon_demo && git pull
colcon build --packages-select watermelon_demo
source install/setup.bash
```

## 3. Measure camera mount

With the arm at HOME pose, use calipers to measure from the centre of the `tool0`
flange face to the RealSense D435 optical centre, then fill in `launch/demo_real.launch.py`:

```python
CAMERA_X   = 0.00   # lateral offset left/right (m)
CAMERA_Y   = 0.00   # up/down offset (m)
CAMERA_Z   = 0.05   # forward distance flange → lens (m), typically 4–8 cm
CAMERA_ROLL  = 0.0  # tilt (rad)
CAMERA_PITCH = 0.0  # angle toward ground (rad)
CAMERA_YAW   = 0.0  # left/right angle (rad)
```

> These values only affect the RViz laser beam arrow and world-space dead-zone.
> The camera image and centering loop work even with the placeholder values.

## 4. YOLO model (optional)

Place `best.pt` at `/home/lior/best.pt`.
Without it, pass `use_real_model:=false` to fall back to the HSV colour stub.

## 5. Find the robot IP

Check the UR5 teach pendant → Network settings. Default in launch file: `192.168.1.100`.

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
    robot_ip:=<ur5_ip> dry_run:=true
```

A live annotated camera window opens automatically. RViz shows 3D field markers.
