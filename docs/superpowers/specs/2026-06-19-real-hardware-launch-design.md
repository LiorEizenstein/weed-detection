# Real Hardware Launch — Design Spec

**Date:** 2026-06-19  
**Status:** Approved

## Goal

Add a `demo_real.launch.py` that runs the watermelon weed-detection pipeline
on a physical UR5 robot with an Intel RealSense D435 wrist-mounted camera,
with no Gazebo dependency. The existing `demo.launch.py` (Gazebo simulation)
is left completely unchanged.

Two operation modes are selectable at launch time via a `dry_run` argument:

| Mode | `dry_run` | Behaviour |
|---|---|---|
| Detection-only | `true` | Arm scans and centres over weeds; never fires laser |
| Full pipeline | `false` | Arm scans, centres, and publishes `/laser_fire True` |

---

## Files Changed / Added

| File | Action |
|---|---|
| `launch/demo_real.launch.py` | **New** — hardware launch file |
| `config/real_params.yaml` | **New** — hardware parameter overrides |
| `watermelon_demo/arm_controller_node.py` | **Edit** — add `dry_run` parameter |
| `watermelon_demo/detection_node.py` | **Edit** — subscribe to `camera_info` for live intrinsics |

---

## Section 1 — `launch/demo_real.launch.py`

### Nodes launched

```
ur_ros2_driver          — connects to UR5 over Ethernet (robot_ip launch arg)
realsense2_camera       — publishes color image + calibrated camera_info
static_transform_publisher  — tool0 → camera_link (mount geometry, see §5)
[8 s TimerAction]
  detection_node        — remapped to /camera/color/image_raw
  arm_controller_node   — real_params.yaml, dry_run forwarded as param
  laser_effect_node     — unchanged
  field_manager_node    — unchanged
  image_view            — live /detection_image window for audience
  rviz2                 — demo_rviz.rviz config
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.1.100` | UR5 controller IP address |
| `dry_run` | `true` | `true` = detection-only, `false` = full pipeline with laser signal |

### Usage

```bash
# Detection-only (no laser signal):
ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=true

# Full pipeline (laser signal published):
ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=false
```

### Topic remapping

`detection_node` remaps its subscription from `/camera/image_raw`
(Gazebo convention) to `/camera/color/image_raw` (RealSense convention).

---

## Section 2 — `config/real_params.yaml`

```yaml
arm_controller_node:
  ros__parameters:
    scan_dwell_time: 1.0
    laser_fire_duration: 1.0
    dry_run: false          # overridden by launch arg
    use_sim_time: false

detection_node:
  ros__parameters:
    use_real_model: true
    model_path: '/home/lior/best.pt'
    save_debug_frames: false
    use_sim_time: false

laser_effect_node:
  ros__parameters:
    use_sim_time: false

field_manager_node:
  ros__parameters:
    use_sim_time: false
```

---

## Section 3 — `arm_controller_node.py` — `dry_run` parameter

### Change

Declare a new ROS parameter `dry_run` (bool, default `false`).

In the `FIRE_LASER` state:

```python
if self.get_parameter('dry_run').value:
    self.get_logger().info(
        'DRY RUN: weed centred — would fire laser here')
    self._state = State.SCAN_MOVE
else:
    # existing fire logic unchanged
    self._fire_pub.publish(Bool(data=True))
    self._fire_start = self._now()
    self._state = State.FIRING
```

When `dry_run=true` the `FIRING` state is never entered. The arm centres over
the weed, logs clearly, and immediately resumes scanning. All dead-zone
suppression (blacklisting the pan angle, recording world position) is skipped
since no treatment occurred.

---

## Section 4 — `detection_node.py` — live camera intrinsics

### Problem

`laser_effect_node` currently uses hardcoded intrinsics (`FX`, `FY`, `CX`,
`CY`) derived from the Gazebo camera plugin parameters. A real RealSense D435
has different focal lengths and publishes accurate calibrated values on
`/camera/color/camera_info`.

### Change

`detection_node` subscribes to `/camera/color/camera_info`
(`sensor_msgs/CameraInfo`) and stores the latest `K` matrix (3×3 camera
intrinsic matrix). It then republishes a `std_msgs/Float64MultiArray` on a new
topic `/camera/intrinsics` containing `[fx, fy, cx, cy]`.

`laser_effect_node` subscribes to `/camera/intrinsics` and uses those values
for ray-casting instead of the hardcoded constants. When no intrinsics message
has been received yet, it falls back to the existing hardcoded values and logs
a warning.

This keeps the Gazebo simulation working (no `camera_info` bridge needed — it
falls back to hardcoded values) and gives the real hardware accurate intrinsics
automatically.

---

## Section 5 — Camera mount static TF

The RealSense D435 is mounted on the UR5 `tool0` flange. The static transform
`tool0 → camera_link` must encode the physical offset and rotation of the
mount.

### Placeholder values (must be measured on the physical robot)

```python
# In demo_real.launch.py — MEASURE THESE FROM THE PHYSICAL MOUNT:
# x, y, z: translation from tool0 origin to camera optical centre (metres)
# roll, pitch, yaw: rotation from tool0 frame to camera_link frame (radians)
CAMERA_X   = 0.00   # TODO: measure
CAMERA_Y   = 0.00   # TODO: measure
CAMERA_Z   = 0.05   # TODO: measure (approximate flange-to-lens distance)
CAMERA_ROLL  = 0.0  # TODO: measure
CAMERA_PITCH = 0.0  # TODO: measure
CAMERA_YAW   = 0.0  # TODO: measure
```

The launch file emits a clear `TODO` comment so it is impossible to miss.

### How to measure

With the arm at HOME_POSE, use a ruler/calipers to measure the offset from the
centre of the tool0 flange face to the camera optical centre. The RealSense
D435 optical frame convention is: +Z forward, +X right, +Y down (ROS optical
convention). The UR5 tool0 frame is: +Z out of the flange, +X forward.

---

## Tests Per Stage

Each implementation stage has a corresponding test that can be run without
the physical robot present (offline / CI-friendly).

### Stage 1 — `dry_run` parameter logic (`arm_controller_node`)

**File:** `test/test_dry_run.py`

| Test | What it checks |
|---|---|
| `test_dry_run_skips_fire_laser` | When `dry_run=true` and state reaches `FIRE_LASER`, state transitions to `SCAN_MOVE` without publishing `/laser_fire` |
| `test_dry_run_false_publishes_fire` | When `dry_run=false`, state machine enters `FIRING` and publishes `Bool(data=True)` |
| `test_dry_run_no_blacklist` | When `dry_run=true`, `_treated_pans` and `_treated_world_xy` remain empty after a would-be fire |

These are unit tests using `importlib` to instantiate the node with a mock
publisher (same pattern as existing `test_centering.py`).

### Stage 2 — live intrinsics pipeline (`detection_node`)

**File:** `test/test_camera_intrinsics.py`

| Test | What it checks |
|---|---|
| `test_intrinsics_published_from_camera_info` | Given a `CameraInfo` message with known `K` matrix, `/camera/intrinsics` publishes `[fx, fy, cx, cy]` matching `K[0,0]`, `K[1,1]`, `K[0,2]`, `K[1,2]` |
| `test_intrinsics_fallback_when_no_camera_info` | If no `CameraInfo` received, `laser_effect_node` logs a warning and uses hardcoded fallback values without crashing |
| `test_intrinsics_update_on_new_camera_info` | A second `CameraInfo` with different values updates the published intrinsics |

### Stage 3 — static TF mount (launch-level)

**File:** `test/test_camera_tf.py`

| Test | What it checks |
|---|---|
| `test_camera_tf_published` | Launch `demo_real.launch.py` in dry-run mode (no robot, no camera — nodes allowed to fail); check that `tf2` reports a transform from `tool0` to `camera_link` within 5 s |
| `test_camera_tf_values` | The published TF matches the constants defined in `demo_real.launch.py` to within 1 mm / 0.001 rad |

These are `launch_testing` tests that bring up only the static TF publisher in
isolation, not the full stack.

### Stage 4 — end-to-end dry run (no robot required)

**File:** `test/test_dry_run_integration.py`

| Test | What it checks |
|---|---|
| `test_no_laser_fire_published_in_dry_run` | Publish a synthetic `Detection2DArray` weed detection into a running node with `dry_run=true`; confirm `/laser_fire` is never published during a 10 s window |
| `test_detection_image_published` | `detection_node` with a synthetic image input publishes `/detection_image` with bounding box annotation within 2 s |

---

## Out of Scope

- Real laser device integration (no hardware laser available)
- RealSense depth stream usage (colour image only)
- Auto-detection of robot IP
- HSV stub on real hardware (YOLO required; `use_real_model` forced to `true`)
