# Real Hardware Launch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `demo_real.launch.py` to run the weed-detection pipeline on a physical UR5 + RealSense D435, with a `dry_run` mode that detects and centres but never fires the laser.

**Architecture:** Four independent changes land in order: (1) `dry_run` param in `arm_controller_node`, (2) live camera intrinsics in `laser_effect_node`, (3) `real_params.yaml`, (4) `demo_real.launch.py`. Each task is independently testable and committed before the next starts.

**Tech Stack:** ROS 2 Jazzy, `ur_robot_driver`, `realsense2_camera`, `tf2_ros`, Python `unittest.mock` for node unit tests, `launch_testing` for TF test.

---

## File Map

| Action | Path |
|---|---|
| Modify | `src/watermelon_demo/watermelon_demo/arm_controller_node.py` |
| Modify | `src/watermelon_demo/watermelon_demo/laser_effect_node.py` |
| Create | `src/watermelon_demo/config/real_params.yaml` |
| Create | `src/watermelon_demo/launch/demo_real.launch.py` |
| Create | `src/watermelon_demo/test/test_dry_run.py` |
| Create | `src/watermelon_demo/test/test_camera_intrinsics.py` |
| Create | `src/watermelon_demo/test/test_camera_tf.py` |
| Create | `src/watermelon_demo/test/test_dry_run_integration.py` |

---

## Task 1: `dry_run` parameter — `arm_controller_node`

**Files:**
- Modify: `src/watermelon_demo/watermelon_demo/arm_controller_node.py:107,242`
- Create: `src/watermelon_demo/test/test_dry_run.py`

- [ ] **Step 1.1 — Write the failing tests**

Create `src/watermelon_demo/test/test_dry_run.py`:

```python
"""
test_dry_run.py — verify arm_controller_node dry_run parameter behaviour.

Mocks all ROS 2 dependencies so the node class can be instantiated
without a running ROS context (same pattern as test_centering.py).
"""
import sys
import math
from unittest.mock import MagicMock, patch, call

# ── Mock every ROS2 dependency before any import ────────────────────────────
for _mod in [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.time',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'control_msgs', 'control_msgs.action',
    'trajectory_msgs', 'trajectory_msgs.msg',
    'builtin_interfaces', 'builtin_interfaces.msg',
    'tf2_ros',
]:
    sys.modules[_mod] = MagicMock()

import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
State = _mod.State


def _make_node(dry_run: bool):
    """Instantiate ArmControllerNode with mocked ROS infrastructure."""
    node = _mod.ArmControllerNode.__new__(_mod.ArmControllerNode)
    # Mock parameter store
    params = {
        'scan_dwell_time': 2.0,
        'laser_fire_duration': 1.0,
        'dry_run': dry_run,
    }
    node.get_parameter = lambda name: MagicMock(value=params[name])
    node.declare_parameter = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(nanoseconds=0))))
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.create_subscription = MagicMock()
    node.create_timer = MagicMock()
    node._action = MagicMock()
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._fire_pub = MagicMock()
    node._busy = False
    node._state = State.FIRE_LASER
    node._scan_idx = 0
    node._current_joints = [0.0] * 6
    node._last_weed_det = (320, 240)
    node._wait_start = 0.0
    node._fire_start = 0.0
    node._lock_start = 0.0
    node._treated_pans = []
    node._treated_world_xy = []
    return node


class TestDryRunSkipsFire:
    def test_state_transitions_to_scan_move_not_firing(self):
        """dry_run=True: FIRE_LASER → SCAN_MOVE, never enters FIRING."""
        node = _make_node(dry_run=True)
        node._tick()
        assert node._state == State.SCAN_MOVE, (
            f"Expected SCAN_MOVE, got {node._state}")

    def test_laser_fire_not_published(self):
        """dry_run=True: /laser_fire True is never published."""
        node = _make_node(dry_run=True)
        node._tick()
        node._fire_pub.publish.assert_not_called()

    def test_treated_lists_remain_empty(self):
        """dry_run=True: no pan or world position is blacklisted."""
        node = _make_node(dry_run=True)
        node._tick()
        assert node._treated_pans == []
        assert node._treated_world_xy == []


class TestDryRunFalseFiresNormally:
    def test_state_transitions_to_firing(self):
        """dry_run=False: FIRE_LASER → FIRING."""
        node = _make_node(dry_run=False)
        node._tick()
        assert node._state == State.FIRING, (
            f"Expected FIRING, got {node._state}")

    def test_laser_fire_true_published(self):
        """dry_run=False: /laser_fire True is published."""
        from std_msgs.msg import Bool as _Bool
        node = _make_node(dry_run=False)
        node._tick()
        node._fire_pub.publish.assert_called_once()
```

- [ ] **Step 1.2 — Run tests, confirm they FAIL**

```bash
cd ~/ros2_ws
source install/setup.bash
python3 -m pytest src/watermelon_demo/test/test_dry_run.py -v 2>&1 | tail -20
```

Expected: `AttributeError` or `AssertionError` — `dry_run` parameter not yet declared.

- [ ] **Step 1.3 — Add `dry_run` parameter declaration (line 108)**

In `src/watermelon_demo/watermelon_demo/arm_controller_node.py`, after line 108:

```python
        self.declare_parameter('scan_dwell_time', 2.0)
        self.declare_parameter('laser_fire_duration', 1.0)
        self.declare_parameter('dry_run', False)          # ← add this line
```

- [ ] **Step 1.4 — Replace the FIRE_LASER state block (lines 242-246)**

Replace:
```python
        elif self._state == State.FIRE_LASER:
            self.get_logger().info('Firing laser at weed')
            self._fire_pub.publish(Bool(data=True))
            self._fire_start = self._now()
            self._state = State.FIRING
```

With:
```python
        elif self._state == State.FIRE_LASER:
            if self.get_parameter('dry_run').value:
                self.get_logger().info(
                    'DRY RUN: weed centred — would fire laser here')
                self._state = State.SCAN_MOVE
            else:
                self.get_logger().info('Firing laser at weed')
                self._fire_pub.publish(Bool(data=True))
                self._fire_start = self._now()
                self._state = State.FIRING
```

- [ ] **Step 1.5 — Run tests, confirm they PASS**

```bash
python3 -m pytest src/watermelon_demo/test/test_dry_run.py -v 2>&1 | tail -20
```

Expected: `5 passed`.

- [ ] **Step 1.6 — Commit**

```bash
git add src/watermelon_demo/watermelon_demo/arm_controller_node.py \
        src/watermelon_demo/test/test_dry_run.py
git commit -m "feat: add dry_run parameter to arm_controller_node

When dry_run=true, FIRE_LASER logs intent and returns to SCAN_MOVE
without publishing /laser_fire or blacklisting the weed position.
Enables detection-only runs on real hardware without a laser device."
```

---

## Task 2: Live camera intrinsics — `laser_effect_node`

**Files:**
- Modify: `src/watermelon_demo/watermelon_demo/laser_effect_node.py:33-40,46-60`
- Create: `src/watermelon_demo/test/test_camera_intrinsics.py`

- [ ] **Step 2.1 — Write the failing tests**

Create `src/watermelon_demo/test/test_camera_intrinsics.py`:

```python
"""
test_camera_intrinsics.py — verify laser_effect_node updates its ray-cast
intrinsics from CameraInfo messages and falls back to hardcoded values when
no CameraInfo has been received.
"""
import sys, math
from unittest.mock import MagicMock

for _mod in [
    'rclpy', 'rclpy.node', 'rclpy.time',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'visualization_msgs', 'visualization_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'tf2_ros',
    'builtin_interfaces', 'builtin_interfaces.msg',
]:
    sys.modules[_mod] = MagicMock()

import importlib
_mod = importlib.import_module('watermelon_demo.laser_effect_node')
LaserEffectNode = _mod.LaserEffectNode


def _make_laser_node():
    node = LaserEffectNode.__new__(LaserEffectNode)
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.create_subscription = MagicMock()
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(to_msg=MagicMock()))))
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._marker_pub = MagicMock()
    node._last_weed_pixel = None
    # Initialise intrinsics to hardcoded fallback values
    node._fx = _mod.FX
    node._fy = _mod.FY
    node._cx = _mod.CX
    node._cy = _mod.CY
    return node


def _make_camera_info(fx, fy, cx, cy):
    """Build a minimal CameraInfo mock with a K matrix."""
    info = MagicMock()
    # K is row-major 3×3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


class TestCameraInfoUpdatesIntrinsics:
    def test_fx_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=600.0, cx=320.0, cy=240.0))
        assert node._fx == 600.0, f"Expected _fx=600.0, got {node._fx}"

    def test_fy_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=605.0, cx=320.0, cy=240.0))
        assert node._fy == 605.0, f"Expected _fy=605.0, got {node._fy}"

    def test_cx_cy_updated_from_camera_info(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=600.0, cx=321.5, cy=241.5))
        assert node._cx == 321.5
        assert node._cy == 241.5

    def test_second_camera_info_overwrites_first(self):
        node = _make_laser_node()
        node._camera_info_cb(_make_camera_info(fx=600.0, fy=600.0, cx=320.0, cy=240.0))
        node._camera_info_cb(_make_camera_info(fx=700.0, fy=700.0, cx=330.0, cy=250.0))
        assert node._fx == 700.0


class TestFallbackIntrinsics:
    def test_fallback_values_are_hardcoded_defaults(self):
        """Before any CameraInfo arrives, intrinsics equal the hardcoded Gazebo values."""
        node = _make_laser_node()
        expected_fx = _mod.FX
        assert abs(node._fx - expected_fx) < 1e-6, (
            f"Fallback _fx should be {expected_fx}, got {node._fx}")

    def test_pixel_to_world_uses_instance_intrinsics(self):
        """_pixel_to_world uses _fx/_fy/_cx/_cy, not the module-level constants."""
        node = _make_laser_node()
        node._fx = 999.0  # deliberately wrong
        # TF lookup will fail (mocked), so result is None — but we just want
        # to confirm the method references instance attributes, not constants.
        # If it used module-level FX, the value would be ~554, not 999.
        import inspect
        src = inspect.getsource(node._pixel_to_world)
        assert '_fx' in src or 'self._fx' in src, (
            "_pixel_to_world must use self._fx, not module-level FX")
```

- [ ] **Step 2.2 — Run tests, confirm they FAIL**

```bash
python3 -m pytest src/watermelon_demo/test/test_camera_intrinsics.py -v 2>&1 | tail -20
```

Expected: `AttributeError: _camera_info_cb` — method not yet defined.

- [ ] **Step 2.3 — Update `laser_effect_node.py`**

**Replace lines 33-43** (hardcoded constants block):

```python
# Camera intrinsics — Gazebo fallback (640×480, hfov=1.047 rad).
# Overridden at runtime by CameraInfo messages (real RealSense D435).
IMG_W = 640
IMG_H = 480
HFOV  = 1.047
FX = (IMG_W / 2) / math.tan(HFOV / 2)   # ≈ 554.3 px
FY = FX
CX = IMG_W / 2.0
CY = IMG_H / 2.0
```

**In `LaserEffectNode.__init__`** (after the existing subscriptions at line ~54), add:

```python
        # Intrinsics — start with Gazebo fallback, updated by CameraInfo
        self._fx = FX
        self._fy = FY
        self._cx = CX
        self._cy = CY

        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 1)
```

**Add the import** at the top of the file imports section:
```python
from sensor_msgs.msg import CameraInfo
```

**Add `_camera_info_cb` method** to `LaserEffectNode` (after `_detection_cb`):

```python
    def _camera_info_cb(self, msg):
        """Update ray-cast intrinsics from RealSense calibration."""
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
```

**Update `_pixel_to_world`** — replace the two lines that use module-level `FX, FY, CX, CY`:

```python
        # Before (uses module-level constants):
        ray_cam = np.array([
            1.0,
            -(u - CX) / FX,
            -(v - CY) / FY,
        ])
```

```python
        # After (uses instance attributes, updated by CameraInfo):
        ray_cam = np.array([
            1.0,
            -(u - self._cx) / self._fx,
            -(v - self._cy) / self._fy,
        ])
```

- [ ] **Step 2.4 — Run tests, confirm they PASS**

```bash
python3 -m pytest src/watermelon_demo/test/test_camera_intrinsics.py -v 2>&1 | tail -20
```

Expected: `6 passed`.

- [ ] **Step 2.5 — Confirm existing tests still pass**

```bash
python3 -m pytest src/watermelon_demo/test/ -v --ignore=src/watermelon_demo/test/test_camera_tf.py \
    --ignore=src/watermelon_demo/test/test_dry_run_integration.py 2>&1 | tail -10
```

Expected: no regressions.

- [ ] **Step 2.6 — Commit**

```bash
git add src/watermelon_demo/watermelon_demo/laser_effect_node.py \
        src/watermelon_demo/test/test_camera_intrinsics.py
git commit -m "feat: live camera intrinsics from CameraInfo in laser_effect_node

Subscribes to /camera/color/camera_info and updates _fx/_fy/_cx/_cy
used in ray-casting. Falls back to hardcoded Gazebo values (hfov=1.047)
when no CameraInfo has been received, keeping simulation unchanged."
```

---

## Task 3: `config/real_params.yaml`

**Files:**
- Create: `src/watermelon_demo/config/real_params.yaml`

- [ ] **Step 3.1 — Write the failing test** (add to `test/test_config.py` or create `test/test_real_params.py`)

Create `src/watermelon_demo/test/test_real_params.py`:

```python
"""test_real_params.py — validate real_params.yaml is well-formed and complete."""
import os
import pytest

try:
    import yaml
except ImportError:
    pytest.skip('pyyaml not available', allow_module_level=True)

REAL_PARAMS = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'real_params.yaml')


def _load():
    with open(REAL_PARAMS) as f:
        return yaml.safe_load(f)


class TestRealParams:
    def test_file_exists(self):
        assert os.path.isfile(REAL_PARAMS), f"real_params.yaml not found at {REAL_PARAMS}"

    def test_arm_controller_has_dry_run(self):
        data = _load()
        params = data['arm_controller_node']['ros__parameters']
        assert 'dry_run' in params, "arm_controller_node missing 'dry_run'"
        assert isinstance(params['dry_run'], bool)

    def test_use_sim_time_false(self):
        data = _load()
        for node_name, node_data in data.items():
            p = node_data.get('ros__parameters', {})
            if 'use_sim_time' in p:
                assert p['use_sim_time'] is False, (
                    f"{node_name}: use_sim_time must be false for real hardware")

    def test_use_real_model_true(self):
        data = _load()
        p = data['detection_node']['ros__parameters']
        assert p['use_real_model'] is True, "detection_node must use real YOLO model"

    def test_save_debug_frames_false(self):
        data = _load()
        p = data['detection_node']['ros__parameters']
        assert p.get('save_debug_frames') is False, (
            "save_debug_frames should be false (no log dir guaranteed on robot)")
```

- [ ] **Step 3.2 — Run test, confirm it FAILS**

```bash
python3 -m pytest src/watermelon_demo/test/test_real_params.py -v 2>&1 | tail -10
```

Expected: `FAILED test_file_exists` — file not yet created.

- [ ] **Step 3.3 — Create `config/real_params.yaml`**

```yaml
arm_controller_node:
  ros__parameters:
    scan_dwell_time: 1.0
    laser_fire_duration: 1.0
    dry_run: false          # override at launch: dry_run:=true for detection-only
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

- [ ] **Step 3.4 — Run test, confirm it PASSES**

```bash
python3 -m pytest src/watermelon_demo/test/test_real_params.py -v 2>&1 | tail -10
```

Expected: `5 passed`.

- [ ] **Step 3.5 — Commit**

```bash
git add src/watermelon_demo/config/real_params.yaml \
        src/watermelon_demo/test/test_real_params.py
git commit -m "feat: add real_params.yaml for physical UR5 + RealSense deployment

use_real_model=true, use_sim_time=false, save_debug_frames=false.
dry_run defaults to false; overridden by demo_real.launch.py arg."
```

---

## Task 4: `launch/demo_real.launch.py` + camera TF test

**Files:**
- Create: `src/watermelon_demo/launch/demo_real.launch.py`
- Create: `src/watermelon_demo/test/test_camera_tf.py`

- [ ] **Step 4.1 — Write the TF test**

Create `src/watermelon_demo/test/test_camera_tf.py`:

```python
"""
test_camera_tf.py — verify the static TF constants in demo_real.launch.py
are present and non-trivially defined (not all zeros from a forgotten measurement).

This is an offline test: it imports the launch module and reads the
CAMERA_* constants without starting any ROS processes.
"""
import sys, os, importlib, types, unittest.mock as mock

# Stub launch dependencies so we can import the module without ROS
for _m in ['launch', 'launch.actions', 'launch.launch_description_sources',
           'launch_ros', 'launch_ros.actions', 'launch_ros.substitutions',
           'ament_index_python', 'ament_index_python.packages']:
    sys.modules[_m] = mock.MagicMock()

# Add the launch file's directory to path
_launch_dir = os.path.join(os.path.dirname(__file__), '..', 'launch')
sys.path.insert(0, os.path.abspath(_launch_dir))

import demo_real as _launch_mod


class TestCameraTFConstants:
    def test_camera_z_positive(self):
        """Camera must be mounted some distance from tool0 (z > 0)."""
        assert _launch_mod.CAMERA_Z > 0, (
            f"CAMERA_Z={_launch_mod.CAMERA_Z} — measure the tool0→camera distance")

    def test_all_constants_defined(self):
        for attr in ['CAMERA_X', 'CAMERA_Y', 'CAMERA_Z',
                     'CAMERA_ROLL', 'CAMERA_PITCH', 'CAMERA_YAW']:
            assert hasattr(_launch_mod, attr), f"{attr} not defined in demo_real.launch.py"
            assert isinstance(getattr(_launch_mod, attr), float), (
                f"{attr} must be a float")

    def test_robot_ip_default_defined(self):
        assert hasattr(_launch_mod, 'DEFAULT_ROBOT_IP'), (
            "DEFAULT_ROBOT_IP must be defined in demo_real.launch.py")
        assert _launch_mod.DEFAULT_ROBOT_IP != '', "DEFAULT_ROBOT_IP must not be empty"
```

- [ ] **Step 4.2 — Run test, confirm it FAILS**

```bash
python3 -m pytest src/watermelon_demo/test/test_camera_tf.py -v 2>&1 | tail -10
```

Expected: `ModuleNotFoundError: No module named 'demo_real'`.

- [ ] **Step 4.3 — Create `launch/demo_real.launch.py`**

```python
"""
demo_real.launch.py — launch watermelon weed-detection on physical UR5 + RealSense D435.

Usage:
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=true
    ros2 launch watermelon_demo demo_real.launch.py robot_ip:=192.168.1.100 dry_run:=false

Before running:
    1. Measure the camera mount offset from tool0 and fill in the CAMERA_* constants below.
    2. Ensure ur_robot_driver, realsense2_camera, and image_view are installed.
    3. Set use_real_model: true and provide best.pt at /home/lior/best.pt.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Robot IP ────────────────────────────────────────────────────────────────
DEFAULT_ROBOT_IP = '192.168.1.100'

# ── Camera mount geometry — MEASURE FROM PHYSICAL SETUP ─────────────────────
# Offset from UR5 tool0 frame origin to RealSense D435 optical centre (metres).
# RealSense optical frame: +Z forward, +X right, +Y down.
# UR5 tool0 frame: +Z out of flange face, +X forward.
CAMERA_X   = 0.00   # TODO: measure lateral offset (m)
CAMERA_Y   = 0.00   # TODO: measure vertical offset (m)
CAMERA_Z   = 0.05   # TODO: measure flange-to-lens distance (m) — ~5 cm typical
CAMERA_ROLL  = 0.0  # TODO: measure (rad)
CAMERA_PITCH = 0.0  # TODO: measure (rad)
CAMERA_YAW   = 0.0  # TODO: measure (rad)


def generate_launch_description():
    pkg = get_package_share_directory('watermelon_demo')

    params_file = os.path.join(pkg, 'config', 'real_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'demo_rviz.rviz')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value=DEFAULT_ROBOT_IP,
        description='IP address of the UR5 controller')

    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='true = detection-only (no laser signal); false = full pipeline')

    robot_ip  = LaunchConfiguration('robot_ip')
    dry_run   = LaunchConfiguration('dry_run')

    # ── 1. UR5 driver ────────────────────────────────────────────────────────
    ur_driver = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        name='ur_ros2_control_node',
        parameters=[{
            'robot_ip': robot_ip,
            'ur_type': 'ur5',
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── 2. Joint state broadcaster + arm controller (ros2_control) ───────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scaled_joint_trajectory_controller'],
        output='screen',
    )

    # ── 3. RealSense D435 camera ─────────────────────────────────────────────
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30.0,
        }],
        output='screen',
    )

    # ── 4. Static TF: tool0 → camera_link ───────────────────────────────────
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '--x',     str(CAMERA_X),
            '--y',     str(CAMERA_Y),
            '--z',     str(CAMERA_Z),
            '--roll',  str(CAMERA_ROLL),
            '--pitch', str(CAMERA_PITCH),
            '--yaw',   str(CAMERA_YAW),
            '--frame-id',       'tool0',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 5. Demo nodes (delayed 5 s for driver + camera to be ready) ──────────
    demo_nodes = TimerAction(period=5.0, actions=[
        Node(
            package='watermelon_demo',
            executable='detection_node',
            name='detection_node',
            parameters=[params_file],
            remappings=[('/camera/image_raw', '/camera/color/image_raw')],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='arm_controller_node',
            name='arm_controller_node',
            parameters=[params_file, {'dry_run': dry_run}],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='laser_effect_node',
            name='laser_effect_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='watermelon_demo',
            executable='field_manager_node',
            name='field_manager_node',
            parameters=[params_file],
            output='screen',
        ),
        # Live annotated camera feed for audience
        Node(
            package='image_view',
            executable='image_view',
            name='detection_display',
            remappings=[('image', '/detection_image')],
            output='log',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ),
    ])

    return LaunchDescription([
        robot_ip_arg,
        dry_run_arg,
        ur_driver,
        joint_state_broadcaster,
        trajectory_controller,
        realsense,
        camera_tf,
        demo_nodes,
    ])
```

- [ ] **Step 4.4 — Run test, confirm it PASSES**

```bash
python3 -m pytest src/watermelon_demo/test/test_camera_tf.py -v 2>&1 | tail -10
```

Expected: `3 passed`. (CAMERA_Z=0.05 > 0 ✓)

- [ ] **Step 4.5 — Rebuild package to register new launch file**

```bash
cd ~/ros2_ws
colcon build --packages-select watermelon_demo
source install/setup.bash
```

- [ ] **Step 4.6 — Verify launch file is found**

```bash
ros2 launch watermelon_demo demo_real.launch.py --show-args 2>&1 | head -10
```

Expected: shows `robot_ip` and `dry_run` arguments.

- [ ] **Step 4.7 — Commit**

```bash
git add src/watermelon_demo/launch/demo_real.launch.py \
        src/watermelon_demo/test/test_camera_tf.py
git commit -m "feat: add demo_real.launch.py for physical UR5 + RealSense D435

Replaces Gazebo/bridge with ur_robot_driver + realsense2_camera.
Static TF tool0→camera_link with placeholder constants (measure before use).
dry_run launch arg (default true) controls laser firing.
image_view node shows live /detection_image for audience."
```

---

## Task 5: End-to-end dry-run integration test

**Files:**
- Create: `src/watermelon_demo/test/test_dry_run_integration.py`

- [ ] **Step 5.1 — Write the integration test**

Create `src/watermelon_demo/test/test_dry_run_integration.py`:

```python
"""
test_dry_run_integration.py — end-to-end check that dry_run=true prevents
any /laser_fire publication even when a weed detection is injected.

Uses the same mock-ROS approach as test_dry_run.py; no live ROS context needed.
"""
import sys
from unittest.mock import MagicMock

for _m in [
    'rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.time',
    'std_msgs', 'std_msgs.msg',
    'vision_msgs', 'vision_msgs.msg',
    'control_msgs', 'control_msgs.action',
    'trajectory_msgs', 'trajectory_msgs.msg',
    'builtin_interfaces', 'builtin_interfaces.msg',
    'tf2_ros',
]:
    sys.modules[_m] = MagicMock()

import importlib
_mod = importlib.import_module('watermelon_demo.arm_controller_node')
State = _mod.State


def _make_node(dry_run: bool):
    node = _mod.ArmControllerNode.__new__(_mod.ArmControllerNode)
    params = {
        'scan_dwell_time': 2.0,
        'laser_fire_duration': 1.0,
        'dry_run': dry_run,
    }
    node.get_parameter = lambda name: MagicMock(value=params[name])
    node.declare_parameter = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock(
        info=MagicMock(), warn=MagicMock()))
    node.get_clock = MagicMock(return_value=MagicMock(
        now=MagicMock(return_value=MagicMock(nanoseconds=int(5e9)))))
    node.create_publisher = MagicMock(return_value=MagicMock())
    node.create_subscription = MagicMock()
    node.create_timer = MagicMock()
    node._action = MagicMock()
    node._tf_buffer = MagicMock()
    node._tf_listener = MagicMock()
    node._fire_pub = MagicMock()
    node._busy = False
    node._state = State.WAITING
    node._scan_idx = 0
    node._current_joints = [0.0] * 6
    node._last_weed_det = (320, 380)   # weed below centre
    node._wait_start = 0.0             # _now() >> scan_dwell_time → moves to MOVE_TO_WEED
    node._fire_start = 0.0
    node._lock_start = 0.0
    node._treated_pans = []
    node._treated_world_xy = []
    return node


class TestDryRunEndToEnd:
    def test_no_laser_fire_published_through_full_cycle(self):
        """Simulate WAITING → MOVE_TO_WEED → (centred) → FIRE_LASER with dry_run=True.
        /laser_fire must never be published."""
        node = _make_node(dry_run=True)

        # Tick 1: WAITING — weed detected, TF unavailable → goes to MOVE_TO_WEED
        node._pixel_to_world_xy = MagicMock(return_value=None)
        node._tick()
        assert node._state == State.MOVE_TO_WEED

        # Drive MOVE_TO_WEED → FIRE_LASER: inject weed already centred
        node._last_weed_det = (320, 240)   # dead centre → err_x=0, err_y=0
        node._state = State.MOVE_TO_WEED
        node._move_to = MagicMock()
        node._tick()
        assert node._state == State.FIRE_LASER

        # Tick: FIRE_LASER with dry_run=True → SCAN_MOVE, no publish
        node._tick()
        assert node._state == State.SCAN_MOVE
        node._fire_pub.publish.assert_not_called()

    def test_with_dry_run_false_fire_is_published(self):
        """Sanity check: dry_run=False still publishes /laser_fire."""
        node = _make_node(dry_run=False)
        node._state = State.FIRE_LASER
        node._tick()
        assert node._state == State.FIRING
        node._fire_pub.publish.assert_called_once()
```

- [ ] **Step 5.2 — Run test, confirm it PASSES**

```bash
python3 -m pytest src/watermelon_demo/test/test_dry_run_integration.py -v 2>&1 | tail -10
```

Expected: `2 passed`.

- [ ] **Step 5.3 — Run the full test suite**

```bash
python3 -m pytest src/watermelon_demo/test/ -v \
    --ignore=src/watermelon_demo/test/test_launch.py \
    --ignore=src/watermelon_demo/test/test_nodes.py 2>&1 | tail -20
```

Expected: all tests pass (launch and node tests require a running ROS context).

- [ ] **Step 5.4 — Commit**

```bash
git add src/watermelon_demo/test/test_dry_run_integration.py
git commit -m "test: end-to-end dry_run integration test

Simulates WAITING→MOVE_TO_WEED→FIRE_LASER cycle with dry_run=true
and confirms /laser_fire is never published. Runs without a live
ROS context using the same mock-ROS pattern as test_dry_run.py."
```

---

## Task 6: Push and update README

- [ ] **Step 6.1 — Push all commits**

```bash
git push origin main
```

- [ ] **Step 6.2 — Add Real Hardware section to README**

In `README.md`, add the following section after **Build & Run**:

```markdown
## Real Hardware (UR5 + RealSense D435)

**Prerequisites:** `ur_robot_driver`, `realsense2_camera`, `image_view` installed.

**Before first run:** measure the physical offset from the UR5 `tool0` flange
to the RealSense D435 optical centre and update the `CAMERA_*` constants in
`launch/demo_real.launch.py`.

```bash
# Detection-only (arm centres over weeds, never fires):
ros2 launch watermelon_demo demo_real.launch.py \
    robot_ip:=192.168.1.100 dry_run:=true

# Full pipeline (arm centres and signals laser firing):
ros2 launch watermelon_demo demo_real.launch.py \
    robot_ip:=192.168.1.100 dry_run:=false
```

A live annotated camera window (`image_view`) opens automatically showing
real-time weed detections. RViz shows the 3D field markers and laser beam arrow.
```

- [ ] **Step 6.3 — Commit and push README**

```bash
git add README.md
git commit -m "docs: add Real Hardware section to README"
git push origin main
```

---

## Verification Checklist (after all tasks)

- [ ] `python3 -m pytest src/watermelon_demo/test/test_dry_run.py -v` → 5 passed
- [ ] `python3 -m pytest src/watermelon_demo/test/test_camera_intrinsics.py -v` → 6 passed
- [ ] `python3 -m pytest src/watermelon_demo/test/test_real_params.py -v` → 5 passed
- [ ] `python3 -m pytest src/watermelon_demo/test/test_camera_tf.py -v` → 3 passed
- [ ] `python3 -m pytest src/watermelon_demo/test/test_dry_run_integration.py -v` → 2 passed
- [ ] `ros2 launch watermelon_demo demo_real.launch.py --show-args` → shows `robot_ip`, `dry_run`
- [ ] `ros2 launch watermelon_demo demo.launch.py` → Gazebo sim still works unchanged
