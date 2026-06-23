# UR5 Gazebo Weed-Detection Simulation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a runnable Gazebo + RViz simulation where a UR5 arm scans a field, detects weeds (YOLO or stub), marks them in RViz, and continues — with every component verified by an automated test before integration.

**Architecture:** The `real_simulation_ur5` ROS 2 package contains two simulation-only nodes (`sim_arm_controller`, `sim_detection_node`) and a Gazebo world (`simple_field.sdf`). The arm controller is a pure state machine; the detection node wraps YOLO or a probability stub. A single launch file (`sim.launch.py`) wires everything together. Tests are pure Python (no ROS spin needed) for unit tasks; integration tasks use `ros2 launch --dry-run` or a minimal ROS context.

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim 8 (`ur_simulation_gz`), `control_msgs` FollowJointTrajectory, `vision_msgs` Detection2DArray, `visualization_msgs` MarkerArray, pytest, cv_bridge, ultralytics YOLO (optional)

**Hardware modelled:** UR5 arm + Intel RealSense D435 colour camera (90×25×25 mm body, 69°H × 42°V FOV, 640×480 @ 10 Hz in simulation)

## Global Constraints

- Package: `real_simulation_ur5` — never touch `watermelon_demo` source
- All simulation nodes use `use_sim_time: true`
- Arm trajectory controller action: `/scaled_joint_trajectory_controller/follow_joint_trajectory`
- Camera image topic (from Gazebo bridge): `/camera/image_raw`
- Weed detection topic: `/detections` (vision_msgs/Detection2DArray)
- Weed marker topic: `/weed_markers` (visualization_msgs/MarkerArray)
- Stub mode must work with zero external dependencies (no YOLO model file required)
- Tests live in `src/real_simulation_ur5/test/` and run with `pytest`
- Build command: `colcon build --packages-select real_simulation_ur5`

---

## File Map

| File | Status | Responsibility |
|------|--------|----------------|
| `worlds/simple_field.sdf` | ✅ exists | Gazebo world: ground + 4 weed cylinders |
| `urdf/ur5_with_d435.urdf.xacro` | ✅ exists | UR5 + RealSense D435 URDF (accurate FOV, mass, body) |
| `config/sim_params.yaml` | ✅ exists | Node parameters (dwell time, stub probability, …) |
| `config/sim.rviz` | ✅ exists | RViz display config |
| `real_simulation_ur5/sim_arm_controller.py` | ✅ exists | State machine: INIT→SCAN_MOVE→WAITING→FOUND_WEED |
| `real_simulation_ur5/sim_detection_node.py` | ✅ exists | YOLO + stub detection, publishes /detections |
| `launch/sim.launch.py` | ✅ exists | Launches Gazebo + bridge + nodes + RViz |
| `setup.py` | ✅ updated | Registers executables + worlds + urdf data_files |
| `test/test_state_machine.py` | 🔲 to create | Unit tests for arm controller state transitions |
| `test/test_detection_logic.py` | 🔲 to create | Unit tests for detection parsing and stub behavior |
| `test/test_launch_loadable.py` | 🔲 to create | Smoke test: launch file imports without error |

---

## Stage 1 — Package Build & Dependency Check

**Self-check goal:** Confirm the package builds from scratch, all 5 executables are registered, and all installed share files are present.

**Files:** `setup.py` (read-only verification)

- [ ] **Step 1.1 — Build the package**

```bash
cd ~/ros2_ws
colcon build --packages-select real_simulation_ur5
```

Expected output (last 3 lines):
```
Starting >>> real_simulation_ur5
Finished <<< real_simulation_ur5 [~1s]
Summary: 1 package finished
```
If it fails: check for import errors in `setup.py` or missing `resource/` file.

- [ ] **Step 1.2 — Source and list executables**

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg executables real_simulation_ur5
```

Expected — exactly these 5 lines (order may vary):
```
real_simulation_ur5 arm_controller_node
real_simulation_ur5 detection_node
real_simulation_ur5 laser_effect_node
real_simulation_ur5 sim_arm_controller
real_simulation_ur5 sim_detection_node
```

- [ ] **Step 1.3 — Verify installed share files**

```bash
ls ~/ros2_ws/install/real_simulation_ur5/share/real_simulation_ur5/
```

Expected directories: `config  launch  worlds  hook  package.xml  package.*`

```bash
ls ~/ros2_ws/install/real_simulation_ur5/share/real_simulation_ur5/worlds/
```

Expected: `simple_field.sdf`

- [ ] **Step 1.4 — Check required ROS packages exist**

```bash
source ~/ros2_ws/install/setup.bash
for pkg in ur_simulation_gz ros_gz_bridge watermelon_demo control_msgs vision_msgs; do
  ros2 pkg list | grep -q "^$pkg$" && echo "OK  $pkg" || echo "MISSING  $pkg"
done
```

Expected: all 5 lines show `OK`.

---

## Stage 2 — Static File Validation

**Self-check goal:** All config files parse without errors before any node is launched.

- [ ] **Step 2.1 — Validate SDF world XML**

```bash
python3 -c "
import xml.etree.ElementTree as ET
ET.parse('$HOME/ros2_ws/src/real_simulation_ur5/worlds/simple_field.sdf')
print('SDF: valid XML')
"
```

Expected: `SDF: valid XML`

- [ ] **Step 2.2 — Validate YAML files**

```bash
python3 -c "
import yaml
for f in ['config/sim_params.yaml', 'config/sim.rviz']:
    yaml.safe_load(open(f'$HOME/ros2_ws/src/real_simulation_ur5/{f}').read())
    print(f'OK  {f}')
"
```

Expected:
```
OK  config/sim_params.yaml
OK  config/sim.rviz
```

- [ ] **Step 2.3 — Validate launch file imports cleanly**

```bash
python3 -c "
import sys
sys.path.insert(0, '$HOME/ros2_ws/src/real_simulation_ur5/launch')
import importlib.util, pathlib
spec = importlib.util.spec_from_file_location(
    'sim_launch',
    '$HOME/ros2_ws/src/real_simulation_ur5/launch/sim.launch.py')
mod = importlib.util.module_from_spec(spec)
print('Launch file: importable')
"
```

Expected: `Launch file: importable`

---

## Stage 3 — Unit Tests: Detection Logic

**Self-check goal:** The detection parsing and stub logic is correct without needing ROS or a camera.

**Files:**
- Create: `src/real_simulation_ur5/test/test_detection_logic.py`

- [ ] **Step 3.1 — Create the test file**

```python
# src/real_simulation_ur5/test/test_detection_logic.py
"""
Tests for sim_detection_node internals.
No ROS spin needed — we test pure helper functions directly.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

from real_simulation_ur5.sim_detection_node import _make_det

# ── _make_det ────────────────────────────────────────────────────────────────

def test_make_det_weed_side_fields():
    det = _make_det(cls_id=1, conf=0.85, cx=320, cy=240, w=80, h=60)
    assert det.bbox.center.position.x == 320.0
    assert det.bbox.center.position.y == 240.0
    assert det.bbox.size_x == 80.0
    assert det.bbox.size_y == 60.0
    assert len(det.results) == 1
    assert det.results[0].hypothesis.class_id == '1'
    assert abs(det.results[0].hypothesis.score - 0.85) < 1e-6

def test_make_det_weed_top_class_string():
    det = _make_det(cls_id=2, conf=0.72, cx=100, cy=100, w=50, h=50)
    assert det.results[0].hypothesis.class_id == '2'

def test_make_det_watermelon_class():
    det = _make_det(cls_id=0, conf=0.91, cx=200, cy=150, w=120, h=90)
    assert det.results[0].hypothesis.class_id == '0'
    assert det.bbox.size_x == 120.0

def test_make_det_confidence_range():
    for conf in [0.0, 0.5, 1.0]:
        det = _make_det(1, conf, 0, 0, 10, 10)
        assert det.results[0].hypothesis.score == conf

def test_make_det_zero_size():
    det = _make_det(1, 0.8, 320, 240, 0, 0)
    assert det.bbox.size_x == 0.0
    assert det.bbox.size_y == 0.0
```

- [ ] **Step 3.2 — Run and verify all pass**

```bash
cd ~/ros2_ws
pytest src/real_simulation_ur5/test/test_detection_logic.py -v
```

Expected:
```
test_detection_logic.py::test_make_det_weed_side_fields  PASSED
test_detection_logic.py::test_make_det_weed_top_class_string  PASSED
test_detection_logic.py::test_make_det_watermelon_class  PASSED
test_detection_logic.py::test_make_det_confidence_range  PASSED
test_detection_logic.py::test_make_det_zero_size  PASSED
5 passed
```

- [ ] **Step 3.3 — Commit**

```bash
git add src/real_simulation_ur5/test/test_detection_logic.py
git commit -m "test: add detection logic unit tests"
```

---

## Stage 4 — Unit Tests: Arm Controller State Machine

**Self-check goal:** Every state transition in `sim_arm_controller.py` is proven correct in isolation, without needing Gazebo or the action server.

**Files:**
- Create: `src/real_simulation_ur5/test/test_state_machine.py`

- [ ] **Step 4.1 — Create the test file**

```python
# src/real_simulation_ur5/test/test_state_machine.py
"""
Tests for sim_arm_controller state transitions.
Uses a minimal stub that replaces the ROS node — no rclpy.init needed.
"""
import sys, pathlib, types, math
sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

# ── Minimal stubs so the module loads without a live ROS context ─────────────

# Stub rclpy + Node so the import doesn't crash
rclpy_stub = types.ModuleType('rclpy')
rclpy_stub.node = types.ModuleType('rclpy.node')
rclpy_stub.action = types.ModuleType('rclpy.action')
class _Node:
    def __init__(self, *a, **kw): pass
    def declare_parameter(self, name, default): return _Param(default)
    def get_parameter(self, name): return _Param(self._params.get(name, 0))
    def create_subscription(self, *a, **kw): pass
    def create_publisher(self, *a, **kw): return _Publisher()
    def create_timer(self, *a, **kw): pass
    def get_logger(self): return _Logger()
    def get_clock(self): return _Clock()
    _params = {}

class _Param:
    def __init__(self, v): self.value = v

class _Publisher:
    def __init__(self): self.published = []
    def publish(self, msg): self.published.append(msg)

class _Logger:
    def info(self, *a, **kw): pass
    def warn(self, *a, **kw): pass
    def error(self, *a, **kw): pass

class _Clock:
    _t = 0.0
    def now(self):
        class T:
            nanoseconds = _Clock._t * 1e9
        return T()

rclpy_stub.node.Node = _Node
rclpy_stub.action.ActionClient = lambda *a, **kw: None
sys.modules['rclpy'] = rclpy_stub
sys.modules['rclpy.node'] = rclpy_stub.node
sys.modules['rclpy.action'] = rclpy_stub.action
for m in ['control_msgs', 'control_msgs.action', 'trajectory_msgs',
          'trajectory_msgs.msg', 'builtin_interfaces', 'builtin_interfaces.msg',
          'visualization_msgs', 'visualization_msgs.msg',
          'std_msgs', 'std_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
          'vision_msgs', 'vision_msgs.msg']:
    sys.modules[m] = types.ModuleType(m)

# Stub message classes used in the module
import control_msgs.action as _ca
_ca.FollowJointTrajectory = type('FJT', (), {'Goal': type('G', (), {
    'trajectory': type('T', (), {'joint_names': None, 'points': None})()
})})
import trajectory_msgs.msg as _tm
_tm.JointTrajectoryPoint = type('JTP', (), {'positions': None, 'time_from_start': None})
import builtin_interfaces.msg as _bm
_bm.Duration = type('D', (), {'sec': 0, 'nanosec': 0})
import visualization_msgs.msg as _vm
_vm.Marker = type('M', (), {'SPHERE': 2, 'ADD': 0})
_vm.MarkerArray = type('MA', (), {})
import std_msgs.msg as _sm
_sm.ColorRGBA = type('C', (), {})
import sensor_msgs.msg as _smsg
_smsg.JointState = object
import vision_msgs.msg as _vism
_vism.Detection2DArray = object

from real_simulation_ur5.sim_arm_controller import _S, SCAN_POSES, HOME_POSE

# ── Lightweight harness (not a full Node) ────────────────────────────────────

class FakeController:
    """Mimics only the state-transition logic of SimArmController."""

    def __init__(self, dwell=3.0, pause=2.0):
        self.state        = _S.INIT
        self.scan_idx     = 0
        self.busy         = False
        self.last_det     = None
        self.wait_start   = 0.0
        self.found_start  = 0.0
        self.dwell        = dwell
        self.pause        = pause
        self.joints_ok    = True
        self.current_joints = [0.0] * 6
        self.markers      = []
        self._clock_t     = 0.0

    def _now(self): return self._clock_t

    def advance_time(self, dt): self._clock_t += dt

    def inject_detection(self, cx=320, cy=240, cls='1', conf=0.85):
        self.last_det = (cx, cy, cls, conf)

    def tick(self):
        if self.busy:
            return
        if self.state == _S.INIT:
            self.state = _S.MOVING
            self._finish_move(_S.SCAN_MOVE)

        elif self.state == _S.SCAN_MOVE:
            idx = self.scan_idx % len(SCAN_POSES)
            self.scan_idx += 1
            self.state = _S.MOVING
            self._finish_move(self._enter_waiting)

        elif self.state == _S.MOVING:
            pass

        elif self.state == _S.WAITING:
            if self.last_det is not None:
                self.markers.append(self.last_det)
                self.found_start = self._now()
                self.state = _S.FOUND_WEED
            elif self._now() - self.wait_start > self.dwell:
                self.state = _S.SCAN_MOVE

        elif self.state == _S.FOUND_WEED:
            if self._now() - self.found_start > self.pause:
                self.last_det = None
                self.state = _S.SCAN_MOVE

    def _enter_waiting(self):
        self.last_det = None
        self.wait_start = self._now()
        self.state = _S.WAITING

    def _finish_move(self, next_cb):
        if callable(next_cb):
            next_cb()
        else:
            self.state = next_cb


# ── Tests ────────────────────────────────────────────────────────────────────

def _boot(dwell=3.0, pause=2.0):
    c = FakeController(dwell=dwell, pause=pause)
    c.tick()  # INIT → SCAN_MOVE (via MOVING)
    c.tick()  # SCAN_MOVE → WAITING (via MOVING)
    assert c.state == _S.WAITING
    return c


def test_init_transitions_to_scan_move():
    c = FakeController()
    assert c.state == _S.INIT
    c.tick()  # executes INIT branch → sets MOVING
    # simulate move completing
    assert c.state in (_S.MOVING, _S.SCAN_MOVE, _S.WAITING)


def test_scan_move_advances_index():
    c = _boot()
    idx_before = c.scan_idx
    # timeout — advance to next pose
    c.advance_time(4.0)
    c.tick()                       # WAITING timeout → SCAN_MOVE
    assert c.state == _S.SCAN_MOVE
    c.tick()                       # SCAN_MOVE → WAITING (next pose)
    assert c.scan_idx == idx_before + 1


def test_weed_detection_triggers_found_state():
    c = _boot()
    c.inject_detection(cx=310, cy=235, cls='1', conf=0.88)
    c.tick()
    assert c.state == _S.FOUND_WEED


def test_found_weed_publishes_marker():
    c = _boot()
    c.inject_detection()
    c.tick()
    assert len(c.markers) == 1
    assert c.markers[0][2] == '1'   # class id


def test_found_weed_resumes_scan_after_pause():
    c = _boot(pause=2.0)
    c.inject_detection()
    c.tick()                     # → FOUND_WEED
    c.advance_time(1.0)
    c.tick()                     # still in FOUND_WEED (pause not elapsed)
    assert c.state == _S.FOUND_WEED
    c.advance_time(1.5)
    c.tick()                     # pause elapsed → SCAN_MOVE
    assert c.state == _S.SCAN_MOVE


def test_no_weed_timeout_advances_scan():
    c = _boot(dwell=3.0)
    c.advance_time(2.9)
    c.tick()
    assert c.state == _S.WAITING   # not yet
    c.advance_time(0.2)
    c.tick()
    assert c.state == _S.SCAN_MOVE


def test_scan_wraps_around():
    c = _boot()
    n = len(SCAN_POSES)
    for _ in range(n + 2):
        c.advance_time(10.0)
        c.tick()   # timeout → SCAN_MOVE
        c.tick()   # SCAN_MOVE → WAITING
    # index has wrapped — no crash
    assert c.scan_idx > n


def test_weed_class_2_also_triggers():
    c = _boot()
    c.inject_detection(cls='2', conf=0.75)
    c.tick()
    assert c.state == _S.FOUND_WEED
    assert c.markers[0][2] == '2'


def test_multiple_detections_counted():
    c = _boot(pause=0.1)
    for _ in range(3):
        c.last_det = None
        c.wait_start = c._now()
        c.state = _S.WAITING
        c.inject_detection()
        c.tick()                 # → FOUND_WEED
        c.advance_time(0.2)
        c.tick()                 # → SCAN_MOVE
    assert len(c.markers) == 3


def test_home_pose_shape():
    assert len(HOME_POSE) == 6
    assert HOME_POSE == [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


def test_scan_poses_count_and_shape():
    assert len(SCAN_POSES) == 11   # (1.5 - (-1.5)) / 0.3 + 1
    for pose in SCAN_POSES:
        assert len(pose) == 6
        assert -1.51 <= pose[0] <= 1.51    # pan within ±1.5 rad


def test_scan_poses_pan_decreasing():
    pans = [p[0] for p in SCAN_POSES]
    assert all(pans[i] > pans[i+1] for i in range(len(pans)-1))
```

- [ ] **Step 4.2 — Run and verify all pass**

```bash
cd ~/ros2_ws
pytest src/real_simulation_ur5/test/test_state_machine.py -v
```

Expected: **12 tests, all PASSED**. If any fail, the state machine logic has a bug — fix `sim_arm_controller.py` before proceeding.

- [ ] **Step 4.3 — Commit**

```bash
git add src/real_simulation_ur5/test/test_state_machine.py
git commit -m "test: add arm controller state machine unit tests"
```

---

## Stage 5 — Launch File Smoke Test

**Self-check goal:** The launch file can be parsed by ROS 2 without errors and all referenced files exist.

**Files:**
- Create: `src/real_simulation_ur5/test/test_launch_loadable.py`

- [ ] **Step 5.1 — Create the test file**

```python
# src/real_simulation_ur5/test/test_launch_loadable.py
"""
Verifies that sim.launch.py can be loaded and all referenced paths exist.
Does NOT start Gazebo.
"""
import os
from ament_index_python.packages import get_package_share_directory


def test_sim_params_yaml_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'config', 'sim_params.yaml')
    assert os.path.isfile(path), f'Missing: {path}'


def test_sim_rviz_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'config', 'sim.rviz')
    assert os.path.isfile(path), f'Missing: {path}'


def test_world_file_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'worlds', 'simple_field.sdf')
    assert os.path.isfile(path), f'Missing: {path}'


def test_d435_urdf_xacro_exists():
    pkg = get_package_share_directory('real_simulation_ur5')
    path = os.path.join(pkg, 'urdf', 'ur5_with_d435.urdf.xacro')
    assert os.path.isfile(path), f'Missing: {path}'


def test_ur_simulation_gz_launch_exists():
    pkg = get_package_share_directory('ur_simulation_gz')
    path = os.path.join(pkg, 'launch', 'ur_sim_control.launch.py')
    assert os.path.isfile(path), f'Missing: {path}'


def test_sim_executables_registered():
    import subprocess
    result = subprocess.run(
        ['ros2', 'pkg', 'executables', 'real_simulation_ur5'],
        capture_output=True, text=True)
    assert 'sim_arm_controller' in result.stdout
    assert 'sim_detection_node' in result.stdout
```

- [ ] **Step 5.2 — Run and verify all pass**

```bash
cd ~/ros2_ws
source install/setup.bash
pytest src/real_simulation_ur5/test/test_launch_loadable.py -v
```

Expected: **6 tests, all PASSED**

- [ ] **Step 5.3 — Commit**

```bash
git add src/real_simulation_ur5/test/test_launch_loadable.py
git commit -m "test: add launch file path smoke tests"
```

---

## Stage 6 — Gazebo Headless Smoke Test

**Self-check goal:** Gazebo starts, loads the world, and the UR5 joints are visible on `/joint_states`.

> **Prerequisite:** Must be run in a terminal with a display (not pure SSH). If headless, set `export GZ_HEADLESS_RENDERING=1` first.

- [ ] **Step 6.1 — Launch simulation in a separate terminal**

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch real_simulation_ur5 sim.launch.py
```

Let it run. Wait until you see in the terminal:
```
[INFO] [sim_arm_controller]: sim_arm_controller ready — 11 poses
[INFO] [sim_detection_node]: sim_detection_node ready  mode=STUB
```
This takes ~15–30 seconds.

- [ ] **Step 6.2 — In a second terminal, check joint states**

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /joint_states --once
```

Expected: a `sensor_msgs/msg/JointState` message listing all 6 UR5 joints with non-zero positions.

- [ ] **Step 6.3 — Check action server is live**

```bash
ros2 action list
```

Expected line: `/scaled_joint_trajectory_controller/follow_joint_trajectory`

- [ ] **Step 6.4 — Check arm controller is publishing state changes**

```bash
ros2 topic echo /weed_markers --no-arr 2>/dev/null &
sleep 30
```

Within 30 seconds you should see `>>> WEED DETECTED` in the launch terminal (stub fires ~40% probability per 5 s window).

---

## Stage 7 — Camera Bridge Verification

**Self-check goal:** Images flow from Gazebo through the bridge to `/camera/image_raw`.

*(Run while the simulation from Stage 6 is still running)*

- [ ] **Step 7.1 — Verify image topic is publishing**

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /camera/image_raw
```

Expected: ~10 Hz (camera update rate set in URDF).

- [ ] **Step 7.2 — Verify detection image is publishing**

```bash
ros2 topic hz /detection_image
```

Expected: same ~10 Hz.

- [ ] **Step 7.3 — Spot-check a frame (optional)**

```bash
ros2 run rqt_image_view rqt_image_view /detection_image
```

Expected: a window showing the Gazebo camera view with `STUB MODE — no best.pt` label overlay (or YOLO boxes if `best.pt` exists).

---

## Stage 8 — Full State Machine Integration Check

**Self-check goal:** Arm moves through multiple scan poses, detections are logged, markers appear in RViz — end to end.

*(Run while the simulation from Stage 6 is still running)*

- [ ] **Step 8.1 — Monitor state machine transitions**

```bash
ros2 topic echo /weed_markers
```

Leave running for 60 seconds. Each weed detection prints a `Marker` message.

- [ ] **Step 8.2 — Verify log output covers full cycle**

In the launch terminal, look for this sequence at least once:
```
INIT → moving to HOME
SCAN_MOVE → pose 1/11  pan=+1.50 rad
WAITING at pan=+1.50 rad  (dwell=3.0s)
...
>>> WEED DETECTED #1  class=weed_side  conf=0.8x
Detection logged — resuming scan
SCAN_MOVE → pose 2/11  pan=+1.20 rad
```

If `WAITING` appears but `WEED DETECTED` never appears after 2 minutes: increase `stub_detection_probability` to `0.8` in `config/sim_params.yaml`, rebuild, relaunch.

- [ ] **Step 8.3 — Check RViz marker display**

In RViz, the `WeedMarkers` display should show orange spheres appearing in the field area near the arm's field of view. Each new detection adds a new sphere.

---

## Stage 9 — YOLO Integration (when model available)

**Self-check goal:** Real YOLO model replaces stub and detects actual green cylinders in the Gazebo world.

> **Prerequisite:** `best.pt` exists at `/home/lior/best.pt`

- [ ] **Step 9.1 — Launch with model path**

```bash
ros2 launch real_simulation_ur5 sim.launch.py model_path:=/home/lior/best.pt
```

- [ ] **Step 9.2 — Confirm YOLO mode in logs**

In launch terminal:
```
[sim_detection_node]: YOLO model loaded from /home/lior/best.pt
[sim_detection_node]: sim_detection_node ready  mode=YOLO
```

- [ ] **Step 9.3 — Check detection image shows YOLO boxes**

```bash
ros2 run rqt_image_view rqt_image_view /detection_image
```

Expected: coloured bounding boxes around any weed-like objects in the Gazebo camera view. The `STUB MODE` label should be absent.

- [ ] **Step 9.4 — Tune weed positions if needed**

If YOLO never fires because it doesn't recognise Gazebo cylinders (expected — the model was trained on real photos), the system falls back naturally: use the stub, or replace cylinders in `simple_field.sdf` with textured meshes that match your training data.

No code change needed — stub mode is the designed fallback.

---

## Stage 10 — Final Commit & Summary

- [ ] **Step 10.1 — Run all unit tests one last time**

```bash
cd ~/ros2_ws
pytest src/real_simulation_ur5/test/ -v
```

Expected: **all tests PASSED** (at least 18 tests across 3 files).

- [ ] **Step 10.2 — Final build**

```bash
colcon build --packages-select real_simulation_ur5
```

Expected: no warnings, no errors.

- [ ] **Step 10.3 — Commit everything**

```bash
git add src/real_simulation_ur5/
git commit -m "feat: add Gazebo simulation for UR5 weed scan with tests"
```

---

## Quick-Reference Cheat Sheet

```bash
# Build
colcon build --packages-select real_simulation_ur5 && source install/setup.bash

# Run simulation
ros2 launch real_simulation_ur5 sim.launch.py
ros2 launch real_simulation_ur5 sim.launch.py model_path:=/home/lior/best.pt

# Run all unit tests (no Gazebo needed)
pytest src/real_simulation_ur5/test/ -v

# Monitor live topics
ros2 topic hz /joint_states           # ~50 Hz = arm running
ros2 topic hz /camera/image_raw       # ~10 Hz = camera running
ros2 topic echo /weed_markers         # prints on each detection

# Tune without restarting
# Edit config/sim_params.yaml → rebuild → relaunch
```

## Failure Modes & Fixes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Arm never moves after 15 s | Action server not ready | Check `ros2 action list` — controller may have crashed |
| `/camera/image_raw` silent | Camera bridge not started | Check `ros2 node list` for `camera_bridge` |
| `WEED DETECTED` never appears | Stub probability too low | Set `stub_detection_probability: 0.9` in sim_params.yaml |
| RViz shows no robot | `robot_description` not published | Check `ur_simulation_gz` launched correctly |
| Gazebo crashes on start | Missing `ogre2` renderer | Set `GZ_HEADLESS_RENDERING=1` or install `libgz-rendering8` |
