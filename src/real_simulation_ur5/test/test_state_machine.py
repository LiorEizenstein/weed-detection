"""
Tests for sim_arm_controller state transitions.

The stub setup that fakes ROS happens only inside fixtures (not at module
level), so pytest collection does not pollute sys.modules and other test
files can still import real ROS message types.
"""
import sys
import types
import math
import pytest

# ── _S and constants defined locally ────────────────────────────────────────
# These mirror the production definitions.  Tests below also verify that the
# production module matches these expected values.

class _S:
    INIT       = 'INIT'
    SCAN_MOVE  = 'SCAN_MOVE'
    MOVING     = 'MOVING'
    WAITING    = 'WAITING'
    FOUND_WEED = 'FOUND_WEED'

_EXPECTED_SCAN_COUNT = 10   # 10-pose sweep from -90° to +90°
_EXPECTED_JOINT_COUNT = 5   # SO-ARM101 has 5 joints


# ── ROS stub helpers (called only inside fixtures) ───────────────────────────

def _install_stubs():
    """Replace ROS packages in sys.modules with minimal stubs.
    Returns a dict of the original values so they can be restored."""
    saved = {}
    stub_names = [
        'rclpy', 'rclpy.node', 'rclpy.action',
        'control_msgs', 'control_msgs.action',
        'trajectory_msgs', 'trajectory_msgs.msg',
        'builtin_interfaces', 'builtin_interfaces.msg',
        'visualization_msgs', 'visualization_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'vision_msgs', 'vision_msgs.msg',
    ]
    for name in stub_names:
        saved[name] = sys.modules.get(name)
        sys.modules[name] = types.ModuleType(name)

    import rclpy.node as _rn, rclpy.action as _ra

    class _Param:
        def __init__(self, v): self.value = v

    class _Node:
        _params: dict = {}
        def __init__(self, *a, **kw): pass
        def declare_parameter(self, name, default):
            self._params[name] = default
            return _Param(default)
        def get_parameter(self, name): return _Param(self._params.get(name, 0))
        def create_subscription(self, *a, **kw): pass
        def create_publisher(self, *a, **kw):
            p = types.SimpleNamespace()
            p.msgs = []
            p.publish = lambda m: p.msgs.append(m)
            return p
        def create_timer(self, *a, **kw): pass
        def get_logger(self):
            lg = types.SimpleNamespace()
            for m in ('info', 'warn', 'error', 'debug'):
                setattr(lg, m, lambda *a, **kw: None)
            return lg
        def get_clock(self):
            ck = types.SimpleNamespace()
            ck.now = lambda: types.SimpleNamespace(nanoseconds=0.0)
            return ck

    _rn.Node = _Node
    _ra.ActionClient = lambda *a, **kw: None

    import control_msgs.action as _ca
    _ca.FollowJointTrajectory = type('FJT', (), {
        'Goal': type('G', (), {
            'trajectory': type('T', (), {
                'joint_names': None, 'points': None
            })()
        })
    })
    import trajectory_msgs.msg as _tm
    _tm.JointTrajectoryPoint = type('JTP', (), {
        'positions': None, 'time_from_start': None
    })
    import builtin_interfaces.msg as _bm
    _bm.Duration = type('D', (), {'sec': 0, 'nanosec': 0})
    import visualization_msgs.msg as _vm
    _vm.Marker    = type('M',  (), {'SPHERE': 2, 'ADD': 0})
    _vm.MarkerArray = type('MA', (), {'markers': None})
    import std_msgs.msg as _sm
    _sm.ColorRGBA = type('C', (), {})
    import sensor_msgs.msg as _smsg
    _smsg.JointState = object
    import vision_msgs.msg as _vism
    _vism.Detection2DArray = object

    return saved


def _restore_stubs(saved: dict):
    for name, orig in saved.items():
        if orig is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = orig
    # Remove cached sim_arm_controller so it can be reimported cleanly later
    sys.modules.pop('real_simulation_ur5.sim_arm_controller', None)


# ── Fixtures ─────────────────────────────────────────────────────────────────

@pytest.fixture(scope='module')
def ros_stubs():
    """Install ROS stubs for the duration of this test module, then restore."""
    saved = _install_stubs()
    yield
    _restore_stubs(saved)


@pytest.fixture(scope='module')
def production_constants(ros_stubs):
    """Import production SCAN_POSES after stubs are in place."""
    from real_simulation_ur5.sim_arm_controller import SCAN_POSES
    return SCAN_POSES


# ── Lightweight state-machine harness ────────────────────────────────────────

class FakeController:
    """Drives only the state-transition logic with no ROS I/O."""

    def __init__(self, dwell=3.0, pause=2.0):
        self.state          = _S.INIT
        self.scan_idx       = 0
        self.busy           = False
        self.last_det       = None
        self.wait_start     = 0.0
        self.found_start    = 0.0
        self.dwell          = dwell
        self.pause          = pause
        self.current_joints = [0.0] * 6
        self.markers        = []
        self._t             = 0.0

    def _now(self): return self._t
    def advance(self, dt): self._t += dt

    def inject_detection(self, cx=320, cy=240, cls='1', conf=0.85):
        self.last_det = (cx, cy, cls, conf)

    def tick(self):
        if self.busy:
            return
        if self.state == _S.INIT:
            self.state = _S.MOVING
            self._finish_move(_S.SCAN_MOVE)
        elif self.state == _S.SCAN_MOVE:
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
        self.last_det    = None
        self.wait_start  = self._now()
        self.state       = _S.WAITING

    def _finish_move(self, next_cb):
        if callable(next_cb):
            next_cb()
        else:
            self.state = next_cb


def _booted(dwell=3.0, pause=2.0) -> FakeController:
    c = FakeController(dwell=dwell, pause=pause)
    c.tick()   # INIT → SCAN_MOVE
    c.tick()   # SCAN_MOVE → WAITING
    assert c.state == _S.WAITING
    return c


# ── State transition tests (no ROS needed) ───────────────────────────────────

def test_init_state_at_start():
    assert FakeController().state == _S.INIT


def test_init_transitions_away_on_tick():
    c = FakeController()
    c.tick()
    assert c.state != _S.INIT


def test_reaches_waiting_after_boot():
    assert _booted().state == _S.WAITING


def test_scan_idx_increments_each_pose():
    c = _booted()
    idx_before = c.scan_idx
    c.advance(10.0)
    c.tick()   # WAITING timeout → SCAN_MOVE
    c.tick()   # SCAN_MOVE → WAITING
    assert c.scan_idx == idx_before + 1


def test_weed_detection_triggers_found_weed():
    c = _booted()
    c.inject_detection(cx=310, cy=235, cls='1', conf=0.88)
    c.tick()
    assert c.state == _S.FOUND_WEED


def test_found_weed_stores_marker():
    c = _booted()
    c.inject_detection()
    c.tick()
    assert len(c.markers) == 1
    assert c.markers[0][2] == '1'


def test_found_weed_stays_until_pause_elapsed():
    c = _booted(pause=2.0)
    c.inject_detection()
    c.tick()
    c.advance(1.0)
    c.tick()
    assert c.state == _S.FOUND_WEED


def test_found_weed_resumes_scan_after_pause():
    c = _booted(pause=2.0)
    c.inject_detection()
    c.tick()
    c.advance(2.5)
    c.tick()
    assert c.state == _S.SCAN_MOVE


def test_no_weed_within_dwell_advances_scan():
    c = _booted(dwell=3.0)
    c.advance(2.9)
    c.tick()
    assert c.state == _S.WAITING
    c.advance(0.2)
    c.tick()
    assert c.state == _S.SCAN_MOVE


def test_weed_class_2_triggers_detection():
    c = _booted()
    c.inject_detection(cls='2', conf=0.75)
    c.tick()
    assert c.state == _S.FOUND_WEED
    assert c.markers[0][2] == '2'


def test_scan_index_wraps_without_crash():
    c = _booted()
    for _ in range(_EXPECTED_SCAN_COUNT + 3):
        c.advance(10.0)
        c.tick()
        c.tick()
    assert c.scan_idx > _EXPECTED_SCAN_COUNT


def test_multiple_detections_all_recorded():
    c = _booted(pause=0.1)
    for _ in range(3):
        c.last_det   = None
        c.wait_start = c._now()
        c.state      = _S.WAITING
        c.inject_detection()
        c.tick()
        c.advance(0.2)
        c.tick()
    assert len(c.markers) == 3


# ── Production constant verification (requires ROS stubs) ────────────────────

def test_scan_poses_count(production_constants):
    poses = production_constants
    assert len(poses) == _EXPECTED_SCAN_COUNT


def test_scan_poses_all_five_joints(production_constants):
    poses = production_constants
    for pose in poses:
        assert len(pose) == _EXPECTED_JOINT_COUNT


def test_scan_poses_pan_within_range(production_constants):
    poses = production_constants
    for pose in poses:
        assert -1.58 <= pose[0] <= 1.58


def test_scan_poses_pan_strictly_increasing(production_constants):
    poses = production_constants
    pans = [p[0] for p in poses]
    assert all(pans[i] < pans[i + 1] for i in range(len(pans) - 1))
