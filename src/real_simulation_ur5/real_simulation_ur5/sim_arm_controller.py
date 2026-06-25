"""
sim_arm_controller — SO-ARM101 field scanning state machine.

Behaviour:
  1. Wait for the trajectory action server to become available.
  2. Move to SCAN_POSES[0] (left edge of the sweep).
  3. Dwell scan_dwell_time seconds at each pose, collecting /detections.
     Any weeds found are logged and published as RViz markers.
  4. Move to the next pose and repeat until the final pose is reached.
  5. Swing back to the first pose and run the sweep again — continuous patrol.

States:
  WAIT_SERVER  — polling for /arm_controller action server
  INIT_MOVE    — moving to the first scan pose
  MOVING       — moving to the next scan pose (or back to the start)
  DWELLING     — holding pose, watching for weeds

Topics subscribed:
  /detections    vision_msgs/Detection2DArray
  /joint_states  sensor_msgs/JointState

Topics published:
  /weed_markers  visualization_msgs/MarkerArray  (orange spheres in RViz)

Action client:
  /arm_controller/follow_joint_trajectory
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

JOINT_NAMES = [
    'base_link_to_link1',
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
]

# 180° sweep: joint0 (yaw) goes from -π/2 to +π/2 in equal steps.
_N_POSES  = 10
_PAN_MIN  = -1.5708   # -90°
_PAN_MAX  =  1.5708   # +90°

SCAN_POSES = [
    [round(_PAN_MIN + i * (_PAN_MAX - _PAN_MIN) / (_N_POSES - 1), 4),
     -1.5, 1.5, -0.5, 0.0]
    for i in range(_N_POSES)
]

WEED_IDS      = {'1', '2'}
ACTION_SERVER = '/arm_controller/follow_joint_trajectory'

# How long to allow a move before giving up and dwelling anyway.
# Must be comfortably longer than scan_move_time so the arm can settle.
_MOVE_TIMEOUT_FACTOR = 2.5

# Seconds for the long return swing from the final pose back to the start.
# Larger than a normal step because it covers the full sweep span (180°).
_RETURN_MOVE_TIME = 3.0


class _S:
    WAIT_SERVER = 'WAIT_SERVER'
    INIT_MOVE   = 'INIT_MOVE'
    MOVING      = 'MOVING'
    DWELLING    = 'DWELLING'
    DONE        = 'DONE'


class SimArmController(Node):

    def __init__(self):
        super().__init__('sim_arm_controller')

        self.declare_parameter('scan_dwell_time', 0.5)
        self.declare_parameter('scan_move_time',  1.5)

        self._action = ActionClient(self, FollowJointTrajectory, ACTION_SERVER)
        self.create_subscription(
            Detection2DArray, '/detections', self._on_detection, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/weed_markers', 10)

        self._state           = _S.WAIT_SERVER
        self._pose_idx        = 0          # which pose we are currently dwelling at
        self._current_joints  = [0.0] * len(JOINT_NAMES)
        self._joints_ok       = False
        self._state_entered   = 0.0
        self._goal_active     = False
        self._move_duration   = 0.0        # duration of the in-flight move goal
        self._dwell_dets      = []
        self._detections_total = 0
        self._passes_done     = 0          # completed full sweeps

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'sim_arm_controller ready — {len(SCAN_POSES)} poses, '
            f'{math.degrees(_PAN_MAX - _PAN_MIN):.0f}° sweep  '
            f'dwell={self.get_parameter("scan_dwell_time").value}s  '
            f'move={self.get_parameter("scan_move_time").value}s')

    # ── subscriptions ─────────────────────────────────────────────────────────

    def _on_detection(self, msg: Detection2DArray):
        if self._state != _S.DWELLING:
            return
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id in WEED_IDS:
                    self._dwell_dets.append((
                        int(det.bbox.center.position.x),
                        int(det.bbox.center.position.y),
                        hyp.hypothesis.class_id,
                        hyp.hypothesis.score,
                    ))

    def _on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            self._current_joints = [pos[n] for n in JOINT_NAMES]
            if not self._joints_ok:
                self._joints_ok = True
                self.get_logger().info('Joint states received — arm is live')
        except KeyError:
            pass

    # ── main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        now = self._now()

        # ── WAIT_SERVER ──────────────────────────────────────────────────────
        if self._state == _S.WAIT_SERVER:
            if not self._action.server_is_ready():
                return
            self.get_logger().info(
                'Action server ready — moving to scan start position')
            self._send_goal(SCAN_POSES[0], duration_sec=3.0)
            self._enter(_S.INIT_MOVE)
            return

        # ── INIT_MOVE ────────────────────────────────────────────────────────
        if self._state == _S.INIT_MOVE:
            # Give extra time for the arm to reach the starting pose from
            # wherever Gazebo spawned it.
            if self._goal_active and now - self._state_entered < 5.0:
                return
            if self._goal_active:
                self.get_logger().warn('Init move timed out — starting scan anyway')
                self._goal_active = False
            self._pose_idx = 0
            self._start_dwell()
            return

        # ── MOVING ───────────────────────────────────────────────────────────
        if self._state == _S.MOVING:
            timeout = self._move_duration * _MOVE_TIMEOUT_FACTOR
            if self._goal_active and now - self._state_entered < timeout:
                return
            if self._goal_active:
                self.get_logger().warn(
                    f'Move timed out after {now - self._state_entered:.1f}s '
                    f'(limit {timeout:.1f}s) — advancing to dwell')
                self._goal_active = False
            self._start_dwell()
            return

        # ── DWELLING ─────────────────────────────────────────────────────────
        if self._state == _S.DWELLING:
            dwell = self.get_parameter('scan_dwell_time').value
            if now - self._state_entered < dwell:
                return
            # Dwell complete — log weeds
            self._flush_detections()
            # End of sweep → swing back to the start and run another pass.
            if self._pose_idx >= len(SCAN_POSES) - 1:
                self._passes_done += 1
                self.get_logger().info(
                    f'Sweep #{self._passes_done} complete — reached final pose '
                    f'({math.degrees(_PAN_MAX):.0f}°). Returning to start for '
                    f'pass #{self._passes_done + 1}.')
                self._pose_idx = 0
                self._send_goal(SCAN_POSES[0], duration_sec=_RETURN_MOVE_TIME)
                self._enter(_S.MOVING)
                return
            self._pose_idx += 1
            pose      = SCAN_POSES[self._pose_idx]
            move_time = self.get_parameter('scan_move_time').value
            self.get_logger().info(
                f'→ pose {self._pose_idx + 1}/{len(SCAN_POSES)}  '
                f'pan={math.degrees(pose[0]):+.1f}°')
            self._send_goal(pose, duration_sec=move_time)
            self._enter(_S.MOVING)
            return

        # ── DONE — nothing to do ─────────────────────────────────────────────

    # ── helpers ───────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _enter(self, state):
        self._state        = state
        self._state_entered = self._now()

    def _start_dwell(self):
        self._dwell_dets = []
        self._enter(_S.DWELLING)
        pan_deg = math.degrees(self._current_joints[0]) if self._joints_ok else 0.0
        self.get_logger().info(
            f'DWELLING at pose {self._pose_idx + 1}/{len(SCAN_POSES)}  '
            f'pan={pan_deg:+.1f}°  '
            f'(dwell={self.get_parameter("scan_dwell_time").value:.1f}s)')

    def _flush_detections(self):
        if not self._dwell_dets:
            self.get_logger().info(
                f'Pose {self._pose_idx + 1}/{len(SCAN_POSES)} — no weeds detected')
            return
        seen = set()
        for cx, cy, cls_id, conf in self._dwell_dets:
            key = (cx // 20, cy // 20)
            if key in seen:
                continue
            seen.add(key)
            self._detections_total += 1
            cls_name = 'weed_side' if cls_id == '1' else 'weed_top'
            self.get_logger().info(
                f'>>> WEED #{self._detections_total}  '
                f'class={cls_name}  conf={conf:.2f}  px=({cx},{cy})')
            self._publish_marker(cls_name)
        self._dwell_dets = []

    def _publish_marker(self, cls_name: str):
        pan = self._current_joints[0] if self._joints_ok else 0.0
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'weeds'
        m.id     = self._detections_total
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = 0.6 * math.cos(pan)
        m.pose.position.y = 0.6 * math.sin(pan)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.9)
        m.lifetime.sec = 0
        arr = MarkerArray()
        arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _send_goal(self, joints, duration_sec: float):
        if not self._action.server_is_ready():
            self.get_logger().warn(f'{ACTION_SERVER} not ready — skipping move')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        self._goal_active = True
        self._move_duration = duration_sec

        def _done(fut):
            self._goal_active = False

        def _accepted(fut):
            try:
                gh = fut.result()
            except Exception as exc:
                self.get_logger().error(f'Goal send error: {exc}')
                self._goal_active = False
                return
            if not gh.accepted:
                self.get_logger().warn(
                    f'Goal REJECTED → {[round(j, 2) for j in joints]}')
                self._goal_active = False
                return
            self.get_logger().info(
                f'Goal accepted → {[round(j, 2) for j in joints]}')
            gh.get_result_async().add_done_callback(_done)

        self._action.send_goal_async(goal).add_done_callback(_accepted)


def main(args=None):
    rclpy.init(args=args)
    node = SimArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
