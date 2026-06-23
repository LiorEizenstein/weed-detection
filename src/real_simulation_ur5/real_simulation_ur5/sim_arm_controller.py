"""
sim_arm_controller — simplified Gazebo simulation state machine.

Behaviour:
  INIT        → move to HOME pose
  SCAN_MOVE   → move arm to the next scan pose (shoulder_pan sweeps left→right)
  WAITING     → hold still for scan_dwell_time seconds, watching /detections
                  ├─ weed detected  → FOUND_WEED
                  └─ timeout        → SCAN_MOVE  (advance to next pose)
  FOUND_WEED  → log detection, publish an RViz sphere marker, wait
                detection_pause seconds, then → SCAN_MOVE

No centering loop — this is deliberately simpler than the real-hardware node.
Add it later if you want to close the visual servo loop in simulation.

Topics subscribed:
  /detections    vision_msgs/Detection2DArray
  /joint_states  sensor_msgs/JointState

Topics published:
  /weed_markers  visualization_msgs/MarkerArray   (orange spheres in RViz)

Action client:
  /scaled_joint_trajectory_controller/follow_joint_trajectory
  (provided by ur_simulation_gz via ur_ros2_control)
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
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Scan arc: shoulder_pan sweeps +1.5 → -1.5 rad (~170 deg) in larger steps
# (fewer poses than real-hw node so each dwell is clearly visible)
PAN_MAX  =  1.5
PAN_MIN  = -1.5
PAN_STEP = 0.3
_N       = int(round((PAN_MAX - PAN_MIN) / PAN_STEP)) + 1
SCAN_POSES = [
    [round(PAN_MAX - i * PAN_STEP, 3), -1.2, 1.4, -1.8, -1.57, 0.0]
    for i in range(_N)
]

HOME_POSE    = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
WEED_IDS     = {'1', '2'}
ACTION_SERVER = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


class _S:
    INIT       = 'INIT'
    SCAN_MOVE  = 'SCAN_MOVE'
    MOVING     = 'MOVING'
    WAITING    = 'WAITING'
    FOUND_WEED = 'FOUND_WEED'


class SimArmController(Node):

    def __init__(self):
        super().__init__('sim_arm_controller')

        self.declare_parameter('scan_dwell_time', 3.0)
        self.declare_parameter('detection_pause', 2.0)
        self.declare_parameter('scan_move_time', 2.0)

        self._action = ActionClient(self, FollowJointTrajectory, ACTION_SERVER)
        self._det_sub = self.create_subscription(
            Detection2DArray, '/detections', self._on_detection, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/weed_markers', 10)

        self._state           = _S.INIT
        self._scan_idx        = 0
        self._current_joints  = list(HOME_POSE)
        self._joints_ok       = False
        self._busy            = False
        self._pending_dets    = []     # all weeds seen this dwell window
        self._wait_start      = 0.0
        self._found_start     = 0.0
        self._marker_id       = 0
        self._detections_seen = 0

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'sim_arm_controller ready — {len(SCAN_POSES)} poses  '
            f'dwell={self.get_parameter("scan_dwell_time").value}s  '
            f'action={ACTION_SERVER}')

    # ── subscriptions ────────────────────────────────────────────────────────

    def _on_detection(self, msg: Detection2DArray):
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id in WEED_IDS:
                    entry = (int(det.bbox.center.position.x),
                             int(det.bbox.center.position.y),
                             hyp.hypothesis.class_id,
                             hyp.hypothesis.score)
                    self._pending_dets.append(entry)

    def _on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            self._current_joints = [pos[n] for n in JOINT_NAMES]
            if not self._joints_ok:
                self._joints_ok = True
                self.get_logger().info('Joint states received — arm is live')
        except KeyError:
            pass

    # ── main tick ────────────────────────────────────────────────────────────

    def _tick(self):
        if self._busy:
            return

        if self._state == _S.INIT:
            self.get_logger().info('INIT → moving to HOME')
            self._send_goal(HOME_POSE, duration_sec=4.0,
                            on_done=lambda: self._set(_S.SCAN_MOVE))

        elif self._state == _S.SCAN_MOVE:
            idx  = self._scan_idx % len(SCAN_POSES)
            pose = SCAN_POSES[idx]
            self._scan_idx += 1
            self.get_logger().info(
                f'SCAN_MOVE → pose {idx + 1}/{len(SCAN_POSES)}  '
                f'pan={pose[0]:+.2f} rad')
            self._last_det = None
            t = self.get_parameter('scan_move_time').value
            self._send_goal(pose, duration_sec=t, on_done=self._enter_waiting)

        elif self._state == _S.MOVING:
            pass  # waiting for action to finish

        elif self._state == _S.WAITING:
            now = self._now()
            dwell_expired = now - self._wait_start > \
                self.get_parameter('scan_dwell_time').value
            if dwell_expired:
                if self._pending_dets:
                    # Mark ALL weeds collected during this dwell window
                    seen = set()
                    for cx, cy, cls_id, conf in self._pending_dets:
                        key = (cx // 20, cy // 20)  # deduplicate nearby hits
                        if key in seen:
                            continue
                        seen.add(key)
                        cls_name = 'weed_side' if cls_id == '1' else 'weed_top'
                        self._detections_seen += 1
                        self.get_logger().info(
                            f'>>> WEED #{self._detections_seen}  '
                            f'class={cls_name}  conf={conf:.2f}  px=({cx},{cy})')
                        self._publish_marker(cls_name)
                    self._pending_dets = []
                    self._found_start = now
                    self._state = _S.FOUND_WEED
                else:
                    self.get_logger().info(
                        'No weed in dwell window — advancing to next pose')
                    self._state = _S.SCAN_MOVE

        elif self._state == _S.FOUND_WEED:
            if self._now() - self._found_start > \
                    self.get_parameter('detection_pause').value:
                self.get_logger().info(
                    f'Detection logged (total={self._detections_seen}) — resuming scan')
                self._state = _S.SCAN_MOVE

    # ── helpers ──────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set(self, state):
        self._state = state

    def _enter_waiting(self):
        self._pending_dets = []
        self._wait_start = self._now()
        self._state     = _S.WAITING
        pan = self._current_joints[0] if self._joints_ok else 0.0
        self.get_logger().info(
            f'WAITING at pan={pan:+.2f} rad  '
            f'(dwell={self.get_parameter("scan_dwell_time").value:.1f}s)')

    def _publish_marker(self, cls_name: str):
        # Approximate world position: 0.6 m from base in the scan direction
        pan = self._current_joints[0] if self._joints_ok else 0.0
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns    = 'weeds'
        m.id    = self._marker_id
        self._marker_id += 1
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = 0.6 * math.cos(pan)
        m.pose.position.y = 0.6 * math.sin(pan)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.9)
        m.lifetime.sec = 0   # permanent until cleared

        arr = MarkerArray()
        arr.markers.append(m)
        self._marker_pub.publish(arr)
        self.get_logger().info(
            f'RViz marker #{m.id} ({cls_name}) at '
            f'({m.pose.position.x:.2f}, {m.pose.position.y:.2f})')

    def _send_goal(self, joints, duration_sec=3.0, on_done=None):
        self._state = _S.MOVING
        if not self._action.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f'{ACTION_SERVER} not ready — skipping move')
            self._busy = False
            if on_done:
                on_done()
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        self._busy = True

        def _done(fut):
            self._busy = False
            if on_done:
                on_done()

        def _accepted(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn('Goal rejected by controller')
                self._busy = False
                if on_done:
                    on_done()
                return
            self._current_joints = list(joints)
            gh.get_result_async().add_done_callback(_done)

        self._action.send_goal_async(goal).add_done_callback(_accepted)


def main(args=None):
    rclpy.init(args=args)
    node = SimArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
