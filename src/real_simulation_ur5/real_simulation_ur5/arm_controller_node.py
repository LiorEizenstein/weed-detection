"""
arm_controller_node — state machine that moves the UR5 arm across scan poses,
detects weeds via /detections, centres the camera over them, then fires the laser.

State machine (shoulder_pan sweeps a wide arc across the field):
  INIT → SCAN_MOVE → SCANNING → WAITING_DETECTION
       → (weed seen)  MOVE_TO_WEED → FIRE_LASER → FIRING → SCAN_MOVE
       → (timed out)  SCAN_MOVE   (advance to next pose, never stalls)

Topics subscribed:
  /detections                 vision_msgs/Detection2DArray
  /joint_states               sensor_msgs/JointState
  /camera/color/camera_info   sensor_msgs/CameraInfo

Topics published:
  /laser_fire    std_msgs/Bool

Action client:
  /scaled_joint_trajectory_controller/follow_joint_trajectory
  (control_msgs/FollowJointTrajectory)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener


JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Scan poses (radians). shoulder_pan sweeps left → right in small steps so the
# camera glides across the field. Pan spans +1.5 → -1.5 rad (~170 deg).
PAN_MAX  = 1.5
PAN_MIN  = -1.5
PAN_STEP = 0.15
_N_SCAN  = int(round((PAN_MAX - PAN_MIN) / PAN_STEP)) + 1
SCAN_POSES = [
    [round(PAN_MAX - i * PAN_STEP, 3), -1.2, 1.4, -1.8, -1.57, 0.0]
    for i in range(_N_SCAN)
]

HOME_POSE = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

SCAN_MOVE_TIME = 1.2
WEED_MOVE_TIME = 1.0
WEED_LOCK_TIMEOUT = 8.0

IMAGE_W = 640
IMAGE_H = 480

# Time to wait for CameraInfo before logging a warning. Centering and ray-cast
# are blocked until intrinsics arrive — this prevents silent wrong-geometry ops.
CAMERA_INFO_WARN_SEC = 10.0

TREATED_WORLD_DIST = 0.20
WRIST1_GAIN = 0.08
WRIST2_GAIN = 0.10
FIRE_ZONE_FRAC = 0.20
TREATED_ZONE_RAD = 0.35
WEED_CLASS_IDS = {'1', '2'}


class State:
    INIT         = 'INIT'
    SCAN_MOVE    = 'SCAN_MOVE'
    SCANNING     = 'SCANNING'
    WAITING      = 'WAITING_DETECTION'
    MOVE_TO_WEED = 'MOVE_TO_WEED'
    FIRE_LASER   = 'FIRE_LASER'
    FIRING       = 'FIRING'


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self.declare_parameter('scan_dwell_time', 2.0)
        self.declare_parameter('laser_fire_duration', 1.0)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('weed_z', 0.03)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._fire_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self._det_sub = self.create_subscription(
            Detection2DArray, '/detections', self._detection_cb, 10)

        # Camera intrinsics — populated from CameraInfo; None until received.
        # Ray-cast and centering are blocked until this is set.
        self._intrinsics = None
        self._intrinsics_received_at = None
        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 1)

        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self._state = State.INIT
        self._scan_idx = 0
        self._current_joints = list(HOME_POSE)
        self._last_weed_det = None
        self._busy = False
        self._wait_start = 0.0
        self._fire_start = 0.0
        self._lock_start = 0.0
        self._treated_pans = []
        self._treated_world_xy = []
        self._joints_received = False
        self._tf_warn_time = 0.0

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'arm_controller_node ready  '
            f'dry_run={self.get_parameter("dry_run").value}  '
            f'scan_poses={len(SCAN_POSES)}  '
            f'dwell={self.get_parameter("scan_dwell_time").value}s  '
            f'fire_dur={self.get_parameter("laser_fire_duration").value}s  '
            f'weed_z={self.get_parameter("weed_z").value}m'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _detection_cb(self, msg: Detection2DArray):
        best = None
        best_conf = 0.0
        weed_count = 0
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id in WEED_CLASS_IDS:
                    weed_count += 1
                    if hyp.hypothesis.score > best_conf:
                        best_conf = hyp.hypothesis.score
                        best = (int(det.bbox.center.position.x),
                                int(det.bbox.center.position.y))
        if weed_count > 0:
            self.get_logger().debug(
                f'Detection: {weed_count} weed(s)  '
                f'best px={best} conf={best_conf:.2f}')
        self._last_weed_det = best

    def _camera_info_cb(self, msg: CameraInfo):
        if len(msg.k) < 9 or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            self.get_logger().warn('Ignoring degenerate CameraInfo (k too short or fx/fy=0)')
            return
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._intrinsics_received_at = self._now()
        self.get_logger().info(
            f'Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
            f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.destroy_subscription(self._info_sub)

    def _joint_state_cb(self, msg: JointState):
        pos_by_name = dict(zip(msg.name, msg.position))
        try:
            self._current_joints = [pos_by_name[n] for n in JOINT_NAMES]
            if not self._joints_received:
                self._joints_received = True
                self.get_logger().info(
                    f'First /joint_states received  '
                    f'pan={self._current_joints[0]:+.3f} rad')
        except KeyError:
            pass

    def _tick(self):
        if self._busy:
            return

        # Warn once if CameraInfo hasn't arrived after the expected window.
        if self._intrinsics is None:
            elapsed = self._now() - (self._intrinsics_received_at or self._now())
            if self._now() > CAMERA_INFO_WARN_SEC and self._intrinsics_received_at is None:
                self.get_logger().warn(
                    f'CameraInfo not received after {CAMERA_INFO_WARN_SEC:.0f}s. '
                    f'Check that /camera/color/camera_info is publishing. '
                    f'Ray-cast and centering are disabled until intrinsics arrive.',
                    throttle_duration_sec=10.0)

        if self._state == State.INIT:
            self.get_logger().info('INIT: moving to HOME pose')
            self._move_to(HOME_POSE, duration_sec=3.0)
            self._state = State.SCAN_MOVE

        elif self._state == State.SCAN_MOVE:
            idx = self._scan_idx % len(SCAN_POSES)
            pose = SCAN_POSES[idx]
            self._scan_idx += 1
            self.get_logger().info(
                f'SCAN pose {idx + 1}/{len(SCAN_POSES)}  pan={pose[0]:+.2f} rad')
            self._state = State.SCANNING
            self._move_to(pose, duration_sec=SCAN_MOVE_TIME,
                          on_done=lambda: self._enter_waiting())

        elif self._state == State.SCANNING:
            pass

        elif self._state == State.WAITING:
            if self._last_weed_det is not None:
                cx, cy = self._last_weed_det
                wxy = self._pixel_to_world_xy(cx, cy)
                if wxy is not None and any(
                    math.hypot(wxy[0] - tx, wxy[1] - ty) < TREATED_WORLD_DIST
                    for tx, ty in self._treated_world_xy
                ):
                    self.get_logger().info(
                        f'Skip detection px=({cx},{cy}): world pos '
                        f'({wxy[0]:.2f},{wxy[1]:.2f}) matches treated weed')
                    self._state = State.SCAN_MOVE
                else:
                    self.get_logger().info(
                        f'Weed spotted at pixel {self._last_weed_det} — centering')
                    self._lock_start = self._now()
                    self._state = State.MOVE_TO_WEED
            elif self._now() - self._wait_start > \
                    self.get_parameter('scan_dwell_time').value:
                self.get_logger().info(
                    f'WAITING: no weed in '
                    f'{self.get_parameter("scan_dwell_time").value:.1f}s '
                    f'— advancing scan')
                self._state = State.SCAN_MOVE

        elif self._state == State.MOVE_TO_WEED:
            if self._last_weed_det is None:
                if self._now() - self._lock_start > WEED_LOCK_TIMEOUT:
                    self.get_logger().warn('Lost the weed — resuming scan')
                    self._state = State.SCAN_MOVE
                return
            cx, cy = self._last_weed_det
            err_x = (cx - IMAGE_W / 2) / (IMAGE_W / 2)
            err_y = (cy - IMAGE_H / 2) / (IMAGE_H / 2)
            self._lock_start = self._now()

            if math.hypot(err_x, err_y) < FIRE_ZONE_FRAC:
                self.get_logger().info(
                    f'Weed centred: err=({err_x:+.3f},{err_y:+.3f}) '
                    f'dist={math.hypot(err_x, err_y):.3f} < {FIRE_ZONE_FRAC} → FIRE_LASER')
                self._state = State.FIRE_LASER
                return

            corrected = list(self._current_joints)
            corrected[3] -= err_x * WRIST1_GAIN
            corrected[4] -= err_y * WRIST2_GAIN
            corrected[3] = max(-2.5, min(-0.8, corrected[3]))
            corrected[4] = max(-2.14, min(-1.00, corrected[4]))
            self.get_logger().info(
                f'Centering: weed px=({cx},{cy}) err=({err_x:+.2f},{err_y:+.2f}) '
                f'w1={corrected[3]:+.3f} w2={corrected[4]:+.3f}')
            self._move_to(corrected, duration_sec=WEED_MOVE_TIME,
                          on_done=lambda: None)

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

        elif self._state == State.FIRING:
            if self._now() - self._fire_start > \
                    self.get_parameter('laser_fire_duration').value:
                self._fire_pub.publish(Bool(data=False))
                fired_pan = self._current_joints[0]
                self._treated_pans.append(fired_pan)
                if self._last_weed_det is not None:
                    wxy = self._pixel_to_world_xy(*self._last_weed_det)
                    if wxy is not None:
                        self._treated_world_xy.append(wxy)
                        self.get_logger().info(
                            f'Treated weed world pos: ({wxy[0]:.2f},{wxy[1]:.2f})')
                self.get_logger().info(
                    f'Weed treated — pan={fired_pan:+.2f} blacklisted '
                    f'(total treated: {len(self._treated_pans)}). Resuming scan.')
                self._state = State.SCAN_MOVE

    def _pixel_to_world_xy(self, cx, cy):
        if self._intrinsics is None:
            self.get_logger().warn(
                'Cannot ray-cast: camera intrinsics not yet received.',
                throttle_duration_sec=5.0)
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', 'camera_link', rclpy.time.Time())
        except Exception as exc:
            now = self._now()
            if now - self._tf_warn_time > 5.0:
                self._tf_warn_time = now
                self.get_logger().warn(f'TF world→camera_link unavailable: {exc}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        fx, fy, cx_opt, cy_opt = self._intrinsics
        weed_z = self.get_parameter('weed_z').value
        ray_cam = np.array([1.0, -(cx - cx_opt) / fx, -(cy - cy_opt) / fy])
        ray_world = R @ ray_cam
        origin = np.array([t.x, t.y, t.z])
        if abs(ray_world[2]) < 1e-6:
            self.get_logger().warn('_pixel_to_world_xy: ray parallel to ground plane')
            return None
        lam = (weed_z - origin[2]) / ray_world[2]
        if lam < 0:
            self.get_logger().warn(
                f'_pixel_to_world_xy: camera looking away from ground '
                f'(lam={lam:.3f}) — check TF/camera orientation')
            return None
        pt = origin + lam * ray_world
        self.get_logger().debug(
            f'World proj: px=({cx},{cy}) → ({pt[0]:.3f},{pt[1]:.3f})m')
        return (float(pt[0]), float(pt[1]))

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [  2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    def _enter_waiting(self):
        current_pan = self._current_joints[0]
        for treated in self._treated_pans:
            if abs(current_pan - treated) < TREATED_ZONE_RAD:
                self.get_logger().info(
                    f'Skip scan pos pan={current_pan:+.2f} — already treated '
                    f'weed at pan={treated:+.2f}')
                self._state = State.SCAN_MOVE
                return
        self._last_weed_det = None
        self._wait_start = self._now()
        self._state = State.WAITING
        self.get_logger().info(
            f'WAITING at pan={current_pan:+.2f} rad '
            f'(dwell={self.get_parameter("scan_dwell_time").value:.1f}s)'
        )

    def _move_to(self, joints, duration_sec=3.0, on_done=None):
        if not self._action.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'Action server /scaled_joint_trajectory_controller/'
                'follow_joint_trajectory NOT ready — arm will not move')
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

        def _result_cb(future):
            self._busy = False
            if on_done:
                on_done()

        def _goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Trajectory goal REJECTED by controller')
                self._busy = False
                return
            self._current_joints = list(joints)
            goal_handle.get_result_async().add_done_callback(_result_cb)

        self._action.send_goal_async(goal).add_done_callback(_goal_response)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
