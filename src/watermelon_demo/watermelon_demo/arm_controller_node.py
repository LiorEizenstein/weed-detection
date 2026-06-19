"""
arm_controller_node — state machine that moves the UR5 arm across scan poses,
detects weeds via /detections, centres the camera over them, then fires the laser.

State machine (shoulder_pan sweeps a wide arc across the field):
  INIT → SCAN_MOVE → SCANNING → WAITING_DETECTION
       → (weed seen)  MOVE_TO_WEED → FIRE_LASER → FIRING → SCAN_MOVE
       → (timed out)  SCAN_MOVE   (advance to next pose, never stalls)

Topics subscribed:
  /detections    vision_msgs/Detection2DArray

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

# Scan poses (radians). The arm reaches forward + down (shoulder lift/elbow/wrist
# held constant) while shoulder_pan_joint sweeps left -> right in SMALL steps, so
# the camera glides across the field instead of jumping. Pan spans +1.5 -> -1.5
# rad (~170 deg) in PAN_STEP increments.
PAN_MAX  = 1.5     # far left
PAN_MIN  = -1.5    # far right
PAN_STEP = 0.15    # rad between scan poses (~8.6 deg) — smaller = finer sweep
_N_SCAN  = int(round((PAN_MAX - PAN_MIN) / PAN_STEP)) + 1
SCAN_POSES = [
    [round(PAN_MAX - i * PAN_STEP, 3), -1.2, 1.4, -1.8, -1.57, 0.0]
    for i in range(_N_SCAN)
]

HOME_POSE = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

SCAN_MOVE_TIME = 1.2   # s per scan step (short, since steps are small)
WEED_MOVE_TIME = 1.0   # s per centering correction
WEED_LOCK_TIMEOUT = 8.0  # s to keep chasing a weed before giving up

IMAGE_W = 640
IMAGE_H = 480

# Gazebo camera intrinsics (horizontal_fov=1.047) — used as fallback until a
# live CameraInfo message arrives from the real RealSense.
_HFOV = 1.047
_FX = (IMAGE_W / 2) / math.tan(_HFOV / 2)
_FY = _FX
_CX_OPT = IMAGE_W / 2.0
_CY_OPT = IMAGE_H / 2.0
_WEED_Z = 0.03   # weed top height in world frame (m)

# World-space exclusion radius around each treated weed.  Adjacent weeds are
# ~0.30 m apart in this field; 0.20 m catches the same weed from any scan
# angle without blacklisting live neighbours.
TREATED_WORLD_DIST = 0.20   # metres

# Centering gains.
# Empirically (from logs): wrist_1 at wrist_2=-π/2 moves image X with ~0.12/rad
# Y cross-coupling, while wrist_2 moves image Y with near-zero X cross-coupling.
# This makes them an orthogonal pair — both axes can be corrected simultaneously
# without the runaway coupling that shoulder_pan caused (pan had ~4.7/rad Y leak).
WRIST1_GAIN = 0.08  # rad per normalised pixel error (wrist_1 drives image X)
WRIST2_GAIN = 0.10  # rad per normalised pixel error (wrist_2 drives image Y)
FIRE_ZONE_FRAC = 0.20  # weed within 20% of image centre → good enough, fire

# Dead-weed suppression: ignore detections within this many radians of a treated
# weed's pan angle.  0.35 covers ~2.3 PAN_STEPs, handling the same weed visible
# from two adjacent scan positions.
TREATED_ZONE_RAD = 0.35

WEED_CLASS_IDS = {'1', '2'}   # weed_side, weed_top


class State:
    INIT         = 'INIT'
    SCAN_MOVE    = 'SCAN_MOVE'
    SCANNING     = 'SCANNING'   # arm moving to a scan pose
    WAITING      = 'WAITING_DETECTION'
    MOVE_TO_WEED = 'MOVE_TO_WEED'
    FIRE_LASER   = 'FIRE_LASER'
    FIRING       = 'FIRING'     # beam on, waiting out laser_fire_duration


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self.declare_parameter('scan_dwell_time', 2.0)
        self.declare_parameter('laser_fire_duration', 1.0)
        self.declare_parameter('dry_run', False)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._fire_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self._det_sub = self.create_subscription(
            Detection2DArray, '/detections', self._detection_cb, 10)

        # Live camera intrinsics — updated once from CameraInfo, then unsubscribed.
        # Falls back to Gazebo defaults if the topic never arrives (simulation mode).
        self._intrinsics = (_FX, _FY, _CX_OPT, _CY_OPT)
        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 1)

        # Real joint positions from hardware — continuously overwrites the
        # dead-reckoned value so centering corrections start from the true pose.
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self._state = State.INIT
        self._scan_idx = 0
        self._current_joints = list(HOME_POSE)
        self._last_weed_det = None   # (cx, cy) pixel of best weed
        self._busy = False           # waiting for action result
        self._wait_start = 0.0       # sim time we entered WAITING at a pose
        self._fire_start = 0.0       # sim time the laser turned on
        self._lock_start = 0.0       # sim time we locked onto a weed
        self._treated_pans = []      # shoulder_pan angles where laser already fired
        self._treated_world_xy = []  # (x, y) world positions of treated weeds

        self._timer = self.create_timer(0.1, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ #
    #  Detection callback                                                  #
    # ------------------------------------------------------------------ #

    def _detection_cb(self, msg: Detection2DArray):
        best = None
        best_conf = 0.0
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id in WEED_CLASS_IDS:
                    if hyp.hypothesis.score > best_conf:
                        best_conf = hyp.hypothesis.score
                        best = (int(det.bbox.center.position.x),
                                int(det.bbox.center.position.y))
        self._last_weed_det = best

    def _camera_info_cb(self, msg: CameraInfo):
        if len(msg.k) < 6 or msg.k[0] == 0.0:
            self.get_logger().warn('Ignoring degenerate CameraInfo (k too short or fx=0)')
            return
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.get_logger().info(
            f'Camera intrinsics: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
            f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.destroy_subscription(self._info_sub)

    def _joint_state_cb(self, msg: JointState):
        pos_by_name = dict(zip(msg.name, msg.position))
        try:
            self._current_joints = [pos_by_name[n] for n in JOINT_NAMES]
        except KeyError:
            pass  # partial message — not all joints published yet

    # ------------------------------------------------------------------ #
    #  State machine tick                                                  #
    # ------------------------------------------------------------------ #

    def _tick(self):
        if self._busy:
            return

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
            # Hold in SCANNING until the move finishes, so tick does not
            # immediately fire the next scan move before we look for a weed.
            self._state = State.SCANNING
            self._move_to(pose, duration_sec=SCAN_MOVE_TIME,
                          on_done=lambda: self._enter_waiting())

        elif self._state == State.SCANNING:
            pass  # _enter_waiting (move on_done) drives the transition to WAITING

        elif self._state == State.WAITING:
            # Look for a weed; if none appears within scan_dwell_time, sweep on
            # to the next scan pose instead of stalling here forever.
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
                self._state = State.SCAN_MOVE

        elif self._state == State.MOVE_TO_WEED:
            # Stay locked on the weed. If detection drops out briefly, keep
            # waiting for a fresh frame; give up only after WEED_LOCK_TIMEOUT.
            if self._last_weed_det is None:
                if self._now() - self._lock_start > WEED_LOCK_TIMEOUT:
                    self.get_logger().warn('Lost the weed — resuming scan')
                    self._state = State.SCAN_MOVE
                return
            cx, cy = self._last_weed_det
            err_x = (cx - IMAGE_W / 2) / (IMAGE_W / 2)
            err_y = (cy - IMAGE_H / 2) / (IMAGE_H / 2)

            # Fresh detection — reset loss-timer so active centering moves don't
            # drain the timeout budget (each 1-s move would otherwise eat into 8 s).
            self._lock_start = self._now()

            # Weed is under the camera → fire.  Use circular threshold to avoid
            # firing 41% off-axis when the weed sits in a corner of the square zone.
            if math.hypot(err_x, err_y) < FIRE_ZONE_FRAC:
                self._state = State.FIRE_LASER
                return

            corrected = list(self._current_joints)
            # Correct both axes simultaneously: wrist_1 for X, wrist_2 for Y.
            # These two joints are nearly orthogonal — wrist_1 barely shifts image
            # Y and wrist_2 barely shifts image X, so simultaneous correction
            # converges without the cross-axis runaway that shoulder_pan caused.
            corrected[3] -= err_x * WRIST1_GAIN
            corrected[4] -= err_y * WRIST2_GAIN
            corrected[3] = max(-2.5, min(-0.8, corrected[3]))
            # wrist_2 scan pose is -1.57; allow ±0.57 rad for centering range
            # while keeping the joint well clear of singularity/collision territory.
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

    # ------------------------------------------------------------------ #
    #  Geometry helpers                                                    #
    # ------------------------------------------------------------------ #

    def _pixel_to_world_xy(self, cx, cy):
        """Project image pixel to world XY on the weed ground plane. Returns (x,y) or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', 'camera_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        fx, fy, cx_opt, cy_opt = self._intrinsics
        ray_cam = np.array([1.0, -(cx - cx_opt) / fx, -(cy - cy_opt) / fy])
        ray_world = R @ ray_cam
        origin = np.array([t.x, t.y, t.z])
        if abs(ray_world[2]) < 1e-6:
            return None
        lam = (_WEED_Z - origin[2]) / ray_world[2]
        if lam < 0:
            return None
        pt = origin + lam * ray_world
        return (float(pt[0]), float(pt[1]))

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [  2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    def _enter_waiting(self):
        # Skip this scan position if we already treated a weed nearby.
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

    # ------------------------------------------------------------------ #
    #  Action helper                                                       #
    # ------------------------------------------------------------------ #

    def _move_to(self, joints, duration_sec=3.0, on_done=None):
        if not self._action.wait_for_server(timeout_sec=1.0):
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
        # _current_joints is updated only after the controller accepts the goal.
        # Writing it before acceptance created ghost state: a rejected goal left
        # the joints at the intended-but-never-executed pose, and the next
        # correction step compounded the error from that phantom position.

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
            self._current_joints = list(joints)   # commit only on acceptance
            goal_handle.get_result_async().add_done_callback(_result_cb)

        self._action.send_goal_async(goal).add_done_callback(_goal_response)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
