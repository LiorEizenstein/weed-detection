"""
arm_controller_node — state machine that moves the UR5 arm across scan poses,
detects weeds via /detections, centres the camera over them, then fires the laser.

State machine (shoulder_pan sweeps a wide arc across the field):
  INIT → SCAN_MOVE → SCANNING → WAITING_DETECTION
       → (weed seen)  MOVE_TO_WEED → FIRE_LASER → FIRING → RETURN_HOME → SCAN_MOVE
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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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
PAN_GAIN  = 0.25  # rad per normalised pixel error
LIFT_GAIN = 0.18
FIRE_ZONE_FRAC = 0.10  # weed within 10% of image centre counts as "under camera"

WEED_CLASS_IDS = {'1', '2'}   # weed_side, weed_top


class State:
    INIT             = 'INIT'
    SCAN_MOVE        = 'SCAN_MOVE'
    SCANNING         = 'SCANNING'   # arm moving to a scan pose
    WAITING          = 'WAITING_DETECTION'
    MOVE_TO_WEED     = 'MOVE_TO_WEED'
    FIRE_LASER       = 'FIRE_LASER'
    FIRING           = 'FIRING'     # beam on, waiting out laser_fire_duration
    RETURN_HOME      = 'RETURN_HOME'


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self.declare_parameter('scan_dwell_time', 2.0)
        self.declare_parameter('laser_fire_duration', 1.0)

        self._action = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._fire_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self._det_sub = self.create_subscription(
            Detection2DArray, '/detections', self._detection_cb, 10)

        self._state = State.INIT
        self._scan_idx = 0
        self._current_joints = list(HOME_POSE)
        self._last_weed_det = None   # (cx, cy) pixel of best weed
        self._busy = False           # waiting for action result
        self._wait_start = 0.0       # sim time we entered WAITING at a pose
        self._fire_start = 0.0       # sim time the laser turned on
        self._lock_start = 0.0       # sim time we locked onto a weed

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

            # Weed is under the camera -> stop and fire.
            if abs(err_x) < FIRE_ZONE_FRAC and abs(err_y) < FIRE_ZONE_FRAC:
                self._state = State.FIRE_LASER
                return

            self.get_logger().info(
                f'Centering: weed px=({cx},{cy}) err=({err_x:+.2f},{err_y:+.2f})',
                throttle_duration_sec=0.5)
            corrected = list(self._current_joints)
            corrected[0] -= err_x * PAN_GAIN
            corrected[1] -= err_y * LIFT_GAIN
            corrected[0] = max(-math.pi, min(math.pi, corrected[0]))
            self._move_to(corrected, duration_sec=WEED_MOVE_TIME,
                          on_done=lambda: None)

        elif self._state == State.FIRE_LASER:
            self.get_logger().info('Firing laser at weed')
            self._fire_pub.publish(Bool(data=True))
            self._fire_start = self._now()
            self._state = State.FIRING

        elif self._state == State.FIRING:
            if self._now() - self._fire_start > \
                    self.get_parameter('laser_fire_duration').value:
                self._fire_pub.publish(Bool(data=False))
                self._state = State.RETURN_HOME

        elif self._state == State.RETURN_HOME:
            self.get_logger().info('Weed treated — returning HOME, then resuming scan')
            self._move_to(HOME_POSE, duration_sec=3.0,
                          on_done=lambda: self._next_scan())

    def _enter_waiting(self):
        self._last_weed_det = None
        self._wait_start = self._now()
        self._state = State.WAITING

    def _next_scan(self):
        self._state = State.SCAN_MOVE

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
        self._current_joints = list(joints)

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
            goal_handle.get_result_async().add_done_callback(_result_cb)

        self._action.send_goal_async(goal).add_done_callback(_goal_response)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
