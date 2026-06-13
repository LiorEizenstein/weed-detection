"""
arm_controller_node — state machine that moves the UR5 arm across scan poses,
detects weeds via /detections, centres the camera over them, then fires the laser.

State machine:
  INIT → SCAN_MOVE → WAITING_DETECTION → MOVE_TO_WEED → FIRE_LASER
       → RETURN_HOME → SCAN_MOVE (loops)

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

# Pre-defined scan poses (radians) covering the four field sectors
SCAN_POSES = [
    [0.3,  -1.2,  1.4, -1.8, -1.57, 0.0],
    [0.0,  -1.2,  1.4, -1.8, -1.57, 0.0],
    [-0.3, -1.2,  1.4, -1.8, -1.57, 0.0],
    [0.6,  -1.2,  1.4, -1.8, -1.57, 0.0],
]

HOME_POSE = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

IMAGE_W = 640
IMAGE_H = 480
PAN_GAIN  = 0.3   # rad per normalised pixel error
LIFT_GAIN = 0.2
FIRE_ZONE_FRAC = 0.05  # weed must be within 5% of image centre to fire

WEED_CLASS_IDS = {'1', '2'}   # weed_side, weed_top


class State:
    INIT             = 'INIT'
    SCAN_MOVE        = 'SCAN_MOVE'
    SCANNING         = 'SCANNING'   # holding at a scan pose until dwell completes
    WAITING          = 'WAITING_DETECTION'
    MOVE_TO_WEED     = 'MOVE_TO_WEED'
    FIRE_LASER       = 'FIRE_LASER'
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

        self._timer = self.create_timer(0.1, self._tick)

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
            self._move_to(HOME_POSE, duration_sec=3.0)
            self._state = State.SCAN_MOVE

        elif self._state == State.SCAN_MOVE:
            pose = SCAN_POSES[self._scan_idx % len(SCAN_POSES)]
            self._scan_idx += 1
            # Hold in SCANNING until the move + dwell finish, so tick does not
            # immediately fire the next scan move before we look for a weed.
            self._state = State.SCANNING
            self._move_to(pose, duration_sec=3.0,
                          on_done=lambda: self._enter_waiting())

        elif self._state == State.SCANNING:
            pass  # _enter_waiting (dwell timer) drives the transition to WAITING

        elif self._state == State.WAITING:
            if self._last_weed_det is not None:
                self._state = State.MOVE_TO_WEED

        elif self._state == State.MOVE_TO_WEED:
            if self._last_weed_det is None:
                # Detection dropped out between frames — go back to looking.
                self._state = State.WAITING
                return
            cx, cy = self._last_weed_det
            err_x = (cx - IMAGE_W / 2) / (IMAGE_W / 2)
            err_y = (cy - IMAGE_H / 2) / (IMAGE_H / 2)

            fire_zone = FIRE_ZONE_FRAC
            if abs(err_x) < fire_zone and abs(err_y) < fire_zone:
                self._state = State.FIRE_LASER
                return

            corrected = list(self._current_joints)
            corrected[0] -= err_x * PAN_GAIN
            corrected[1] -= err_y * LIFT_GAIN
            corrected[0] = max(-math.pi, min(math.pi, corrected[0]))
            self._move_to(corrected, duration_sec=1.5,
                          on_done=lambda: None)
            self._last_weed_det = None

        elif self._state == State.FIRE_LASER:
            self.get_logger().info('Firing laser at weed')
            msg = Bool()
            msg.data = True
            self._fire_pub.publish(msg)
            duration = self.get_parameter('laser_fire_duration').value
            self.create_timer(duration, self._after_fire)
            self._state = State.RETURN_HOME

        elif self._state == State.RETURN_HOME:
            self._move_to(HOME_POSE, duration_sec=3.0,
                          on_done=lambda: self._next_scan())

    def _enter_waiting(self):
        dwell = self.get_parameter('scan_dwell_time').value
        self._last_weed_det = None
        self.create_timer(dwell, lambda: setattr(self, '_state', State.WAITING))

    def _after_fire(self):
        msg = Bool()
        msg.data = False
        self._fire_pub.publish(msg)

    def _next_scan(self):
        self._state = State.SCAN_MOVE

    # ------------------------------------------------------------------ #
    #  Action helper                                                       #
    # ------------------------------------------------------------------ #

    def _move_to(self, joints, duration_sec=3.0, on_done=None):
        if not self._action.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not ready')
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

        send_future = self._action.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(_result_cb))


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
