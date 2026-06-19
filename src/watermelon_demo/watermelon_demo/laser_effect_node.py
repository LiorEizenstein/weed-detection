"""
laser_effect_node — on /laser_fire True, computes the Tm:YLF beam vector from
the laser (co-mounted with the camera on tool0) to the detected weed's 3D world
position, then publishes a red RViz arrow marker showing the firing beam.

Architecture:
  1. Listen for /laser_fire (std_msgs/Bool)
  2. Cache latest weed pixel from /detections
  3. On fire: TF lookup camera_link → world, project pixel → ground plane
  4. TF lookup laser_link → world to get current laser origin (arm may have moved)
  5. Draw ARROW marker from laser_link world position → weed xyz

Topics subscribed:
  /laser_fire      std_msgs/Bool
  /detections      vision_msgs/Detection2DArray

Topics published:
  /visualization_marker    visualization_msgs/Marker
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import CameraInfo


# Camera intrinsics — Gazebo fallback (640×480, hfov=1.047 rad).
# Overridden at runtime by CameraInfo messages (real RealSense D435).
IMG_W = 640
IMG_H = 480
HFOV  = 1.047
FX = (IMG_W / 2) / math.tan(HFOV / 2)   # ≈ 554.3 px
FY = FX
CX = IMG_W / 2.0
CY = IMG_H / 2.0

WEED_CLASS_IDS = {'1', '2'}
WEED_GROUND_Z  = 0.03   # weed top height in world frame


class LaserEffectNode(Node):

    def __init__(self):
        super().__init__('laser_effect_node')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._fire_sub = self.create_subscription(
            Bool, '/laser_fire', self._fire_cb, 10)
        self._det_sub = self.create_subscription(
            Detection2DArray, '/detections', self._detection_cb, 10)
        self._marker_pub = self.create_publisher(
            Marker, '/visualization_marker', 10)

        self._last_weed_pixel = None   # (cx, cy) in image coords

        # Intrinsics — start with Gazebo fallback, updated by CameraInfo
        self._fx = FX
        self._fy = FY
        self._cx = CX
        self._cy = CY

        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._camera_info_cb, 1)

    # ------------------------------------------------------------------ #

    def _detection_cb(self, msg: Detection2DArray):
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id in WEED_CLASS_IDS:
                    self._last_weed_pixel = (
                        det.bbox.center.position.x,
                        det.bbox.center.position.y,
                    )
                    return

    def _camera_info_cb(self, msg):
        """Update ray-cast intrinsics from RealSense calibration."""
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

    def _fire_cb(self, msg: Bool):
        if not msg.data:
            return
        if self._last_weed_pixel is None:
            self.get_logger().warn('laser_fire received but no weed pixel cached')
            return

        # Get laser world position via TF — laser_link moves with the arm
        try:
            laser_tf = self._tf_buffer.lookup_transform(
                'world', 'laser_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'laser_link TF lookup failed: {e}')
            return
        lt = laser_tf.transform.translation
        laser_origin = np.array([lt.x, lt.y, lt.z])

        weed_world = self._pixel_to_world(*self._last_weed_pixel)
        if weed_world is None:
            return

        self._publish_beam(laser_origin, weed_world)

    # ------------------------------------------------------------------ #
    #  Geometry                                                            #
    # ------------------------------------------------------------------ #

    def _pixel_to_world(self, u: float, v: float):
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', 'camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

        # camera_link frame: +X = optical axis (Gazebo convention), -Y = image right,
        # -Z = image down.  Do NOT use z-forward — that's the ROS optical convention
        # but camera_link has +X forward here.
        ray_cam = np.array([
            1.0,
            -(u - self._cx) / self._fx,
            -(v - self._cy) / self._fy,
        ])

        # Rotation from camera frame to world frame
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)

        ray_world = R @ ray_cam
        origin = np.array([t.x, t.y, t.z])

        self.get_logger().info(
            f'[laser dbg] camera_link world pos=({origin[0]:.3f},{origin[1]:.3f},{origin[2]:.3f}) '
            f'ray_world=({ray_world[0]:.3f},{ray_world[1]:.3f},{ray_world[2]:.3f})')

        # Intersect ray with horizontal plane z = WEED_GROUND_Z
        if abs(ray_world[2]) < 1e-6:
            self.get_logger().warn('Ray is parallel to ground — ray_world_z≈0 suggests wrong camera frame convention')
            return None
        lam = (WEED_GROUND_Z - origin[2]) / ray_world[2]
        if lam < 0:
            self.get_logger().warn(f'Weed projected behind camera (lam={lam:.3f})')
            return None

        point = origin + lam * ray_world
        self.get_logger().info(
            f'[laser dbg] pixel=({u:.0f},{v:.0f}) lam={lam:.3f} → '
            f'weed_world=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f})')
        return point  # [x, y, z]

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),   1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    # ------------------------------------------------------------------ #
    #  Marker                                                              #
    # ------------------------------------------------------------------ #

    def _publish_beam(self, laser_origin: np.ndarray, weed_xyz: np.ndarray):
        beam_len = float(np.linalg.norm(weed_xyz - laser_origin))
        if beam_len > 5.0:
            self.get_logger().warn(
                f'[laser dbg] beam length {beam_len:.2f}m looks wrong (>5m) — '
                f'check camera_link TF convention. '
                f'laser=({laser_origin[0]:.3f},{laser_origin[1]:.3f},{laser_origin[2]:.3f}) '
                f'weed=({weed_xyz[0]:.3f},{weed_xyz[1]:.3f},{weed_xyz[2]:.3f})')
        else:
            self.get_logger().info(
                f'[laser dbg] beam length={beam_len:.3f}m — looks reasonable')
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'laser_beam'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        start = Point(x=float(laser_origin[0]),
                      y=float(laser_origin[1]),
                      z=float(laser_origin[2]))
        end   = Point(x=float(weed_xyz[0]),
                      y=float(weed_xyz[1]),
                      z=float(weed_xyz[2]))
        m.points = [start, end]

        m.scale.x = 0.02   # shaft diameter
        m.scale.y = 0.04   # head diameter
        m.scale.z = 0.05   # head length

        m.color = ColorRGBA(r=1.0, g=0.1, b=0.0, a=0.9)
        m.lifetime = Duration(sec=1, nanosec=500_000_000)

        self._marker_pub.publish(m)
        self.get_logger().info(
            f'Laser beam: ({laser_origin[0]:.3f},{laser_origin[1]:.3f},{laser_origin[2]:.3f}) → '
            f'({weed_xyz[0]:.3f},{weed_xyz[1]:.3f},{weed_xyz[2]:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = LaserEffectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
