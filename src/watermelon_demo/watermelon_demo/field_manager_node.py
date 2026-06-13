"""
field_manager_node — publishes a MarkerArray showing plant positions from the SDF.
Weeds turn grey when burned; the laser mount is shown as a reference marker.

Colors:
  dark green  = watermelon (static)
  red         = untreated weed
  grey        = treated weed (after /laser_fire True near that weed)
  dark red box = laser mount position

Topics subscribed:
  /laser_fire    std_msgs/Bool

Topics published:
  /field_markers    visualization_msgs/MarkerArray
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3


# Mirrors watermelon_field.sdf exactly
WATERMELONS = {
    0: (0.5,  0.2,  0.08),
    1: (0.5, -0.2,  0.08),
    2: (0.6,  0.0,  0.08),
    3: (0.4,  0.4,  0.08),
}

WEEDS = {
    10: (0.55,  0.35, 0.03),
    11: (0.45, -0.35, 0.03),
    12: (0.65, -0.1,  0.03),
}

# The laser is shown via the robot model (laser_link on base_link), not a marker.

COLOR_WATERMELON = ColorRGBA(r=0.0,  g=0.5,  b=0.0,  a=0.9)
COLOR_WEED       = ColorRGBA(r=0.8,  g=0.1,  b=0.1,  a=0.9)
COLOR_TREATED    = ColorRGBA(r=0.4,  g=0.4,  b=0.4,  a=0.7)


class FieldManagerNode(Node):

    def __init__(self):
        super().__init__('field_manager_node')

        self._treated = set()   # IDs of burned weeds

        self._fire_sub = self.create_subscription(
            Bool, '/laser_fire', self._fire_cb, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/field_markers', 10)

        self._timer = self.create_timer(1.0, self._publish_markers)

    def _fire_cb(self, msg: Bool):
        if not msg.data:
            return
        # Mark the closest untreated weed as treated
        for wid in WEEDS:
            if wid not in self._treated:
                self._treated.add(wid)
                self.get_logger().info(f'Weed {wid} marked as treated')
                break

    def _publish_markers(self):
        array = MarkerArray()

        for mid, (x, y, z) in WATERMELONS.items():
            m = self._sphere(mid, x, y, z, radius=0.08, color=COLOR_WATERMELON)
            array.markers.append(m)

        for wid, (x, y, z) in WEEDS.items():
            color = COLOR_TREATED if wid in self._treated else COLOR_WEED
            m = self._cylinder(wid, x, y, z,
                               radius=0.03, height=0.06, color=color)
            array.markers.append(m)

        self._marker_pub.publish(array)

    # ------------------------------------------------------------------ #

    def _base_marker(self, mid: int, x, y, z) -> Marker:
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'field'
        m.id = mid
        m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=float(z))
        m.pose.orientation.w = 1.0
        return m

    def _sphere(self, mid, x, y, z, radius, color) -> Marker:
        m = self._base_marker(mid, x, y, z)
        m.type = Marker.SPHERE
        m.scale = Vector3(x=radius * 2, y=radius * 2, z=radius * 2)
        m.color = color
        return m

    def _cylinder(self, mid, x, y, z, radius, height, color) -> Marker:
        m = self._base_marker(mid, x, y, z)
        m.type = Marker.CYLINDER
        m.scale = Vector3(x=radius * 2, y=radius * 2, z=height)
        m.color = color
        return m

    def _box(self, mid, x, y, z, sx, sy, sz, color) -> Marker:
        m = self._base_marker(mid, x, y, z)
        m.type = Marker.CUBE
        m.scale = Vector3(x=sx, y=sy, z=sz)
        m.color = color
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FieldManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
