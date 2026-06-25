"""
Save one frame from /camera/image_raw to /tmp/camera_frame.png
Run while the simulation is running:
  python3 ~/ros2_ws/grab_camera_frame.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrameGrabber(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self._bridge = CvBridge()
        self._done = False
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
        self.get_logger().info('Waiting for a camera frame...')

    def _cb(self, msg):
        if self._done:
            return
        self._done = True
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        path = '/tmp/camera_frame.png'
        cv2.imwrite(path, img)
        self.get_logger().info(f'Saved frame to {path}  ({img.shape[1]}x{img.shape[0]})')
        raise SystemExit

def main():
    rclpy.init()
    node = FrameGrabber()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
