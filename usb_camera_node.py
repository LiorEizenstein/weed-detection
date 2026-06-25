"""
usb_camera_node — publish a USB/UVC camera to /camera/image_raw.

A pure-Python stand-in for the ros-jazzy-usb-cam driver, used when that
package isn't installed (e.g. no sudo to apt-install it).  Opens the camera
with OpenCV's V4L2 backend and republishes each frame as sensor_msgs/Image
(bgr8) so weed_detection-2.py --topic /camera/image_raw and the rest of the
stack can consume it unchanged.

Run (while nothing else is using the camera):
    python3 ~/ros2_ws/usb_camera_node.py
    python3 ~/ros2_ws/usb_camera_node.py --device 0 --width 640 --height 480 --fps 30

Then, in another terminal:
    python3 ~/ros2_ws/weed_detection-2.py --topic /camera/image_raw \
        --model /home/lior/best.pt --confidence 0.7
"""

import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):

    def __init__(self, device: int, width: int, height: int, fps: float,
                 frame_id: str, topic: str):
        super().__init__('usb_camera')
        self._bridge = CvBridge()
        self._frame_id = frame_id

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f'Could not open camera /dev/video{device}')
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._pub = self.create_publisher(Image, topic, 10)
        self._timer = self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(
            f'usb_camera publishing {actual_w}x{actual_h} @ {fps:.0f}fps '
            f'from /dev/video{device} -> {topic}')

    def _tick(self):
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', type=int, default=0,
                        help='/dev/video<N> index (default: 0)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--frame-id', default='camera_color_optical_frame',
                        help='TF frame stamped on each image')
    parser.add_argument('--topic', default='/camera/image_raw')
    args = parser.parse_args()

    rclpy.init()
    node = UsbCameraNode(args.device, args.width, args.height, args.fps,
                         args.frame_id, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
