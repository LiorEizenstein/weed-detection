"""
detection_node — subscribes to /camera/color/image_raw, runs YOLO (best.pt) to
detect weeds and watermelons, publishes detections and an annotated image.

Topics published:
  /detections          vision_msgs/Detection2DArray
  /detection_image     sensor_msgs/Image

Topic subscribed:
  /camera/color/image_raw    sensor_msgs/Image
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2


WEED_CLASSES = {1, 2}          # weed_side=1, weed_top=2
CONFIDENCE_THRESHOLD = 0.7
SOIL_OFFSET_FACTOR = 0.08
TOP_Y_FACTOR = 0.55


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('model_path', '/home/lior/best.pt')
        self.declare_parameter('save_debug_frames', False)
        self.declare_parameter(
            'debug_frames_dir',
            os.path.expanduser('~/ros2_ws/run_logs/frames'))

        self._bridge = CvBridge()
        self._model = None
        self._load_model()

        self._frame_count = 0
        self._save_idx = 0
        self._debug_dir = self.get_parameter('debug_frames_dir').value
        if self.get_parameter('save_debug_frames').value:
            os.makedirs(self._debug_dir, exist_ok=True)
            self.get_logger().info(f'Saving debug frames to {self._debug_dir}')

        self._sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, 10)
        self._det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self._img_pub = self.create_publisher(Image, '/detection_image', 10)

    def _load_model(self):
        path = self.get_parameter('model_path').value
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self.get_logger().info(f'YOLO model loaded from {path}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to load YOLO model from {path}: {e}\n'
                f'Detection will publish empty arrays until the model is available.\n'
                f'Fix: place best.pt at {path} or pass model_path:=<path> at launch.')

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self._model is not None:
            detections, annotated = self._run_yolo(frame)
        else:
            detections = Detection2DArray()
            annotated = frame.copy()

        self._det_pub.publish(detections)
        self._img_pub.publish(self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

        n_weeds = sum(
            1 for d in detections.detections for h in d.results
            if h.hypothesis.class_id in {'1', '2'})
        b, g, r = (int(frame[:, :, 0].mean()),
                   int(frame[:, :, 1].mean()),
                   int(frame[:, :, 2].mean()))
        self.get_logger().info(
            f'frame#{self._frame_count} {frame.shape[1]}x{frame.shape[0]} '
            f'meanBGR=({b},{g},{r}) | {len(detections.detections)} det, '
            f'{n_weeds} weed(s)',
            throttle_duration_sec=2.0)

        if self.get_parameter('save_debug_frames').value \
                and self._frame_count % 15 == 0:
            i = self._save_idx % 12
            cv2.imwrite(os.path.join(self._debug_dir, f'raw_{i:02d}.png'), frame)
            cv2.imwrite(os.path.join(self._debug_dir, f'annotated_{i:02d}.png'),
                        annotated)
            self._save_idx += 1

    def _run_yolo(self, frame):
        results = self._model.predict(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
        array_msg = Detection2DArray()
        annotated = frame.copy()
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            box_h = y2 - y1
            cx = int((x1 + x2) / 2)
            if cls_id == 1:
                cy = int(y2 - SOIL_OFFSET_FACTOR * box_h)
            else:
                cy = int(y1 + TOP_Y_FACTOR * box_h)
            array_msg.detections.append(
                self._make_detection(cls_id, conf, cx, cy, x2 - x1, box_h))
            color = (0, 0, 220) if cls_id in WEED_CLASSES else (0, 180, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f'{self._model.names[cls_id]} {conf:.2f}'
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 255), -1)
        return array_msg, annotated

    @staticmethod
    def _make_detection(cls_id: int, conf: float,
                        cx: int, cy: int, w: int, h: int) -> Detection2D:
        det = Detection2D()
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(cls_id)
        hyp.hypothesis.score = conf
        det.results.append(hyp)
        return det


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
