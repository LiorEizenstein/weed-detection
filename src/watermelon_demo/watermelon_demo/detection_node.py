"""
detection_node — subscribes to /camera/image_raw, runs YOLO (best.pt) to detect
weeds and watermelons, publishes detections and an annotated image.

Topics published:
  /detections          vision_msgs/Detection2DArray
  /detection_image     sensor_msgs/Image

Topic subscribed:
  /camera/image_raw    sensor_msgs/Image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


WEED_CLASSES = {1, 2}          # weed_side=1, weed_top=2
CONFIDENCE_THRESHOLD = 0.7
SOIL_OFFSET_FACTOR = 0.08
TOP_Y_FACTOR = 0.55

# HSV stub thresholds for the brown/orange weed cylinders (use_real_model=false).
# Wide enough to survive Gazebo lighting, but hue<=30 excludes green watermelons
# and sat>=50 excludes the grey ground plane.
WEED_HSV_LOW  = (5, 50, 40)
WEED_HSV_HIGH = (30, 255, 255)
MIN_CONTOUR_AREA = 40          # px^2 — weeds are small/far, so keep this low


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('use_real_model', False)
        self.declare_parameter('model_path', '/home/lior/best.pt')

        self._bridge = CvBridge()
        self._model = None
        self._load_model()

        self._sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)
        self._det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self._img_pub = self.create_publisher(Image, '/detection_image', 10)

    def _load_model(self):
        use_real = self.get_parameter('use_real_model').value
        path = self.get_parameter('model_path').value
        if use_real:
            try:
                from ultralytics import YOLO
                self._model = YOLO(path)
                self.get_logger().info(f'YOLO model loaded from {path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().info('Using HSV color stub (use_real_model=false)')

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self._model is not None:
            detections, annotated = self._run_yolo(frame)
        else:
            detections, annotated = self._run_stub(frame)
        self._det_pub.publish(detections)
        self._img_pub.publish(self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

        n_weeds = sum(
            1 for d in detections.detections for h in d.results
            if h.hypothesis.class_id in {'1', '2'})
        self.get_logger().info(
            f'image {frame.shape[1]}x{frame.shape[0]} | '
            f'{len(detections.detections)} detection(s), {n_weeds} weed(s)',
            throttle_duration_sec=2.0)

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

    def _run_stub(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        array_msg = Detection2DArray()
        annotated = frame.copy()

        weed_mask = cv2.inRange(hsv,
                                np.array(WEED_HSV_LOW),
                                np.array(WEED_HSV_HIGH))
        for contour in cv2.findContours(
                weed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
            array_msg.detections.append(
                self._make_detection(1, 0.85, cx, cy, w, h))
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 220), 2)
            cv2.putText(annotated, 'weed_stub', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 220), 1)

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
