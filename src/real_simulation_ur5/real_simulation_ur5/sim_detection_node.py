"""
sim_detection_node — adapts the weed_detection.py YOLO logic as a ROS 2 node.

If best.pt exists at model_path:
  • Runs YOLO on each camera frame (same class/confidence/aiming logic as your script)
  • Publishes detections + annotated image

Otherwise (no model file):
  • Falls back to a probability-based stub that randomly generates weed detections
  • Lets you see the full arm state-machine without needing the YOLO weights

Topics subscribed:
  /camera/image_raw    sensor_msgs/Image   (from Gazebo bridge)

Topics published:
  /detections          vision_msgs/Detection2DArray
  /detection_image     sensor_msgs/Image   (view with rqt_image_view or RViz)
"""

import os
import random

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

# Same constants as your weed_detection.py
WEED_CLASSES      = {1, 2}          # 1=weed_side, 2=weed_top
SOIL_OFFSET_FACTOR = 0.08           # aim point for weed_side
TOP_Y_FACTOR       = 0.55           # aim point for weed_top
FIRE_ZONE_W_FACTOR = 0.05
FIRE_ZONE_H_FACTOR = 0.05


class SimDetectionNode(Node):

    def __init__(self):
        super().__init__('sim_detection_node')

        self.declare_parameter('model_path', '/home/lior/best.pt')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stub_detection_probability', 0.40)
        self.declare_parameter('stub_cooldown_sec', 5.0)

        self._bridge  = CvBridge()
        self._model   = None
        self._names   = {0: 'watermelon', 1: 'weed_side', 2: 'weed_top'}
        self._frame_n = 0
        self._last_stub_t = 0.0

        self._load_model()

        self._sub     = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)
        self._det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        self._img_pub = self.create_publisher(
            Image, '/detection_image', 10)

        mode = 'YOLO' if self._model else 'STUB'
        self.get_logger().info(
            f'sim_detection_node ready  mode={mode}  '
            f'model={self.get_parameter("model_path").value}')

    # ── model loading ────────────────────────────────────────────────────────

    def _load_model(self):
        path = self.get_parameter('model_path').value
        if not os.path.exists(path):
            self.get_logger().warn(
                f'Model not found at {path} — using stub detections. '
                'Place best.pt there or pass model_path:=<path> at launch.')
            return
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            if hasattr(self._model, 'names'):
                self._names = {int(k): v for k, v in self._model.names.items()}
            self.get_logger().info(f'YOLO model loaded from {path}')
        except Exception as exc:
            self.get_logger().error(
                f'YOLO load failed ({exc}) — falling back to stub detections')

    # ── image callback ───────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        self._frame_n += 1
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        if self._model is not None:
            detections, annotated = self._yolo_detect(frame, w, h)
        else:
            detections, annotated = self._stub_detect(frame, w, h)

        self._det_pub.publish(detections)
        self._img_pub.publish(
            self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

    # ── YOLO detection (from your weed_detection.py logic) ──────────────────

    def _yolo_detect(self, frame, w, h):
        conf_thresh = self.get_parameter('confidence_threshold').value
        result = self._model.predict(frame, conf=conf_thresh, verbose=False)[0]
        annotated = result.plot()         # draws standard YOLO boxes + labels
        array_msg = Detection2DArray()

        cx_img, cy_img = w // 2, h // 2
        fzw = int(w * FIRE_ZONE_W_FACTOR)
        fzh = int(h * FIRE_ZONE_H_FACTOR)
        cv2.rectangle(annotated,
                      (cx_img - fzw, cy_img - fzh),
                      (cx_img + fzw, cy_img + fzh),
                      (255, 80, 0), 2)

        if result.boxes is None:
            return array_msg, annotated

        label_y = 60
        for b in result.boxes:
            raw_id  = int(b.cls[0])
            conf    = float(b.conf[0])
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            box_h   = y2 - y1
            box_w   = x2 - x1
            cx      = int((x1 + x2) / 2)

            # Aim point: same logic as your script
            if raw_id == 1:   # weed_side — aim near soil base
                cy = int(y2 - SOIL_OFFSET_FACTOR * box_h)
            elif raw_id == 2: # weed_top — aim at top centre
                cy = int(y1 + TOP_Y_FACTOR * box_h)
            else:
                cy = int((y1 + y2) / 2)

            cy = max(0, min(cy, h - 1))
            cv2.circle(annotated, (cx, cy), 7, (0, 0, 255), -1)

            # Fire-zone check — each weed gets its own label line
            in_zone = (abs(cx - cx_img) < fzw and abs(cy - cy_img) < fzh)
            status  = 'FIRE' if in_zone else 'AIM'
            color   = (0, 0, 255) if in_zone else (0, 200, 0)
            name    = self._names.get(raw_id, str(raw_id))
            cv2.putText(annotated, f'{status} {name} {conf:.2f}',
                        (40, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            label_y += 36

            if raw_id in WEED_CLASSES:
                array_msg.detections.append(
                    _make_det(raw_id, conf, cx, cy, box_w, box_h))

        return array_msg, annotated

    # ── Stub detection (random, for use without YOLO weights) ───────────────

    def _stub_detect(self, frame, w, h):
        array_msg = Detection2DArray()
        annotated = frame.copy()
        now       = self.get_clock().now().nanoseconds / 1e9

        cooldown = self.get_parameter('stub_cooldown_sec').value
        prob     = self.get_parameter('stub_detection_probability').value

        if now - self._last_stub_t > cooldown and random.random() < prob:
            self._last_stub_t = now
            # Random weed box in the central region of the image
            bw, bh   = random.randint(60, 100), random.randint(60, 100)
            cx       = random.randint(int(w * 0.25), int(w * 0.75))
            cy       = random.randint(int(h * 0.25), int(h * 0.75))
            cls_id   = random.choice([1, 2])
            conf     = round(random.uniform(0.72, 0.94), 2)
            name     = self._names.get(cls_id, 'weed')

            x1, y1 = max(0, cx - bw // 2), max(0, cy - bh // 2)
            x2, y2 = min(w - 1, cx + bw // 2), min(h - 1, cy + bh // 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 200), 2)
            cv2.circle(annotated, (cx, cy), 7, (0, 255, 255), -1)
            cv2.putText(annotated, f'STUB {name} {conf:.2f}',
                        (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 1)

            array_msg.detections.append(
                _make_det(cls_id, conf, cx, cy, bw, bh))

        # Draw fire zone and mode label
        cx_img, cy_img = w // 2, h // 2
        fzw = int(w * FIRE_ZONE_W_FACTOR)
        fzh = int(h * FIRE_ZONE_H_FACTOR)
        cv2.rectangle(annotated,
                      (cx_img - fzw, cy_img - fzh),
                      (cx_img + fzw, cy_img + fzh),
                      (255, 80, 0), 2)
        cv2.putText(annotated, 'STUB MODE — no best.pt',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)

        return array_msg, annotated


# ── helpers ──────────────────────────────────────────────────────────────────

def _make_det(cls_id: int, conf: float,
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
    node = SimDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
